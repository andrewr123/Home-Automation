/* Zone control for Gt Hall

Version history
---------------

Sep 20 - v1 - preliminary version, based on basic zone controller for comms, plus analogue read of x(O) sensors

*/

#include <avr/wdt.h>
#include <SPI.h>
#include "Ethernet.h"
#define UDP_TX_PACKET_MAX_SIZE 128			// Override the default 24
#include <EthernetUdp.h>

#include "HA_globals.h"
#include "HA_syslog.h"         // For syslog comms
#include "HA_queue.h"          // Needed to support syslog
#include "HA_comms.h"          // For general UDP comms
#include <Wire.h>

#include "HA_time.h"
#include "Time.h"
#include "TimeLib.h"
#include "Wakeup.h"
#include "TimerOne.h"


boolean sendToSyslog = true;

char buffer[UDP_TX_PACKET_MAX_SIZE];        // Max allowed by libraries

// ****** Identity of this controller ******

#define meIP GtHallIP                     
#define meMac GtHallMac
char meName[]									= "Gt Hall";
const static char meInit			= 'G';

// ******  Time ******

// Current time is updated every 30 secs

unsigned int timeNow = 0;

// ******   Internal timing - nothing to do with real time   *******

unsigned long prevTime;
unsigned int heartBeatSecs                 = 0;                
const unsigned int HEARTBEAT_FREQ_MS       = 200;		// 5 heartbeats per second
const unsigned int CHECK_INPUT_FREQ        = 2;			// Check for messages to pass on to boiler every 400mS
const unsigned int GET_STATUS_FREQ         = 50;		// Get status from boiler every 10 secs
const unsigned int CHECK_TIME_FREQ		   = 30;		// Work out current time every 30 secs and act on it

/*
   Code added from https://github.com/Optiboot/optiboot/blob/master/optiboot/examples/test_reset/test_reset.ino

   First, we need a variable to hold the reset cause that can be written before
   early sketch initialization (that might change r2), and won't be reset by the
   various initialization code.
   avr-gcc provides for this via the ".noinit" section.
*/
uint8_t resetFlag __attribute__ ((section(".noinit")));

/*
   Next, we need to put some code to save reset cause from the bootload (in r2)
   to the variable.  Again, avr-gcc provides special code sections for this.
   If compiled with link time optimization (-flto), as done by the Arduno
   IDE version 1.6 and higher, we need the "used" attribute to prevent this
   from being omitted.
*/
void resetFlagsInit(void) __attribute__ ((naked))
__attribute__ ((used))
__attribute__ ((section (".init0")));
void resetFlagsInit(void)
{
  /*
     save the reset flags passed from the bootloader
     This is a "simple" matter of storing (STS) r2 in the special variable
     that we have created.  We use assembler to access the right variable.
  */
  __asm__ __volatile__ ("sts %0, r2\n" : "=m" (resetFlag) :);
}

void setup() {

  // Reset watchdog
   
  MCUSR = 0;
  wdt_disable();

	// Disable the SD CS to avoid interference with UDP transmission - http://arduino.cc/forum/index.php?action=printpage;topic=96651.0
	// And ensure SPI CS pin is set to output to avoid problems with SD card
	pinMode(SD_CS_PIN, OUTPUT);
	digitalWrite(SD_CS_PIN, HIGH);
	pinMode(SPI_CS_PIN, OUTPUT);

	// start the Ethernet connection
	Ethernet.begin(meMac, meIP);

	// Initialise UDP comms
	syslog.init(UdpLogPort, syslogServerIP, syslogServerPort, 'I');
	ard.init(UdpArdPort);

	// Standard setup actions
	setupHA();

  // Start watchdog
  wdt_enable(WDTO_8S);
}


void loop() {

  // Reset watchdog - if fault then this won't happen and watchdog will fire, restarting the sketch
  wdt_reset();

  // Increment heartbeat every second 
  if ((millis() - prevTime) >= HEARTBEAT_FREQ_MS) {   

    prevTime = millis();                  
    heartBeatSecs++;                                          

    // See if any commands from console or other arduinos
    if (heartBeatSecs % CHECK_INPUT_FREQ == 0) acceptInput();

  }
  
  delay(50);
}

void setupHA() {

  int myMCUSR;    // Startup reason

  // Initialise wakeup to support background daemons (eg for time)
  wakeup.init();

  // Get the time from NTP server and log startup
  initialiseTime(UdpNTPPort);
  timeToText(now(), buffer, UDP_TX_PACKET_MAX_SIZE);           // Get current time

  // Log reason for startup to SYSLOG - order of tests is significant; last succesful test is what's reported
  
  if (resetFlag & (1 << WDRF)) myMCUSR = WDRF;
  if (resetFlag & (1 << EXTRF)) myMCUSR = EXTRF;
  if (resetFlag & (1 << PORF)) myMCUSR = PORF;

  switch (myMCUSR) {
    case WDRF: 
      SENDLOG ('I', "Restarted by watchdog @ ", buffer);
      break;
    case EXTRF:
      SENDLOG ('I', "Restarted by reset @ ", buffer);
      break;
    case PORF:
      SENDLOG ('I', "Restarted by power off @ ", buffer);
      break;      
  }

  // Capture relative time
  prevTime = millis();
}


void acceptInput() {
	SAVE_CONTEXT("aInp")
  
	const byte MAX_MSGS = 10;						  // Max msgs to process before exiting while loop; avoids continual loop
	byte numMsgs = 0;
	unsigned int bufPosn, dataLen;							
	char timeText[30];
	char zone;
	#define BUF_ADD bufPosn += snprintf(buffer + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn,      // Take care; buffer used for input & output

	while (dataLen = ard.get(buffer, UDP_TX_PACKET_MAX_SIZE)) {        // If data available - multiple messages potentially
	
		if (sendToSyslog) {
			if (dataLen <= UDP_TX_PACKET_MAX_SIZE) buffer[dataLen] = 0x00;
			SENDLOGM('N', buffer);
		}
		
		if (numMsgs++ > MAX_MSGS) {
			SENDLOGM('W', "Max msgs exceeded");
			return;
		}

		bufPosn = 0;										// Position to start of buffer for output & overwriting redundant input

	}

	RESTORE_CONTEXT
}
