/* Zone control for Gt Hall

Version history
---------------

Sep 20 - v1 - preliminary version, based on basic zone controller for comms, plus analogue read of x(O) sensors
Nov 20 - v2 - upgraded to field queries and send messages to boiler

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



// ****** Identity of this controller ******

#define meIP GtHallIP              // 192.168.7.181
#define meMac GtHallMac
char meName[]									= "GtHall";
const static char meInit			= 'G';

// ***** External comms *****

char logMsg[UDP_TX_PACKET_MAX_SIZE];        // Max allowed by libraries
byte bufPosn;
boolean sendToSyslog = true;

// ******  Window sensors  *****

const int numWindows = 8;
const int window[numWindows] = { A2, A3, A4, A5, A6, A7, A8, A9 };   // See below for logic of A2 start

/*
From https://forum.arduino.cc/index.php?topic=20114.0

What pins does the ethernet shield really use ? According to the schematic...

D2 - Ethernet interrupt(optional with solder bridge "INT")
D4 - SD SPI CS
D10 - Ethernet SPI CS
D11 - Not connected(but should be SPI MOSI)
D12 - Not connected(but should be SPI MISO)
D13 - SPI SCK
A0 - SD Write Protect
A1 - SD Detect

See also: http://shieldlist.org/arduino/ethernet-v5

Although not used by most SD card libraries, A0 is connected 
to the SD slot's write protect (WP) pin and A1 is connected 
to the card detect switch. Both are pulled high by 10k resistors 
on the Ethernet shield. These pins therefore cannot be used for 
analog input with the shield unless they are bent back or clipped off 
before inserting the shield into the arduino. Additionally, if a user's 
circuit uses these pins for digital IO, the pullups may cause problems for existing circuitry.
*/


// *************  Temperature zones ***********

const static byte ZONE_GREAT_HALL = 0;
const char sensorTag[] = { 'G' };
const static byte NUM_SENSORS = 1;
const static byte GREAT_HALL_MANIFOLD = 1;
const static byte NUM_ZONES = NUM_SENSORS + 1;	

// Time patterns define whether zone is ON or OFF

const byte NUM_TIME_PATTERNS = 5;          // Number of different time patterns permitted
const byte ALLDAY = 0;								// 6am - 11pm
const byte BACKGROUND = 1;							// Cycling through the day
const byte AFTERNOON = 2;							// Post noon
const byte TURNOFF = 3;	 							// Turn off
const char timePatternTag[] = { 'A', 'B', 'N', 'X' };

byte timePattern[NUM_ZONES] = { BACKGROUND, BACKGROUND };		// Set the time patterns for each zone - Gg

const byte MAX_TIME_PERIODS = 5;           // Number of ON periods allowed per day per time pattern

// ******  Time ******
// Current time is updated every 30 secs ...

unsigned int timeNow = 0;

// ... and compared against time patterns ...
unsigned int timePeriodStart[NUM_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];        // Start of time period
unsigned int timePeriodEnd[NUM_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];          // End of time period
byte numTimePeriods[NUM_ZONES][NUM_TIME_PATTERNS];

// ... and set flag for zone accordingly to determine if zone on or off

boolean onPeriod[NUM_ZONES];                            // Set by time vs programme.  If ON then active; if OFF then go to stanbdy

// ******   Internal (relative) timing  *******

unsigned long prevTime;
unsigned int heartBeatSecs									= 0;                
const unsigned int HEARTBEAT_FREQ_MS				= 1000;		// 1 heartbeats per second
const unsigned int CHECK_INPUT_FREQ					= 1;			// Check if any messages from console or other arduinos
const unsigned int CHECK_WINDOWS_FREQ       = 5;			// Check window sensors every 5 secs
const unsigned int CHECK_MANIFOLD_FREQ = 5;						// Check if manifolds need heat and alert boiler
const unsigned int CHECK_TIME_FREQ = 30;							// Work out current time every 30 secs and act on it
const unsigned int REPORT_STATUS_FREQ = 10;						// Status log to syslog

// **** Megacore code to preserve internal registers to determine restart reason ****
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

	// Generic setup actions
	setupHA();

	// Setup actions specific to this controller
	// Window sensor input
	for (int i = 0; i < numWindows; i++) {
			pinMode(window[i], INPUT);
	}

  // Start watchdog with 8 sec timeout
  wdt_enable(WDTO_8S);
}


void loop() {

  // Reset watchdog - if fault then this won't happen and watchdog will fire, restarting the sketch
  wdt_reset();

  // Increment heartbeat every second 
  if ((millis() - prevTime) >= HEARTBEAT_FREQ_MS) {   

    prevTime = millis();                  
    heartBeatSecs++;       

		// Prepare buffer for reporting temperatures
		bufPosn = 0;
		memset(logMsg, '\0', UDP_TX_PACKET_MAX_SIZE);

		// See if any commands from console or other arduinos
		if (heartBeatSecs % CHECK_INPUT_FREQ == 0) checkForInput();

		// Calculate current time
		if (heartBeatSecs % CHECK_TIME_FREQ == 0) checkTime();

    // Check window sensors
    if (heartBeatSecs % CHECK_WINDOWS_FREQ == 0) checkWindows();

		// See if manifolds want more heat - tell boiler
		if (heartBeatSecs % CHECK_MANIFOLD_FREQ == 0) checkManifolds();

  }
  
  delay(50);
}

void setupHA() {

  int myMCUSR;    // Startup reason

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

  // Initialise wakeup to support background daemons (eg for time)
  wakeup.init();

  // Get the time from NTP server
  initialiseTime(UdpNTPPort);
  timeToText(now(), logMsg, UDP_TX_PACKET_MAX_SIZE);           // Get current time

  // Log reason for startup to SYSLOG - order of tests is significant; last succesful test is what's reported
  
  if (resetFlag & (1 << WDRF)) myMCUSR = WDRF;
  if (resetFlag & (1 << EXTRF)) myMCUSR = EXTRF;
  if (resetFlag & (1 << PORF)) myMCUSR = PORF;
  
  switch (myMCUSR) {
    case WDRF: 
      SENDLOG ('I', "Restarted by watchdog @ ", logMsg);
      break;
    case EXTRF:
      SENDLOG ('I', "Restarted by reset @ ", logMsg);
      break;
    case PORF:
      SENDLOG ('I', "Restarted by power off @ ", logMsg);
      break;      
  }

	// Set up programmed periods - Day 0 == Everyday, Day 10 == Sat/Sun
	for (int zone = 0; zone < NUM_ZONES; zone++) {
			onPeriod[zone] = false;

			// Pattern ALLDAY 
			timePeriodStart[zone][ALLDAY][0] = dhmMake(0, 6, 0);    // 06:00
			timePeriodEnd[zone][ALLDAY][0] = dhmMake(0, 23, 00);    // 23:00
			numTimePeriods[zone][ALLDAY] = 1;

			// Pattern BACKGROUND - 2 hours on/2 hours off
			timePeriodStart[zone][BACKGROUND][0] = dhmMake(0, 6, 0);
			timePeriodEnd[zone][BACKGROUND][0] = dhmMake(0, 8, 0);
			timePeriodStart[zone][BACKGROUND][1] = dhmMake(0, 10, 0);
			timePeriodEnd[zone][BACKGROUND][1] = dhmMake(0, 12, 0);
			timePeriodStart[zone][BACKGROUND][2] = dhmMake(0, 14, 0);
			timePeriodEnd[zone][BACKGROUND][2] = dhmMake(0, 16, 0);
			timePeriodStart[zone][BACKGROUND][3] = dhmMake(0, 18, 0);
			timePeriodEnd[zone][BACKGROUND][3] = dhmMake(0, 20, 0);
			timePeriodStart[zone][BACKGROUND][4] = dhmMake(0, 22, 0);
			timePeriodEnd[zone][BACKGROUND][4] = dhmMake(0, 23, 0);
			numTimePeriods[zone][BACKGROUND] = 5;

			// Pattern AFTERNOON 
			timePeriodStart[zone][AFTERNOON][0] = dhmMake(0, 12, 0);
			timePeriodEnd[zone][AFTERNOON][0] = dhmMake(0, 23, 00);
			numTimePeriods[zone][AFTERNOON] = 1;

			// Pattern TURNOFF
			numTimePeriods[zone][TURNOFF] = 0;
	}
  
  // Start the relative clock
  prevTime = millis();

}

void checkTime() {		// Get current time, test if any zones are on and turn on zone flag as appropriate
		SAVE_CONTEXT("CheckT")

		// Get current time & pack into dhm format
		timeNow = dhmMake(weekday(), hour(), minute());

		// Send to syslog
		if (sendToSyslog) {
				char timeText[30];
				int bufEnd = snprintf(timeText, 30, "T %04X ", timeNow);
				dhmToText(timeNow, timeText + bufEnd);
				SENDLOGM('N', timeText);
		}

		// Test if zones are on or off
		for (int i = 0; i < NUM_ZONES; i++) {
				onPeriod[i] = isOn(i);
		}

		RESTORE_CONTEXT
}

boolean isOn(byte zone) {  // Check if zone is scheduled to be on or not

		boolean newStatus = false;
		byte programme = timePattern[zone];

		// Loop through on/off periods for zone and programme to find match
		for (int i = 0; i < numTimePeriods[zone][programme]; i++) {
				if (newStatus = dhmBetween(timeNow, timePeriodStart[zone][programme][i], timePeriodEnd[zone][programme][i])) break;    // Exit on match
		}

		return newStatus;
}

void checkWindows() {
	SAVE_CONTEXT("aInp")
  
  const float refVoltage = 5;
  const float maxTicks = 1023;
  const float mvPerTick = refVoltage / maxTicks;

  for (int i = 0; i < numWindows; i++) {
      float ticks = analogRead(window[i]);
      float volts = ticks * mvPerTick;

      char msg[] = { 'W', i + 48, '=', 0 };
			if (sendToSyslog) SENDLOG('I', msg, volts);
  }

	RESTORE_CONTEXT
}

void checkForInput() {
		SAVE_CONTEXT("cFInp")

		const byte MAX_MSGS = 5;						  // Max msgs to process before exiting while loop
		byte numMsgs = 0;
		unsigned int dataLen;

		byte bufPosn;
		char buffer[UDP_TX_PACKET_MAX_SIZE];
		#define BUF_ADD bufPosn += snprintf(buffer + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn,      // Take care; buffer used for input & output

		byte sensorNum, limit;
		char displayMode;
		byte zone;

		while ((dataLen = ard.get(buffer, UDP_TX_PACKET_MAX_SIZE)) && (numMsgs++ < MAX_MSGS)) {        // If data available - multiple messages potentially

				bufPosn = 0;										// Position to start of buffer for output & overwriting redundant input

				if (sendToSyslog) {
						if (dataLen < UDP_TX_PACKET_MAX_SIZE) buffer[dataLen] = 0x00;
						SENDLOGM('N', buffer);
				}

				switch (buffer[0]) {

				case 'P':        // Set timePattern for zone
						switch (buffer[1]) {
						case 'G': zone = GREAT_HALL_MANIFOLD; break;
						default: SENDLOGM('W', "Invalid zone");
						}

						switch (buffer[2]) {
						case 'A': timePattern[zone] = ALLDAY; break;
						case 'B': timePattern[zone] = BACKGROUND; break;
						case 'N':	timePattern[zone] = AFTERNOON; break;
						case 'X':	timePattern[zone] = TURNOFF; break;
						default:     SENDLOGM('W', "Invalid timePattern");
						}

						// Reply to requestor
						ard.reply(REPLY_PORT, buffer, bufPosn);

						// Copy to syslog

						if (sendToSyslog) SENDLOGM('N', buffer);

						break;

				case 'S':        // Status request - just a ping at present to test syslogstatus

						BUF_ADD "{\"DA\":\"%c%c\",", meInit, (sendToSyslog) ? 'Y' : 'N');
						BUF_ADD "}\0");
						
						// Reply to requestor
						ard.reply(REPLY_PORT, buffer, bufPosn);

						// Copy to syslog

						if (sendToSyslog) SENDLOGM('N', buffer);

						break;

				case 'X':
						sendToSyslog = (buffer[1] == '1');
						break;
				}
		}
		RESTORE_CONTEXT
}

void checkManifolds() {      // See if manifolds want more heat - tell boiler
		char message[] = "ZG ";

		// Great Hall just determine by timePeriod pending sensor input
		message[2] = (onPeriod[GREAT_HALL_MANIFOLD]) ? '1' : '0';
		ard.put(basementIP, UdpArdPort, message, 3);
}
