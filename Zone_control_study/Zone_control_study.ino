/* Zone control for Study (lower) zone

- Pin 2 - uses output from relay to determine if Study manifold wants more heat 

*/

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ICMPPing.h>

#include "HA_globals.h"
#include "HA_syslog.h"         // For syslog comms
#include "HA_queue.h"          // Needed to support syslog
#include "HA_comms.h"          // For general UDP comms

#include "HA_time.h"
#include "Time.h"
#include "TimeLib.h"
#include "Wakeup.h"
#include "TimerOne.h"


// ****** Pin assignments ************

const static byte MANIFOLD_DEMAND          = 2;            // High if Study manifold wants heat
const static byte WATCHDOG                 = 22;            // Ensure make open collector every 30 secs min

// ****** Manifold sensor ******

byte studyManifold                 = OFF;

#define meIP studyIP                      
#define meMac studyMac
char meName[] = "Study";
const static char meInit = 'S';
boolean sendToSyslog = false;

boolean somethingWrong = false;
char buffer[UDP_TX_PACKET_MAX_SIZE]; 

ICMPPing ping;

// ****** Master on/off control ******

boolean onPeriod = false;                            // Set by message.  If ON then active; if OFF then go to stanbdy. To do: use this to turn manifold on/off - needs hardware
			
// ******   Timing   *******
//
unsigned long prevTime;
unsigned int heartBeatSecs								 = 0;   
const unsigned int HEARTBEAT_FREQ_MS       = 1000;     // Heartbeat every second
const unsigned int CHECK_INPUT_FREQ        = 1;        // Check if any messages from console or other arduinos
const unsigned int CHECK_MANIFOLD_FREQ     = 5;        // Check if manifold needs heat and alert boiler
const unsigned int WATCHDOG_FREQ           = 20;       // Period determined by external circuitry - amend with care
const unsigned int WATCHDOG_DISCHARGE_MS   = 200;      // Time to hold line open circuit to discharge capacitor
const unsigned int CHECK_ALIVE_FREQ        = 60;			 // Send a ping to see if alive; if not then allow watchdog to restart
const unsigned int SEND_TIME_FREQ          = 30;


void setup() {

  // Standard setup actions 
  setupHA();
  
  // Initialse manifold demand sensor (pins default to INPUT, but just in case)
  pinMode(MANIFOLD_DEMAND, INPUT);

  // Reset watchdog before loop
  resetWatchDog();
    
  // Capture relative time
  prevTime = millis();
}


void loop() {
  SAVE_CONTEXT("Loop")

  // Run any background daemons - needed to support HA_time.cpp
  wakeup.runAnyPending();
  
  // Increment heartbeat every second 
  if ((millis() - prevTime) >= HEARTBEAT_FREQ_MS) {   

    prevTime = millis();                  
    heartBeatSecs++;                                          

    // Test comms is working
	if (heartBeatSecs % CHECK_ALIVE_FREQ == 0) checkAlive();
		
	// See if any commands from console or other arduinos
	if (heartBeatSecs % CHECK_INPUT_FREQ == 0) acceptInput();
      
    // Reset watchdog, unless comms failure when need a reset
    if ((heartBeatSecs % WATCHDOG_FREQ == 0) && !somethingWrong) resetWatchDog();

    // See if manifold wants more heat - tell boiler
    if (heartBeatSecs % CHECK_MANIFOLD_FREQ == 0) checkManifold();
    
    // Send out the time to all other controllers
    if (heartBeatSecs % SEND_TIME_FREQ == 0) sendTime();
  }

  RESTORE_CONTEXT
}

void setupHA() {
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
  
  // Get the time from NTP server and broadcast it
  initialiseTime(UdpNTPPort);
  timeToText(now(), buffer, UDP_TX_PACKET_MAX_SIZE);           // Get current time
  SENDLOG('I', "Startup finished @ ", buffer)                    // Send to Syslog
}

void checkAlive() {
	// See if can communicate - if not, then time to restart
	somethingWrong = !ping(4, (byte*)syslogServerIP, NULL);

	if (somethingWrong) SENDLOGM('W', "Unable to ping");					// Although if syslog down then this is nugatory!
}

void resetWatchDog() {             // Courtesy http://www.playwitharduino.com/?p=291&lang=enChromeHTML\Shell\Open\Command
  pinMode(WATCHDOG, OUTPUT);       // Open collector - resets watchdog
  delay(WATCHDOG_DISCHARGE_MS);    // Wait for capacitor discharge
  pinMode(WATCHDOG, INPUT);        // Hi-Z

	if (sendToSyslog) SENDLOGM('N', "Watchdog reset");
}

void checkManifold() { 
  // Study manifold status based on output signal from manifold
  char message[] = "ZS "; 
  studyManifold = digitalRead(MANIFOLD_DEMAND);
  message[2] = (studyManifold && onPeriod) ? '1' : '0';      // On or off
	
	// Send the message
	ard.put(basementIP, UdpArdPort, message, 3);
  
	// Log it if necessary
	if (sendToSyslog) SENDLOGM('N', (char*)message); 
}    

void sendTime() {
  SAVE_CONTEXT("SendT")
  
  char message[3] = "T";
	unsigned int timeNow;
  
	// Pack time into dhm format
  timeNow = dhmMake(weekday(), hour(), minute());
  message[1] = timeNow >> 8;
  message[2] = timeNow & 0x00ff;

	// Send it to other controllers
  ard.put(basementIP, UdpArdPort, message, 3);
  ard.put(diningIP, UdpArdPort, message, 3);
  
	if (sendToSyslog) {
		int bufEnd = snprintf(buffer, 30, "T %04X ", timeNow);
	  dhmToText(timeNow, buffer + bufEnd);
		SENDLOGM('N', buffer);
	}
  
  RESTORE_CONTEXT
}

/* ***** acceptInput from network *****

Valid inputs:

On		ON or OFF master controller
Xn    Switch ON or OFF syslog reporting

*/

void acceptInput() {
	SAVE_CONTEXT("aInp")
		
	char buffer[UDP_TX_PACKET_MAX_SIZE];
	unsigned int bufPosn, dataLen;							// Positions to start of buffer, overwriting input data	
	#define BUF_ADD bufPosn += snprintf(buffer + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn,    
	const byte MAX_MSGS = 5;						  // Max msgs to process before exiting while loop
	byte numMsgs = 0;
	
	while ((dataLen = ard.get(buffer, UDP_TX_PACKET_MAX_SIZE)) && (numMsgs++ < MAX_MSGS)) {

		bufPosn = 0;										// Position to start of buffer for output & overwriting redundant input

		if (sendToSyslog) {
				if (dataLen <= UDP_TX_PACKET_MAX_SIZE) buffer[dataLen] = 0x00;
				SENDLOGM('N', buffer);
		}

		switch (buffer[0]) {
			case 'O':
				onPeriod = (buffer[1] == '1');
				break;

			case 'X':
				sendToSyslog = (buffer[1] == '1');
				break;
		}
	}
	RESTORE_CONTEXT
}
