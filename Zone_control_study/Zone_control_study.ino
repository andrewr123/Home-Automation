/*
ZONE CONTROLLER FOR STUDY/GUEST WING
------------------------------------

Version history
---------------
 
Jun 13 - preliminary version acting as master time server and using external watchdog
         - uses output from relay to determine if Study manifold wants more heat
Nov 20 - upgraded to use latest pattern - time local to this controller only, internal watchdog using Megacore bootloader

*/

#define UDP_TX_PACKET_MAX_SIZE 128			// Per override in OneDrive\Documents\Arduino\Libraries\Ethernet\EthernetUDP.h

#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "HA_globals.h"
#include "HA_syslog.h"         // For syslog comms
#include "HA_queue.h"          // Needed to support syslog
#include "HA_comms.h"          // For general UDP comms

#include "HA_time.h"
#include "Time.h"
#include "TimeLib.h"
#include "Wakeup.h"
#include "TimerOne.h"



// SPECIFIC ELEMENTS - FOR THIS CONTROLLER

// ****** Identity of this controller ******

#define meIP studyIP                    // 192.168.7.180               
#define meMac studyMac
char meName[] = "Study";
const static char meInit = 'S';

// ****** Manifold demand sensor ************

const static byte STUDY_MANIFOLD_DEMAND         = 2;            // High if Study manifold wants heat

// *************  Temperature zones ***********

const static byte ZONE_STUDY = 0;
const char sensorTag[] = { 'S' };
const static byte NUM_SENSORS = 1;
const static byte NUM_ZONES = NUM_SENSORS;

// GENERIC ELEMENTS - COMMON TO ALL CONTROLLERS
// --------------------------------------------

// ***** External comms *****

char syslogLevel = 'W';				// Available flags - XACEWNID

// ******  Time and time patterns ******

// Current time is updated every 30 secs ...

unsigned int timeNow = 0;

// ... and compared against time patterns ...
const byte NUM_TIME_PATTERNS = 4;          // Number of different time patterns permitted
const byte MAX_TIME_PERIODS = 5;           // Number of ON periods allowed per day per time pattern
unsigned int timePeriodStart[NUM_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];        // Start of time period
unsigned int timePeriodEnd[NUM_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];          // End of time period
byte numTimePeriods[NUM_ZONES][NUM_TIME_PATTERNS];

// Time patterns define whether zone is ON or OFF

const byte ALLDAY = 0;								// 6am - 11pm
const byte BACKGROUND = 1;						// Start and end of day
const byte AFTERNOON = 2;							// Post noon
const byte TURNOFF = 3;	 							// Turn off
const char timePatternTag[] = { 'A', 'B', 'N', 'X' };

// ... and comparison with timeNow sets flag to determine if zone on or off

boolean onPeriod[NUM_ZONES];                            // Set by time vs programme.  If ON then active; if OFF then go to stanbdy

// Set default time patterns
byte timePattern[NUM_ZONES] = { AFTERNOON };		// Set the time patterns for each zone
			
// ******   Timing   *******
//
unsigned long prevTime;
unsigned int heartBeatSecs								 = 0;   
const unsigned int HEARTBEAT_FREQ_MS       = 1000;     // Heartbeat every second
const unsigned int CHECK_INPUT_FREQ        = 1;        // Check if any messages from console or other arduinos
const unsigned int CHECK_MANIFOLD_FREQ     = 5;        // Check if manifold needs heat and alert boiler
const unsigned int CHECK_TIME_FREQ				 = 30;							// Work out current time every 30 secs and act on it


// PRE-BOOT CODE
// -------------

// **** Megacore code to preserve internal registers to determine restart reason ****
/*
   Code added from https://github.com/Optiboot/optiboot/blob/master/optiboot/examples/test_reset/test_reset.ino

   First, we need a variable to hold the reset cause that can be written before early sketch initialization
   (which might change r2), and won't be reset by the various initialization code.
   avr-gcc provides for this via the ".noinit" section.
*/
uint8_t resetFlag __attribute__((section(".noinit")));

/*
   Next, we need to put some code to save reset cause from the bootload (in r2) to the variable. Again, avr-gcc
   provides special code sections for this.  If compiled with link time optimization (-flto), as done by the
   Arduno IDE version 1.6 and higher, we need the "used" attribute to prevent this from being omitted.
*/
void resetFlagsInit(void) __attribute__((naked))
__attribute__((used))
__attribute__((section(".init0")));
void resetFlagsInit(void)
{
    /*
       save the reset flags passed from the bootloader.  This is a "simple" matter of storing (STS) r2 in the
       special variable that we have created.  We use assembler to access the right variable.
    */
    __asm__ __volatile__("sts %0, r2\n" : "=m" (resetFlag) : );
}


void setup() {

    // Reset watchdog
    MCUSR = 0;
    wdt_disable();

    setupComms();
    logStartupReason();
    setupTimePeriods();
    setupManifold();

    // Start watchdog with 8 sec timeout
    wdt_enable(WDTO_8S);

    // Start the relative clock
    prevTime = millis();
}


void loop() {

		const byte BUFLEN = 64;
		char buffer[BUFLEN];

		// Reset watchdog - if fault then this won't happen and watchdog will fire, restarting the sketch
		wdt_reset();

		// Run any background daemons - needed to support HA_time.cpp
		wakeup.runAnyPending();
  
		// Increment heartbeat every second 
		if ((millis() - prevTime) >= HEARTBEAT_FREQ_MS) {   

				prevTime = millis();                  
				heartBeatSecs++;                                          
		
				// See if any commands from console or other arduinos
				if (heartBeatSecs % CHECK_INPUT_FREQ == 0) processIncomingUDP();

				// Calculate current time
				if (heartBeatSecs % CHECK_TIME_FREQ == 0) checkTime();

				// See if manifold wants more heat - tell boiler
				if (heartBeatSecs % CHECK_MANIFOLD_FREQ == 0) checkManifold();

				// Report any errors
				listErrors(buffer, BUFLEN);
				if (strstr(buffer, "OK") == NULL) SENDLOGM('W', buffer);
		}

		delay(50);
}

void setupComms() {

		/*
				Disable the SD CS to avoid interference with UDP transmission - http://arduino.cc/forum/index.php?action=printpage;topic=96651.0
				And ensure SPI CS pin is set to output to avoid problems with SD card	*

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
		pinMode(SD_CS_PIN, OUTPUT);
		digitalWrite(SD_CS_PIN, HIGH);
		pinMode(SPI_CS_PIN, OUTPUT);

		// start the Ethernet connection
		Ethernet.begin(meMac, meIP);

		// Initialise UDP comms
		syslog.init(UdpLogPort, syslogServerIP, syslogServerPort, syslogLevel);
		ard.init(UdpArdPort);

		// Initialise wakeup to support background daemons (eg for time)
		wakeup.init();

		// Get the time from NTP server
		initialiseTime(UdpNTPPort);
}

void logStartupReason() {

		// Log reason for startup to SYSLOG by testing resetFlag made available on startup - see Megacore refs above

		int myMCUSR;    // Startup reason
		char logMsg[UDP_TX_PACKET_MAX_SIZE];
		timeToText(now(), logMsg, UDP_TX_PACKET_MAX_SIZE);           // Get current time

		// Order of tests is significant; last succesful test is what's reported
		if (resetFlag & (1 << WDRF)) myMCUSR = WDRF;				// Watchdog reset
		if (resetFlag & (1 << EXTRF)) myMCUSR = EXTRF;			// Manual reset
		if (resetFlag & (1 << PORF)) myMCUSR = PORF;				// Power-on reset

		switch (myMCUSR) {
		case WDRF:
				SENDLOG('W', "Restarted by watchdog @ ", logMsg);
				break;
		case EXTRF:
				SENDLOG('W', "Restarted by reset @ ", logMsg);
				break;
		case PORF:
				SENDLOG('W', "Restarted by power off @ ", logMsg);
				break;
		}
}

void setupTimePeriods() {

		// Set up programmed periods - Day 0 == Everyday, Day 10 == Sat/Sun
		for (int zone = 0; zone < NUM_ZONES; zone++) {
				onPeriod[zone] = false;

				// Pattern ALLDAY 
				timePeriodStart[zone][ALLDAY][0] = dhmMake(0, 6, 0);    // 06:00
				timePeriodEnd[zone][ALLDAY][0] = dhmMake(0, 23, 00);    // 23:00
				numTimePeriods[zone][ALLDAY] = 1;

				// Pattern BACKGROUND
				timePeriodStart[zone][BACKGROUND][0] = dhmMake(0, 6, 0);
				timePeriodEnd[zone][BACKGROUND][0] = dhmMake(0, 9, 0);
				timePeriodStart[zone][BACKGROUND][1] = dhmMake(0, 18, 0);
				timePeriodEnd[zone][BACKGROUND][1] = dhmMake(0, 23, 00);
				timePeriodStart[zone][BACKGROUND][2] = dhmMake(10, 6, 0);
				timePeriodEnd[zone][BACKGROUND][2] = dhmMake(10, 23, 00);
				numTimePeriods[zone][BACKGROUND] = 3;

				// Pattern AFTERNOON 
				timePeriodStart[zone][AFTERNOON][0] = dhmMake(0, 12, 0);
				timePeriodEnd[zone][AFTERNOON][0] = dhmMake(0, 23, 00);
				numTimePeriods[zone][AFTERNOON] = 1;

				// Pattern TURNOFF
				numTimePeriods[zone][TURNOFF] = 0;
		}
}

void setupManifold() {

		// Initialse manifold demand sensor (pins default to INPUT, but just in case)
		pinMode(STUDY_MANIFOLD_DEMAND, INPUT);
}

void processIncomingUDP() {

		/****** Process incoming from network *****

		Valid inputs:

		PSa			Set time pattern
		Xn			Set syslog reporting level

		*/

		SAVE_CONTEXT("aInp")

		const byte MAX_MSGS = 5;						  // Max msgs to process before exiting while loop
		byte numMsgs = 0;
		unsigned int dataLen;

		byte bufPosn;
		char recvBuffer[UDP_TX_PACKET_MAX_SIZE];
		char sendBuffer[UDP_TX_PACKET_MAX_SIZE];

		#define BUF_ADD bufPosn += snprintf(sendBuffer + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn,      // Take care; buffer used for input & output

		byte zone;

		while (dataLen = ard.get(recvBuffer, UDP_TX_PACKET_MAX_SIZE)) {        // If data available - multiple messages potentially

				if (dataLen <= UDP_TX_PACKET_MAX_SIZE) recvBuffer[dataLen] = 0x00;
				SENDLOGM('D', recvBuffer);

				if (numMsgs++ > MAX_MSGS) {
						SENDLOGM('W', "Max msgs exceeded");
						RESTORE_CONTEXT
						return;
				}

				bufPosn = 0;										// Position to start of buffer for output & overwriting redundant input

				switch (recvBuffer[0]) {

				case 'P':        // Set timePattern for zone
						switch (recvBuffer[1]) {
								case 'S': zone = ZONE_STUDY; break;
								default: SENDLOGM('W', "Invalid zone");
						}

						switch (recvBuffer[2]) {
								case 'A': timePattern[zone] = ALLDAY; break;
								case 'B': timePattern[zone] = BACKGROUND; break;
								case 'N':	timePattern[zone] = AFTERNOON; break;
								case 'X':	timePattern[zone] = TURNOFF; break;
								default: SENDLOGM('W', "Invalid timePattern");
						}

						// Reply to requestor
						ard.reply(REPLY_PORT, sendBuffer, bufPosn);

						// Copy to syslog

						SENDLOGM('D', sendBuffer);

						break;

				case 'S': {     // Status request - at present just timePattern and syslog level
						switch (recvBuffer[1]) {
								case 'P': {					// Current time pattern
										char programme = timePatternTag[timePattern[ZONE_STUDY]];
										BUF_ADD "{\"DA\":\"%cP\", \"%c\":\"%c\"}\0", meInit, meInit, (onPeriod[ZONE_STUDY]) ? programme : programme + 32);
										break;
								}

								case 'X':				// Syslog notification level
										BUF_ADD "{\"DA\":\"%cX\", \"X\":\"%c\"}\0", meInit, syslogLevel);
										break;

								default:
										BUF_ADD "Invalid status request: %c\0", recvBuffer[1]);
										SENDLOGM('W', sendBuffer);
						}
						
						// Reply to requestor
						ard.reply(REPLY_PORT, sendBuffer, bufPosn);

						// Copy to syslog

						SENDLOGM('D', sendBuffer);

						break;
				}

				case 'X':
						syslogLevel = recvBuffer[1];
						syslog.adjustLevel(syslogLevel);
						break;
				}
		}
		
		RESTORE_CONTEXT
}



void checkTime() {		// Get current time, test if any zones are on and turn on zone flag as appropriate

		SAVE_CONTEXT("CheckT")

		char timeText[30];

		// Get current time & pack into dhm format
		timeNow = dhmMake(weekday(), hour(), minute());

		// Send to syslog
		int bufEnd = snprintf(timeText, 30, "T %04X ", timeNow);
		dhmToText(timeNow, timeText + bufEnd);
		SENDLOGM('D', timeText);

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


void checkManifold() {
		// Study manifold status based on output signal from manifold and time pattern
		char message[] = "ZS ";
		message[2] = (digitalRead(STUDY_MANIFOLD_DEMAND) && onPeriod[ZONE_STUDY]) ? '1' : '0';      // On or off

		// Send the message
		ard.put(basementIP, UdpArdPort, message, 3);

		// Log it if necessary
		SENDLOGM('I', message);
}
