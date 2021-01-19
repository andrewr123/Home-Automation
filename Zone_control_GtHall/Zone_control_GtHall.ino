/* 
ZONE CONTROLLER FOR GREAT HALL
------------------------------

Version history
---------------

Sep 20 - v1 - preliminary version, based on basic zone controller for comms, plus analogue read of x(O) sensors
Nov 20 - v2 - upgraded to field queries and send messages to boiler
Dec 20 - v3 - upgraded to handle temperature sensors using HA_temperature library and to use watchdog timer

Pin usage:
- A2-A9			- Honeywell Hall Effect window sensors: voltage dependent on whether window open or not
- D3				- 5v power to temperature sensors - reset if error noticed
- D4-D7			- Dallas temperature sensors - reset if error noticed

UDP commands:
- Pzt		- Set timePattern for zone; z ==
					- G - Great Hall
					t ==
					- A - Allday
					- B - Background
					- N - Afternoon
					- X - Off
- Fzn   - force heating ON (1) or OFF (!1)
- Ss		- Status request; s ==
					- P - timePattern
					- T - temperatures
					- W - window sensors
					- X - current syslog level
- Xn		- Syslog level
- Zznn	- set target temperature for zone z to nnC
*/

#define UDP_TX_PACKET_MAX_SIZE 128			// Per override in OneDrive\Documents\Arduino\Libraries\Ethernet\EthernetUDP.h
#define GREAT_HALL_CONTROLLER						// Sets parameters in HA_globals.h
//#define DEBUG


#include <avr/wdt.h>
#include <SPI.h>
#include "Ethernet.h"
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
#include "HA_temperature.h"


// SPECIFIC ELEMENTS - FOR THIS CONTROLLER
// ---------------------------------------

// ****** Identity of this controller ******

#define meIP GtHallIP              // 192.168.7.181
#define meMac GtHallMac
char meName[] = "GtHall";
const static char meInit = 'G';

// ******  Window sensors  *****

const int NUM_WINDOW_SENSORS = 8;
const int wSensor[NUM_WINDOW_SENSORS] = { A2, A3, A4, A5, A6, A7, A8, A9 };   // See below for logic of A2 start
const char wSensorTag[NUM_WINDOW_SENSORS] = { 'A', 'a', 'B', 'b', 'C', 'c', 'D', 'd' };
float wSensorVolts[NUM_WINDOW_SENSORS];

const float REF_VOLTAGE = 5;
const float MAX_TICKS = 1023;
const float MV_PER_TICK = REF_VOLTAGE / MAX_TICKS;

// *************  Temperature sensors ***********

const char tSensorTag[] = { 'H' }; // { 'G', 'H' };
const static byte TEMP_SENSOR_PIN[NUM_TEMP_SENSORS] = { 7 };  // { 5, 7 };			// Dallas sensor pins
const static byte TEMP_SENSOR_POWER_PIN = 3;												// 5v power to Dallas temp sensors

HA_temperature tSensor[NUM_TEMP_SENSORS];														// Dallas sensor objects
const static byte TARGET_TEMP_PRECISION = 10;												// Equates to 0.25C - good enough

// ****** Heating zones ******
const static byte GREAT_HALL_MANIFOLD = 0;
const static byte NUM_ZONES = 1;


// GENERIC ELEMENTS - COMMON TO ALL CONTROLLERS
// --------------------------------------------

// ***** External comms *****

char syslogLevel = 'D';
const char ALLOWABLE_SYSLOG_LEVELS[] = "XACEWNID";

// ******  Time and time patterns ******

// Current time is updated every 30 secs ...

unsigned int timeNow = 0;

// ... and compared against time patterns ...
const byte NUM_TIME_PATTERNS = 4;          // Number of different time patterns permitted
const byte MAX_TIME_PERIODS = 8;           // Number of ON periods allowed per day per time pattern
unsigned int timePeriodStart[NUM_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];        // Start of time period
unsigned int timePeriodEnd[NUM_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];          // End of time period
byte numTimePeriods[NUM_ZONES][NUM_TIME_PATTERNS];

// Time patterns define whether zone is ON or OFF

const byte ALLDAY = 0;								// 6am - 11pm
const byte BACKGROUND = 1;						// Cycling through the day
const byte AFTERNOON = 2;							// Post noon
const byte TURNOFF = 3;	 							// Turn off
const char timePatternTag[] = { 'A', 'B', 'N', 'X' };

// ... and comparison with timeNow sets flag to determine if zone on or off

boolean onPeriod[NUM_ZONES];                            // Set by time vs programme.  If ON then active; if OFF then go to stanbdy
boolean forceOn[NUM_ZONES];                             // True if OFF period overriden (by message).  Reset with next ON period

// Set default time patterns
byte timePattern[NUM_ZONES] = { BACKGROUND };		// Set the time patterns for each manifold - G

// ******   Internal (relative) timing  *******

unsigned long prevTime;
unsigned int heartBeatSecs									= 0;                
const unsigned int HEARTBEAT_FREQ_MS				= 1000;		// 1 heartbeats per second
const unsigned int CHECK_INPUT_FREQ					= 1;			// Check if any messages from console or other arduinos
const unsigned int CHECK_WINDOWS_FREQ       = 5;			// Check WINDOW sensors every 5 secs
const unsigned int CHECK_TEMP_FREQ					= 5;
const unsigned int CHECK_TEMP_SENSORS_FREQ	= 60;						// Check temperature sensors every minute - restart if necessary
const unsigned int CHECK_MANIFOLD_FREQ			= 5;						// Check if manifolds need heat and alert boiler
const unsigned int CHECK_TIME_FREQ					= 30;							// Work out current time every 30 secs and act on it
const byte CHECK_FREQ[NUM_TEMP_SENSORS]			= { CHECK_TEMP_FREQ };




// PRE-BOOT CODE
// -------------

// **** Megacore code to preserve internal registers to determine restart reason ****
/*
   Code added from https://github.com/Optiboot/optiboot/blob/master/optiboot/examples/test_reset/test_reset.ino

   First, we need a variable to hold the reset cause that can be written before early sketch initialization 
	 (which might change r2), and won't be reset by the various initialization code.
   avr-gcc provides for this via the ".noinit" section.
*/
uint8_t resetFlag __attribute__ ((section(".noinit")));

/*
   Next, we need to put some code to save reset cause from the bootload (in r2) to the variable. Again, avr-gcc 
	 provides special code sections for this.  If compiled with link time optimization (-flto), as done by the 
	 Arduno IDE version 1.6 and higher, we need the "used" attribute to prevent this from being omitted.
*/
void resetFlagsInit(void) __attribute__ ((naked))
__attribute__ ((used))
__attribute__ ((section (".init0")));
void resetFlagsInit(void)
{
  /*
     save the reset flags passed from the bootloader.  This is a "simple" matter of storing (STS) r2 in the 
		 special variable that we have created.  We use assembler to access the right variable.
  */
  __asm__ __volatile__ ("sts %0, r2\n" : "=m" (resetFlag) :);
}

void setup() {
   
		Serial.begin(9600);
		Serial.println("Starting");

  // Reset watchdog
  MCUSR = 0;
  wdt_disable();

	// Initialise wakeup to support background daemons (eg for time, temperature reading etc)
	wakeup.init();

	// Various setup routines
	setupComms();
	logStartupReason();
	setupTimePeriods();
	setupWindowSensors();
	setupTempSensors();

  // Start watchdog with 8 sec timeout
  wdt_enable(WDTO_8S);

	// Start the relative clock
	prevTime = millis();

	// Calculate current time and set onPeriod accordingly
	checkTime();
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

				// Calculate current time and set onPeriod accordingly
				if (heartBeatSecs % CHECK_TIME_FREQ == 0) checkTime();

				// Check WINDOW sensors
				if (heartBeatSecs % CHECK_WINDOWS_FREQ == 0) checkWindowSensors();

				// Check temperature sensors
				if (heartBeatSecs % CHECK_TEMP_SENSORS_FREQ == 0) checkTemperatureSensors();

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

			// Pattern BACKGROUND - 1 hour on/1 hour off for underfloor heating
			timePeriodStart[zone][BACKGROUND][0] = dhmMake(0, 6, 0);			// 2 hr slot to start day
			timePeriodEnd[zone][BACKGROUND][0] = dhmMake(0, 8, 0);
			timePeriodStart[zone][BACKGROUND][1] = dhmMake(0, 9, 0);			// 1 hr slot
			timePeriodEnd[zone][BACKGROUND][1] = dhmMake(0, 10, 0);
			timePeriodStart[zone][BACKGROUND][2] = dhmMake(0, 11, 0);			// 1 hr slot
			timePeriodEnd[zone][BACKGROUND][2] = dhmMake(0, 12, 0);
			timePeriodStart[zone][BACKGROUND][3] = dhmMake(0, 13, 0);			// 1 hr slot
			timePeriodEnd[zone][BACKGROUND][3] = dhmMake(0, 14, 0);
			timePeriodStart[zone][BACKGROUND][4] = dhmMake(0, 15, 0);			// 1 hr slot
			timePeriodEnd[zone][BACKGROUND][4] = dhmMake(0, 16, 0);
			timePeriodStart[zone][BACKGROUND][5] = dhmMake(0, 17, 0);			// 2 hr slot to start evening
			timePeriodEnd[zone][BACKGROUND][5] = dhmMake(0, 19, 0);
			timePeriodStart[zone][BACKGROUND][6] = dhmMake(0, 20, 0);			// 1 hr slot
			timePeriodEnd[zone][BACKGROUND][6] = dhmMake(0, 21, 0);
			timePeriodStart[zone][BACKGROUND][7] = dhmMake(0, 22, 0);			// 1 hr slot
			timePeriodEnd[zone][BACKGROUND][7] = dhmMake(0, 23, 0);
			numTimePeriods[zone][BACKGROUND] = 8;

			// Pattern AFTERNOON 
			timePeriodStart[zone][AFTERNOON][0] = dhmMake(0, 12, 0);
			timePeriodEnd[zone][AFTERNOON][0] = dhmMake(0, 23, 00);
			numTimePeriods[zone][AFTERNOON] = 1;

			// Pattern TURNOFF
			numTimePeriods[zone][TURNOFF] = 0;
	}
}

void setupWindowSensors() {

		// Window sensor input
		for (int i = 0; i < NUM_WINDOW_SENSORS; i++) {
				pinMode(wSensor[i], INPUT);
		}
}

void setupTempSensors() {

		// Provide power to sensors
		pinMode(TEMP_SENSOR_POWER_PIN, OUTPUT);
		digitalWrite(TEMP_SENSOR_POWER_PIN, LOW);
		delay(10);
		digitalWrite(TEMP_SENSOR_POWER_PIN, HIGH);
		delay(10);

		// Initialise sensors, get initial read and schedule regular background refresh - access latest temperature using getTempC()
		for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
				if (!tSensor[i].init(TEMP_SENSOR_PIN[i], TARGET_TEMP_PRECISION, CHECK_FREQ[i])) {
						logError(0xB0 | (i & 0x0F));          // 1st nibble == 'B', 2nd nibble == sensor number response
				}
		}
}

void processIncomingUDP() {
		SAVE_CONTEXT("cFInp")

		const byte MAX_MSGS = 5;						  // Max msgs to process before exiting while loop
		byte numMsgs = 0;
		unsigned int dataLen;

		byte bufPosn;
		char buffer[UDP_TX_PACKET_MAX_SIZE];
		#define BUF_ADD bufPosn += snprintf(buffer + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn,      // Take care; buffer used for input & output

		while (dataLen = ard.get(buffer, UDP_TX_PACKET_MAX_SIZE)) {        // If data available - multiple messages potentially

				if (dataLen < UDP_TX_PACKET_MAX_SIZE) buffer[dataLen] = 0x00;
				SENDLOGM('D', buffer);

				if (numMsgs++ > MAX_MSGS) {
						SENDLOG('W', "Max msgs exceeded - dropped: ", buffer);
						RESTORE_CONTEXT
						return;
				}

				bufPosn = 0;										// Position to start of buffer for output - overwrites redundant input

				switch (buffer[0]) {
						case 'F': {     // Force heating on/off - auto cancels at start of next programmed ON period
								byte zone;

								switch (buffer[1]) {
										case 'G':	zone = GREAT_HALL_MANIFOLD; break;
										default: SENDLOGM('W', "Invalid zone");
								}

								forceOn[zone] = buffer[2] == '1';               // Gets picked up at next time refresh

								break;
						}

						case 'P': {      // Set timePattern for zone
								char zone;

								switch (buffer[1]) {
										case 'G': zone = GREAT_HALL_MANIFOLD; break;
										default: SENDLOGM('E', "Invalid zone");
										}

								switch (buffer[2]) {
										case 'A': timePattern[zone] = ALLDAY; break;
										case 'B': timePattern[zone] = BACKGROUND; break;
										case 'N':	timePattern[zone] = AFTERNOON; break;
										case 'X':	timePattern[zone] = TURNOFF; break;
										default:     SENDLOGM('E', "Invalid timePattern");
								}
								break;
						}

						case 'S':        // Status request; response required
								switch (buffer[1]) {
										case 'P': {					// Current time pattern
												char programme = timePatternTag[timePattern[GREAT_HALL_MANIFOLD]];
												BUF_ADD "{\"DA\":\"%cP\", \"G\":\"%c\"}\0", meInit, (forceOn[GREAT_HALL_MANIFOLD]) ? 'F' : (onPeriod[GREAT_HALL_MANIFOLD]) ? programme : programme + 32);
												break;
										}

										case 'T': {		// Room temperatures
												byte limit = NUM_TEMP_SENSORS - 1;

												BUF_ADD "{\"DA\":\"%cT\",", meInit);
												for (int i = 0; i < (NUM_TEMP_SENSORS); i++) {
														if (tSensor[i].getTempC() == ERR_TEMP) BUF_ADD "\"%c\":\"**\"", tSensorTag[i]);
														else {
																unsigned int leftA, rightA;											// To convert temps from float to two-part ints
																leftA = (unsigned int)tSensor[i].getTempC();								// Take the integer portion
																rightA = (tSensor[i].getTempC() - (float)leftA) * 100;     // Subtract the integer portion from the total, then decimal shift
																BUF_ADD "\"%c\":\"%u.%u\"", tSensorTag[i], leftA, rightA);
														}
														if (i == limit) BUF_ADD "}\0"); else BUF_ADD ",");
												}
												Serial.println(buffer);
												break;
										}

										case 'W': {		// Window sensors
												byte limit = NUM_WINDOW_SENSORS - 1;
												
												BUF_ADD "{\"DA\":\"%cW\",", meInit);
												for (int i = 0; i < (NUM_WINDOW_SENSORS); i++) {
														unsigned int leftA, rightA;																						// To convert readings from float to two-part ints

														leftA = (unsigned int)wSensorVolts[i];																// Take the integer portion
														rightA = (wSensorVolts[i] - (float)leftA) * 100;											// Subtract the integer portion from the total, then decimal shift
														BUF_ADD "\"%c\":\"%u.%u\"", wSensorTag[i], leftA, rightA);

														if (i == limit) BUF_ADD "}\0"); else BUF_ADD ",");
												}
												break;
										}

										case 'X':				// Syslog notification level
												BUF_ADD "{\"DA\":\"%cX\", \"X\":\"%c\"}\0", meInit, syslogLevel);
												break;

										default: 
												BUF_ADD "Invalid status request: %c\0", buffer[1]);
												SENDLOGM('W', buffer);
								}

								// Reply to requestor
								ard.reply(REPLY_PORT, buffer, bufPosn);

								// Copy to syslog

								SENDLOGM('D', buffer);

								break;

						case 'X':
								if (strchr(ALLOWABLE_SYSLOG_LEVELS, buffer[1])) {
										syslogLevel = buffer[1];
										syslog.adjustLevel(syslogLevel);
								}
								else {
										BUF_ADD "Invalid syslog level: %c\0", buffer[1]);
										SENDLOGM('W', buffer);
								}
								break;
				}
		}

		RESTORE_CONTEXT
}

void checkTime() {		// Get current time, test if any zones are on and turn on zone onPeriod as appropriate
		SAVE_CONTEXT("CheckT")

		char timeText[30];

		// Get current time & pack into dhm format
		timeNow = dhmMake(weekday(), hour(), minute());

		// Send to syslog
		int bufEnd = snprintf(timeText, 30, "T %04X \0", timeNow);  
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

		// Clear force flag if moving from OFF to ON
		if (!onPeriod[zone] && newStatus) forceOn[zone] = false;

		return forceOn[zone] || newStatus;
}

void checkWindowSensors() {
	SAVE_CONTEXT("aInp")

  for (int i = 0; i < NUM_WINDOW_SENSORS; i++) {
			wSensorVolts[i] = analogRead(wSensor[i]) * MV_PER_TICK;

      char msg[] = { 'W', i + 48, '=', 0 };
//			SENDLOG('I', msg, wSensorVolts[i]);
  }

	RESTORE_CONTEXT
}

void checkTemperatureSensors() {            // Check temp sensors, reset if errors
		SAVE_CONTEXT("cTemp")

		byte errorCount = 0;

		// Check there are still free slots in wakeup stack
		if (!wakeup.freeSlots()) SENDLOGM('W', "Wakeup full");

		// See if sensors OK; if not power cycle and restart.  At present all sensors on same power line.  Intent is to move to dedicated power lines
		for (int i = 0; i < NUM_TEMP_SENSORS && !errorCount; i++) { 
				if (tSensor[i].getTempC() == ERR_TEMP) errorCount++;						// Count errors (and exit loop)
		}

		if (errorCount > 0) {
				setupTempSensors();
				SENDLOGM('N', "Temp sensors reset");
		}

		RESTORE_CONTEXT
}

void checkManifold() {      // See if manifolds want more heat - tell boiler
		char message[] = "ZG ";

		// Great Hall - just determine by timePeriod pending sensor input
		message[2] = (onPeriod[GREAT_HALL_MANIFOLD]) ? '1' : '0';
		ard.put(basementIP, UdpArdPort, message, 3);
}
