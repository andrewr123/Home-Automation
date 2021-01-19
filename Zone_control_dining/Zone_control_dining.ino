/* Zone control for Dining and Kitchen zones

To do:
- sendlog context/restore
- check time functions  <<----

Version history
---------------

Dec 14 - v1 - baseline version
Feb 15 - v2 - remove time handler and replace with onPeriod received through basement controller
						- add functionality to respond to Status messages
Mar 15 - v3	- periodic reset of sensor power
						- Dining manifold master on/off signal
Apr 15 - v4 - more accurate temp reporting
						- target temp reporting with demand indicator

Jan 17 - v5 - zone-specific programmes (generate time locally, onPeriod an array)

Dec 18 - v6 - temporary Great Hall signal
						- change default temperatures
						- rolling average of sensor temps to smooth glitches

Nov 20 - v7 - remove Gt Hall signal (now has own controller)

Dec 20 - v8 - implement watchdog and incorporate HA_temperature library

- Pins 14, 15 & 16 - uses external voltage divider to sense if (KBE) light on/off - sends message to boiler to turn on/off DHW pump
- Pin 5 - uses output from relay to determine if Dining manifold wants more heat
- Pins 6, 7 & 8 are Dallas temp sensors (DUK)
- Pin 9 is power line for Dallas temp sensors - reset as needed to avoid interference
- Pins 17, 18 & 19 are control pins (DCL) for high power shift register driving relays (ports DUK)

UDP commands: 
- Pzt		- Set timePattern for zone; z ==
					- D - Dining
					- U - Upper Corridor
					- K - Kitchen
					t ==
					- A - Allday
					- B - Background
					- N - Afternoon
					- X - Off
- Fzn   - force heating ON (1) or OFF (!1)
- Ss		- Status request; s ==
					- D - DHW switch status
          - P - timePattern
					- T - temperatures
          - X - current syslog level
- Xn		- Syslog level
- Zznn	- set target temperature for zone z to nnC

Uses TPIC6595 shift register to switch 12v relays - 3 for individual solenoids + 1 for dining manifold

 
                                    TPIC6595 
                              -----------------------
         					    Gnd -> | 1             PGND 20 | <- Gnd
                       5v -> | 2 Vcc         LGND 19 | <- Gnd
               Mega pin 9 -> | 3 Ser in   Ser out 18 | -> TPIC6595 B pin 3
    Dining solenoid relay -> | 4 Drain 0  Drain 7 17 | <- Relay 7
 1st floor solenoid relay -> | 5 Drain 1  Drain 6 16 | <- Relay 6
   Kitchen solenoid relay -> | 6 Drain 2  Drain 5 15 | <- Relay 5
    Dining manifold relay -> | 7 Drain 3  Drain 4 14 | <- Relay 4
  								     5v -> | 8 ~SRCLR      SRCK 13 | <- Mega pin 11 (clock)
  									  Gnd -> | 9 ~G           RCK 12 | <- Mega pin 12 (latch)
  										Gnd -> | 10 PGND       PGND 11 | <- Gnd
  													  -----------------------

Relay pins:
0 - Dining heat demand signal to manifold
1 - 1st floor heat demand signal to manifold
2 - Kitchen heat demand signal to manifold
3 - Dining manifold master on/off signal

*/

#define UDP_TX_PACKET_MAX_SIZE 128			// Per override in OneDrive\Documents\Arduino\Libraries\Ethernet\EthernetUDP.h
#define DINING_CONTROLLER						    // Sets parameters in HA_globals.h
//#define DEBUG


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
#include "HA_temperature.h"



// SPECIFIC ELEMENTS - FOR THIS CONTROLLER
// ---------------------------------------

// ****** Identity of this controller ******

#define meIP diningIP               // 192.168.7.178         
#define meMac diningMac
char meName[] = "Dining";
const static char meInit = 'D';

// ****** Heating zones ******

const static byte NUM_ZONES = 4;                                    // To allow for dining manifold 'zone' as surrogate for all others (with directly wired sensors)
const static byte ZONE_DINING = 0;                                  // Zone ID & channel # on TPIC
const static byte ZONE_UPPER_CORR = 1;
const static byte ZONE_KITCHEN = 2;
const static byte DINING_MANIFOLD = 3;                              // Surrogate zone
const static char zoneTag[NUM_ZONES] = { 'D', 'U', 'K', 'd' };

const static byte NUM_CONTROLLABLE_ZONES = NUM_ZONES - 1;

float targetC[NUM_CONTROLLABLE_ZONES] = { 20, 16, 22 };                     // Target temperature for each controllable zone - can be adjusted using "Zsnn" command

// *************  Temperature sensors ***********

const static byte TEMP_SENSOR_PIN[NUM_TEMP_SENSORS] = { 34, 36, 38, 48 };		// DIO pins - Dallas sensors 
const static byte TEMP_SENSOR_POWER_PIN = 9;												        // DIO pin - 5v power to Dallas temp sensors

HA_temperature tSensor[NUM_TEMP_SENSORS];														        // Dallas sensor objects

const static char tSensorTag[NUM_TEMP_SENSORS] = { 'D', 'U', 'K', 'O' };

const static byte TARGET_TEMP_PRECISION = 10;												        // Equates to 0.25C - good enough

const static byte MANIFOLD_DEMAND = 5;								                      // DIO pin - high if DINING_MANIFOLD wants heat - surrogate for all zones without Dallas sensors

const static byte NUM_ZONE_SENSORS = NUM_TEMP_SENSORS - 1;                  // 'Outside' is not related to a controllable zone
const static byte tSensorZone[NUM_ZONE_SENSORS] = {
    ZONE_DINING,
    ZONE_UPPER_CORR,
    ZONE_KITCHEN
};

// ****** Manifold status ******

//byte kitchenManifold = OFF;
//byte diningManifold = OFF;

// ****** Solenoid activation ******

// Relay ports on TPIC6595 to activate specific manifold inputs and master manifold control
const static byte RELAY_PORT[NUM_ZONES] = { 0, 1, 2, 3 };           // Ports on TPIC same sequence as sensors + relay for dining manifold
byte relayState = 0;                                                // Initial state - all relays off

// ********* Domestic Hot Water demand sensing - light switch on/off *************

byte anyDHWDemand = 0;																		          // Set on interrogation of switches
const static byte NUM_DHW = 3;
const char DHW_TAG[NUM_DHW] = { 'K', 'B', 'E' };                    // Check with boiler_control before changing
const static byte DHW_DEMAND[NUM_DHW] = { 14, 15, 16 };             // DIO pins - high if switch is open (ie, off)

// ****** TPIC relay driver pins ************

const static byte DATA_PIN = 17;							                      // Control lines for relay driver
const static byte CLOCK_PIN = 18;
const static byte LATCH_PIN = 19;


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
const byte MAX_TIME_PERIODS = 5;           // Number of ON periods allowed per day per time pattern
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
byte timePattern[NUM_ZONES] = { ALLDAY, BACKGROUND, ALLDAY, BACKGROUND };		// Set the time patterns for each zone

// ******   Internal (relative) timing  *******

unsigned long prevTime;
unsigned int heartBeatSecs = 0;
const unsigned int HEARTBEAT_FREQ_MS        = 1000;		// 1 heartbeats per second
const unsigned int CHECK_INPUT_FREQ         = 1;			// Check if any messages from console or other arduinos
const unsigned int CHECK_TEMP_FREQ          = 10;
const unsigned int CHECK_TEMP_SENSORS_FREQ  = 60;						// Check temperature sensors every minute - restart if necessary
const unsigned int CHECK_DHW_FREQ           = 5;        // Check if DHW recirc needed and alert boiler
const unsigned int CHECK_MANIFOLD_FREQ      = 5;						// Check if manifolds need heat and alert boiler
const unsigned int CHECK_TIME_FREQ          = 30;							// Work out current time every 30 secs and act on it
const byte CHECK_FREQ[NUM_TEMP_SENSORS]     = { CHECK_TEMP_FREQ, CHECK_TEMP_FREQ, CHECK_TEMP_FREQ, CHECK_TEMP_FREQ };



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

  // Initialise wakeup to support background daemons (eg for time, temperature reading etc)
  wakeup.init();

  // Various setup routines
  setupComms();
  logStartupReason();
  setupTimePeriods();
  setupDHWDemandSensors();
  setupRelays();
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

        // Check temperature sensors
        if (heartBeatSecs % CHECK_TEMP_SENSORS_FREQ == 0) checkTemperatureSensors();

        // See how room temperatures are doing - compare to target temp and set solenoid appropriately
        if (heartBeatSecs % CHECK_TEMP_SENSORS_FREQ == 0) checkRoomTemps();

        // See if manifolds want more heat - tell boiler
        if (heartBeatSecs % CHECK_MANIFOLD_FREQ == 0) checkManifolds();

        // See if any demand for DHW - if so then turn on recirc
        if (heartBeatSecs % CHECK_DHW_FREQ == 0) checkDHW();

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
        timePeriodStart[zone][AFTERNOON][1] = dhmMake(10, 6, 0);
        timePeriodEnd[zone][AFTERNOON][1] = dhmMake(10, 23, 00);
        numTimePeriods[zone][AFTERNOON] = 2;

        // Pattern TURNOFF
        numTimePeriods[zone][TURNOFF] = 0;
    }
}

void setupDHWDemandSensors() {

    // Initialise DHW demand sensors
    for (int i = 0; i < NUM_DHW; i++) pinMode(DHW_DEMAND[i], INPUT);
}

void setupRelays() {

    // Setup relay pins/driver
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);

    // Turn relays off - 1st time in the portstate is zero, so only need to do this for one relay
    turnRelay(0, OFF);
}

void setupTempSensors() {

    // Initialse manifold demand sensor - surrogate for zones without Dallas temp sensors
    pinMode(MANIFOLD_DEMAND, INPUT);

    // Provide power to Dallas sensors
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
    SAVE_CONTEXT("pUDP")

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
                    case 'D':	zone = ZONE_DINING; break;
                    case 'U': zone = ZONE_UPPER_CORR; break;
                    case 'K': zone = ZONE_KITCHEN; break;
                    case 'd': zone = DINING_MANIFOLD; break;
                    default: SENDLOGM('W', "Invalid zone");
                }

                forceOn[zone] = buffer[2] == '1';               // Gets picked up at next time refresh

                break;
            }

            case 'P': {     // Set timePattern for zone
                byte zone;

                switch (buffer[1]) {
                    case 'D':	zone = ZONE_DINING; break;
                    case 'U': zone = ZONE_UPPER_CORR; break;
                    case 'K': zone = ZONE_KITCHEN; break;
                    case 'd': zone = DINING_MANIFOLD; break;
                    default: SENDLOGM('W', "Invalid zone");
                }

                switch (buffer[2]) {
                    case 'A': timePattern[zone] = ALLDAY; break;
                    case 'B': timePattern[zone] = BACKGROUND; break;
                    case 'N':	timePattern[zone] = AFTERNOON; break;
                    case 'X':	timePattern[zone] = TURNOFF; break;
                    default:     SENDLOGM('W', "Invalid timePattern");
                }
                break;
            }

            case 'S': {     // Status request
                char displayMode = buffer[1];

                switch (displayMode) {
                    case 'D': {			// DHW recirc demand
                        byte limit = NUM_DHW - 1;

                        BUF_ADD "{\"DA\":\"%c%c\",", meInit, displayMode);
                        for (int i = 0; i < NUM_DHW; i++) {
                            BUF_ADD "\"%c\":\"%c\"", DHW_TAG[i], (anyDHWDemand & _BV(i)) ? DHW_TAG[i] : DHW_TAG[i] + 32);
                            if (i == limit) BUF_ADD "}\0"); else BUF_ADD ",");
                        }
                        break;
                    }

                    case 'P': {					// Current timePatterns
                        byte limit = NUM_ZONES - 1;

                        BUF_ADD "{\"DA\":\"%c%c\",", meInit, displayMode);
                        for (int zone = 0; zone < NUM_ZONES; zone++) {
                            char programme = timePatternTag[timePattern[zone]];
                            BUF_ADD "\"%c\":\"%c\"", zoneTag[zone], (forceOn[zone]) ? 'F' : (onPeriod[zone]) ? programme : programme + 32);
                            if (zone == limit) BUF_ADD "}\0"); else BUF_ADD ",");
                        }
                        break;
                    }

                    case 'T': {		// Sensor temperatures
                        byte limit = NUM_TEMP_SENSORS - 1;

                        BUF_ADD "{\"DA\":\"%c%c\",", meInit, displayMode);
                        for (int i = 0; i < (NUM_TEMP_SENSORS); i++) {
                            if (tSensor[i].getTempC() == ERR_TEMP) BUF_ADD "\"%c\":\"**\"", tSensorTag[i]);
                            else {
                                int leftA, rightA;											// To convert temps from float to two-part ints
                                leftA = (int)tSensor[i].getTempC();								// Take the (truncated) integer portion
                                rightA = (tSensor[i].getTempC() - (float)leftA) * 100;     // Subtract the integer portion from the total, then decimal shift
                                BUF_ADD "\"%c\":\"%d.%u\"", tSensorTag[i], leftA, (rightA < 0) ? -rightA : rightA);
                            }
                            if (i == limit) BUF_ADD "}\0"); else BUF_ADD ",");
                        }
                        break;
                    }

                    case 'Z': {		// Zone target temperatures.  '+' at end indicates heat demanded, '=' indicates target temp achieved
                        byte limit = NUM_CONTROLLABLE_ZONES - 1;     

                        BUF_ADD "{\"DA\":\"%c%c\",", meInit, displayMode);
                        for (int i = 0; i < NUM_CONTROLLABLE_ZONES; i++) {
                            BUF_ADD "\"%c\":\"%u%c\"", zoneTag[i], (int)targetC[i], (tSensor[i].getTempC() > targetC[i]) ? '=' : '+');  // Fudge as comparing zone to sensor, but OK at the moment
                            if (i == limit) BUF_ADD "}\0"); else BUF_ADD ",");
                        }
                        break;
                    }
                }

                // Reply to requestor
                ard.reply(REPLY_PORT, buffer, bufPosn);

                // Copy to syslog

                SENDLOGM('D', buffer);

                break;
            }

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

            case 'Z': {                                // Set target temp for zone; no point for zone == 'd'
                byte zoneNum = 0;
                while ((buffer[1] != zoneTag[zoneNum]) && (zoneNum < NUM_CONTROLLABLE_ZONES)) zoneNum++;      // Find matching sensor tag
                if (zoneNum < NUM_CONTROLLABLE_ZONES) targetC[zoneNum] = (float)atoi((char*)buffer + 2);          // If found then set target temp
                break;
            }
        }
    }
    RESTORE_CONTEXT
}

void checkTime() {		// Get current time, test if any zones are on and turn on dining manifold if needed
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

void checkTemperatureSensors() {            // Check temp sensors, reset if errors
    SAVE_CONTEXT("cSens")

        byte errorCount = 0;

    // Check there are still free slots in wakeup stack
    if (!wakeup.freeSlots()) SENDLOGM('W', "Wakeup full");

    // See if sensors OK; if not power cycle and restart. 
    // At present all sensors on same power line.  Intent is to move to dedicated power lines
    for (int i = 0; i < NUM_TEMP_SENSORS && !errorCount; i++) {
        if (tSensor[i].getTempC() == ERR_TEMP) errorCount++;						// Count errors (and exit loop)
    }

    if (errorCount > 0) {
        setupTempSensors();
        SENDLOGM('N', "Temp sensors reset");
    }

    RESTORE_CONTEXT
}

void checkRoomTemps() {            // Check sensor temps and set soleniod appropriately
  SAVE_CONTEXT("cTemp")

  for (int i = 0; i < NUM_ZONE_SENSORS; i++) {      // Ignore outside sensor

    if (tSensor[i].getTempC() != ERR_TEMP) {
      // If temp low then turn solenoid on, else off - sensitivity = 0.25, so 0.5 degree hyteresis
      if (onPeriod[i]) {
        if (tSensor[i].getTempC() > targetC[i]) turnRelay(i, OFF);
        if (tSensor[i].getTempC() < targetC[i]) turnRelay(i, ON);
      }
      else turnRelay(i, OFF);
    }
    else {        // Sensor error
      // Turn solenoid off
      turnRelay(i, OFF);
    }
  }

  // Turn on dining manifold if any Dining-powered zones are on and require heat
  turnRelay(DINING_MANIFOLD, ((onPeriod[ZONE_DINING] && relayOn(ZONE_DINING)) || 
                              (onPeriod[ZONE_UPPER_CORR] && relayOn(ZONE_UPPER_CORR)) || 
                               onPeriod[DINING_MANIFOLD]) ? ON : OFF);

  RESTORE_CONTEXT
} 

void checkManifolds() {      // See if manifolds want more heat - tell boiler
  char message[] = "Zzs";

  // Dining manifold status based on output signal from manifold, itself a combination of messages from this controller & room thermostats
  //diningManifold = digitalRead(MANIFOLD_DEMAND);
  message[1] = 'D';
  message[2] = digitalRead(MANIFOLD_DEMAND) ? '1' : '0';      // On or off
  ard.put(basementIP, UdpArdPort, message, 3);
  
  // Kitchen manifold status deduced from temperature
  message[1] = 'K';
  //kitchenManifold = (relayOn(ZONE_KITCHEN)) ? ON : OFF;
  //if (tSensor[ZONE_KITCHEN].getTempC() > targetC[ZONE_KITCHEN]) kitchenManifold = OFF;
  //if (tSensor[ZONE_KITCHEN].getTempC() < targetC[ZONE_KITCHEN]) kitchenManifold = ON;      // If neither, then remains unchanged
  message[2] = (relayOn(ZONE_KITCHEN) && onPeriod[ZONE_KITCHEN]) ? '1' : '0';
  ard.put(basementIP, UdpArdPort, message, 3);  
}

void checkDHW() {      // Test if any demand for DHW - if so then send message to turn on recirc      
	// Clear existing flags
	anyDHWDemand = 0;

  // Test switch status; HIGH == OFF
  for (int i = 0; i < NUM_DHW; i++) {
    char message[] = "Dxn";
    message[1] = DHW_TAG[i];																// Load tag
    message[2] = (digitalRead(DHW_DEMAND[i])) ? '0' : '1';	// Load ON/OFF
		anyDHWDemand |= (message[2] == '1') ? _BV(i) : 0;						// Set internal flag for later status reporting

    ard.put(basementIP, UdpArdPort, message, 3);						// Send message then add to syslog message
  }
}

void turnRelay(byte relay, byte state) {
  SAVE_CONTEXT("tRel")

  static boolean firstTime = true;
  static byte portState = 0;
  byte _prevState = relayState;
  char buffer[16];

  // Cleanse data
  state &= 0x01;
  relay %= 8;

  // Send to syslog
  snprintf(buffer, 16, "Rly %u %u", relay, state);
  SENDLOGM('D', buffer);
  
  
  // Set/unset relevant bit in relay mimic
  switch (state) {
    case OFF:     
      relayState &= ~(1 << relay); 
      portState &= ~(1 << RELAY_PORT[relay]); 
      break;
    case ON:      
      relayState |= (1 << relay);
      portState |= (1 << RELAY_PORT[relay]);
  }
  
  if (_prevState != relayState || firstTime) {
    
    // Disconnect shift register(s) from latch
    digitalWrite(LATCH_PIN, LOW);
  		  
    // Force clock low initially to ensure a rising edge for the clock
    digitalWrite(CLOCK_PIN, LOW);				
  		  
    // Write the state to the shift register, and thus turn on/off the relevant pin								
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, portState);
   		  
    // Latch shift register(s) onto outputs
    digitalWrite(LATCH_PIN, HIGH);
    
    // Clear one-off flag
    firstTime = false;
  }

  RESTORE_CONTEXT
}

boolean relayOn(byte relay) {
    return relayState & _BV(relay);
}