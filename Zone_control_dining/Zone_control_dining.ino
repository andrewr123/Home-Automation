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

- Pins 14, 15 & 16 - uses external voltage divider to sense if (KBE) light on/off - sends message to boiler to turn on/off DHW pump
- Pin 5 - uses output from relay to determine if Dining manifold wants more heat
- Pins 6, 7 & 8 are Dallas temp sensors (DUK)
- Pin 9 is power line for Dallas temp sensors - needed to avoid interference
- Pins 17, 18 & 19 are control pins (DCL) for high power shift register driving relays (ports DUK)

UDP commands: 
- Pzt		- Set timePattern for zone; z ==
					- D - Dining
					- U - Upper Corridor
					- K - Kitchen
					- G - Great Hall
					t ==
					- A - Allday
					- B - Background
					- N - Afternoon
					- X - Off
- Ss		- Status request; s ==
					- D - DHW switch status
					- T - temperatures
- Xn		- Syslog ON (n == '1') or OFF
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

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "HA_globals.h"
#include "HA_syslog.h"         // For syslog comms
#include "HA_queue.h"          // Needed to support syslog
#include "HA_comms.h"          // For general UDP comms

#include <OneWire.h>

#include "HA_time.h"
#include "Time.h"
#include "TimeLib.h"
#include "Wakeup.h"
#include "TimerOne.h"



// ****** Pin assignments ************

const static byte DHW_DEMAND[]             = {14, 15, 16};    // High if  switch is open (ie, off)
const static byte MANIFOLD_DEMAND          = 5;								// High if Dining manifold wants heat
const static byte SENSOR_PIN[]             = {6, 7, 8};				// Dallas temp sensors
const static byte SENSOR_POWER_PIN				 = 9;								// 5v power to Dallas temp sensors
const static byte DATA_PIN                 = 17;							// Control lines for relay driver
const static byte CLOCK_PIN                = 18;
const static byte LATCH_PIN                = 19;

// ********* Domestic Hot Water demand sensing - light switch on/off *************

byte anyDHWDemand = 0;																		// Set on interrogation of switches
const char DHW_TAG[]								= {'K', 'B', 'E'};    // Check with boiler_control before changing
const static byte NUM_DHW						= 3;

// *************  Temperature zones ***********

const static byte ZONE_DINING            = 0;      
const static byte ZONE_UPPER_CORR        = 1;      
const static byte ZONE_KITCHEN			 = 2;
const char sensorTag[]              = { 'D', 'U', 'K'};
const static byte NUM_SENSORS				= 3;
const static byte DINING_MANIFOLD		= 3;
const static byte NUM_ZONES = NUM_SENSORS + 1;		// To allow for dining manifold 'zone' as surrogate for all others (with directly wired sensors)

// Time patterns define whether zone is ON or OFF

const byte NUM_TIME_PATTERNS = 4;          // Number of different time patterns permitted
const byte ALLDAY = 0;								// 6am - 11pm
const byte BACKGROUND = 1;							// Start and end of day
const byte AFTERNOON = 2;							// Post noon
const byte TURNOFF = 3;	 							// Turn off
const char timePatternTag[] = { 'A', 'B', 'N', 'X' };

byte timePattern[NUM_ZONES] = { BACKGROUND, BACKGROUND, BACKGROUND, BACKGROUND };		// Set the time patterns for each zone - DUKdg

const byte MAX_TIME_PERIODS = 5;           // Number of ON periods allowed per day per time pattern

										   // Values set in setupHA()
unsigned int timePeriodStart[NUM_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];        // Start of time period
unsigned int timePeriodEnd[NUM_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];          // End of time period
byte numTimePeriods[NUM_ZONES][NUM_TIME_PATTERNS];

// Sensor data - Dallas DS18B20 
byte sensBuffer[9];																		// Buffer used for both address and data scratchpad
byte romFamily[NUM_SENSORS];                          // Holds the ROM family extracted from buffer
float tempC[NUM_SENSORS];                             // Holds the latest temperature from the sensor
byte sensorErrors[NUM_SENSORS];												// V6 addition - count number of errors
const byte MAX_SENSOR_ERRORS = 5;											// V6 addition
float targetC[NUM_SENSORS] = { 10, 10, 10 };  // Target temperature for each sensor (DUK) - can be adjusted using "Zsnn" command

const static float TOLERANCE = 0.5;
const static int MIN_TEMP = 1;            // Minimum credible temperature - less than this suggests sensor is faulty (or house very cold)
const static int MAX_TEMP = 50;           // Max allowable temp for sensors
const static int ERR_TEMP = 99;           // To indicate error reading
const static byte TEMPERATURE_PRECISION = 10;

// ***********  Adapted version of DALLAS LIB VERSION "3.7.2"   ************

OneWire oneWire[NUM_SENSORS];

// Model IDs
const static byte DS18S20 = 0x10;
const static byte DS18B20 = 0x28;
const static byte DS1822  = 0x22;
		
// OneWire commands
const static byte STARTCONVO 			= 0x44;  // Tells device to take a temperature reading and put it on the scratchpad
const static byte COPYSCRATCH     = 0x48;  // Copy EEPROM
const static byte READSCRATCH     = 0xBE;  // Read EEPROM
const static byte WRITESCRATCH    = 0x4E;  // Write to EEPROM
		
// ROM locations
const static byte ROM_FAMILY	  = 0;
const static byte ROM_CRC	  = 7;
		
// Scratchpad locations
const static byte TEMP_LSB        = 0;
const static byte TEMP_MSB        = 1;
const static byte HIGH_ALARM_TEMP = 2;
const static byte LOW_ALARM_TEMP  = 3;
const static byte CONFIGURATION   = 4;
const static byte INTERNAL_BYTE   = 5;
const static byte COUNT_REMAIN    = 6;  
const static byte COUNT_PER_C     = 7;
const static byte SCRATCHPAD_CRC  = 8;
		
// Device resolution
const static byte TEMP_9_BIT  = 0x1F; //  9 bit
const static byte TEMP_10_BIT = 0x3F; // 10 bit
const static byte TEMP_11_BIT = 0x5F; // 11 bit
const static byte TEMP_12_BIT = 0x7F; // 12 bit
		
// Conversion time - see data sheets
const static int T_CONV_9_BIT    = 200; //94;  
const static int T_CONV_10_BIT   = 300; //188; 
const static int T_CONV_11_BIT   = 450; // 375;  
const static int T_CONV_12_BIT   = 750;  
const static int T_CONV_DS18S20  = 750;

// *****************  On/off relays - match sensors ********************

// Relay ports on TPIC6595
const static byte RELAY_PORT[NUM_ZONES - 1]  = {0, 1, 2, 3};                  // Ports on TPIC same sequence as sensors + relay for dining manifold.  Ignore Great Hall
byte relayState                            = 0;                          // Initial state - all relays off

// ****** Master on/off control ******

boolean onPeriod[NUM_ZONES];                            // Set by time vs programme.  If ON then active; if OFF then go to stanbdy

// ****** Manifold sensors ******

byte kitchenManifold                 = OFF;
byte diningManifold                  = OFF;

// ****** Identity of this controller ********

#define meIP diningIP                   
#define meMac diningMac
char meName[] = "Dining";
const static char meInit = 'D';
boolean sendToSyslog = true;

char logMsg[UDP_TX_PACKET_MAX_SIZE];
byte bufPosn;

// ******  Time ******

// Current time is updated every 30 secs

unsigned int timeNow = 0;

// ******   Timing   *******
//
unsigned long prevTime;
unsigned int heartBeatSecs                 = 0;                
const unsigned int HEARTBEAT_FREQ_MS       = 1000;     // Heartbeat every second
const unsigned int CHECK_INPUT_FREQ        = 1;        // Check if any messages from console or other arduinos
const unsigned int CHECK_MANIFOLD_FREQ     = 5;        // Check if manifolds need heat and alert boiler
const unsigned int CHECK_TEMP_FREQ         = 10;       // Check temperature sensors and adjust solenoids
const unsigned int CHECK_DHW_FREQ          = 5;        // Check if DHW recirc needed and alert boiler
const unsigned int CHECK_SENSOR_STATUS     = 300;      // Check if sensors are OK; reset if not
const unsigned int CHECK_TIME_FREQ					= 30;				// Work out current time every 30 secs and act on it


void setup() {
 
  // Standard setup actions 
  setupHA();
  
  // Initialise DHW demand sensors
  for (int i = 0; i < 3; i++) pinMode(DHW_DEMAND[i], INPUT); 
  
  // Initialse manifold demand sensor
  pinMode(MANIFOLD_DEMAND, INPUT);
  
  // Initialise temperature sensors
	pinMode(SENSOR_POWER_PIN, OUTPUT);
	digitalWrite(SENSOR_POWER_PIN, HIGH);				// Applies 5v to sensors
	for (int i = 0; i < NUM_SENSORS; i++) {
			initSensorD(i, SENSOR_PIN[i]);
			sensorErrors[i] = 0;
	}
   
  // Setup relay pins/driver
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);  
  pinMode(CLOCK_PIN, OUTPUT);
  
  // Turn relays off - 1st time in the portstate is zero, so only need to do this for one relay
  turnRelay(0, OFF);
    
  // Capture relative time
  prevTime = millis();
}


void loop() {


    // Run any background daemons - needed to support HA_time.cpp
    wakeup.runAnyPending();

  // Increment heartbeat every second 
  if ((millis() - prevTime) >= HEARTBEAT_FREQ_MS) {   

    prevTime = millis();                  
    heartBeatSecs++;                                          

    // See if any commands from console or other arduinos
		if (heartBeatSecs % CHECK_INPUT_FREQ == 0) checkForInput();

		// Calculate current time
		if (heartBeatSecs % CHECK_TIME_FREQ == 0) checkTime();
    
    // Prepare buffer for reporting temperatures
    bufPosn = 0;
    memset(logMsg, '\0', UDP_TX_PACKET_MAX_SIZE);

    // See how room temperatures are doing - compare to target temp and set solenoid appropriately
    if (heartBeatSecs % CHECK_TEMP_FREQ == 0) checkRoomTemps();
    
    // See if sensors OK; if not then send message
		if (heartBeatSecs % CHECK_SENSOR_STATUS == 0) {
			digitalWrite(SENSOR_POWER_PIN, LOW);
			delay(50);
			digitalWrite(SENSOR_POWER_PIN, HIGH);
			delay(5);
			checkSensorStatus();
		}
      
    // See if manifolds want more heat - tell boiler
    if (heartBeatSecs % CHECK_MANIFOLD_FREQ == 0) checkManifolds();
         
    // See if any demand for DHW - if so then turn on recirc
    if (heartBeatSecs % CHECK_DHW_FREQ == 0) checkDHW();
    
    // And send aggregate results to log, if required       
    if (heartBeatSecs % CHECK_TEMP_FREQ == 0 && sendToSyslog) SENDLOGM('N', logMsg);
  }
  
  delay(100);

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

			// UFH TURNOFF
			numTimePeriods[zone][TURNOFF] = 0;
  }

	// Initialise wakeup to support background daemons (eg for time)
	wakeup.init();

	// Get the time from NTP server and broadcast it
	initialiseTime(UdpNTPPort);
	timeToText(now(), logMsg, UDP_TX_PACKET_MAX_SIZE);           // Get current time
	SENDLOG('I', "Startup finished @ ", logMsg)                    // Send to Syslog
}

void checkTime() {		// Get current time, test if any zones are on and turn on dining manifold if needed
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

		// Turn on dining manifold if any Dining-powered zones are on
		turnRelay(DINING_MANIFOLD, (onPeriod[ZONE_DINING] || onPeriod[ZONE_UPPER_CORR] || onPeriod[DINING_MANIFOLD]) ? ON : OFF);

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

void checkRoomTemps() {            // Check room temps and set soleniod appropriately
	float tempD;

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Get the temp
		tempD = getTempD(i);

		// v6 addition - delay a bit if showing error
		if (tempD == ERR_TEMP) {
				sensorErrors[i] += 1;
				if (sensorErrors[i] > MAX_SENSOR_ERRORS) tempC[i] = ERR_TEMP;   // Else leave as is
		}
		else {
				tempC[i] = tempD;
				sensorErrors[i] = 0;
		}

    if (tempC[i] != ERR_TEMP) {
      // If temp low then turn solenoid on, else off - sensitivity = 0.25, so 0.5 degree hyteresis
      if (onPeriod[i]) {
        if (tempC[i] > targetC[i]) turnRelay(i, OFF);
        if (tempC[i] < targetC[i]) turnRelay(i, ON);
      }
      else turnRelay(i, OFF);
      
      // Broadcast the temp, converting float to two ints (snprintf on Arduino doesn't support floats)
      int leftA, rightA;
      leftA = (int)tempC[i];              // Take the integer portion
      rightA = (tempC[i] - (float)leftA) * 100;     // Subtract the integer portion from the total, then decimal shift
      
      bufPosn += snprintf(logMsg + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn, "%c%u.%uv%d ", sensorTag[i], leftA, rightA, (int)targetC[i]); 
    }
    else {        // Sensor error
      // Turn solenoid off
      turnRelay(i, OFF);
      
      // Broadcast error
      bufPosn += snprintf(logMsg + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn, "%c**v%d ", sensorTag[i], (int)targetC[i]); 
    }
  }
} 

void checkSensorStatus() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (tempC[i] == ERR_TEMP) {
      if (!oneWire[i].reset()) SENDLOG('W', "Unresponsive sensor: ", i);
    }
  }
}

void checkManifolds() {      // See if manifolds want more heat - tell boiler
  // Dining manifold status based on output signal from manifold
  char message[] = "ZD ";
  diningManifold = digitalRead(MANIFOLD_DEMAND);
  message[2] = diningManifold ? '1' : '0';      // On or off
  ard.put(basementIP, UdpArdPort, message, 3);
  bufPosn += snprintf(logMsg + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn, diningManifold ? "D" : "d");
  
  // Kitchen manifold status deduced from temperature - insert extra blank to separate from DHW demand flags next
  message[1] = 'K';
  if (tempC[ZONE_KITCHEN] > targetC[ZONE_KITCHEN]) kitchenManifold = OFF;
  if (tempC[ZONE_KITCHEN] < targetC[ZONE_KITCHEN]) kitchenManifold = ON;      // If neither, then remains unchanged
  message[2] = (kitchenManifold && onPeriod[ZONE_KITCHEN]) ? '1' : '0';
  ard.put(basementIP, UdpArdPort, message, 3);
  bufPosn += snprintf(logMsg + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn, (message[2] == '1') ? "K " : "k ");
  
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
    bufPosn += snprintf(logMsg + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn, "%c", (message[2] == '1') ? DHW_TAG[i] : DHW_TAG[i] + 32);
  }
}


void initSensorD(byte sensorNum, byte pin) {
  oneWire[sensorNum].init(pin);
  delay(10);

  // Reset the bus and get the address of the first (& only) device to determine ROM family
  sensBuffer[ROM_CRC] = 0;
  oneWire[sensorNum].reset();
  oneWire[sensorNum].readROM(sensBuffer);

  romFamily[sensorNum] = sensBuffer[ROM_FAMILY];	// Hereafter, sensBuffer used to hold scratchpad, not address (saves space)

  // Set the resolution.  Have to write all three bytes of scratchpad, but as alarm function not used first two can be random
  switch (romFamily[sensorNum]) {
    case DS18S20: 	
      break;						// 9 bit resolution only
    case DS18B20:
    case DS1822:
      switch (TEMPERATURE_PRECISION) {					
      	case 12:		sensBuffer[CONFIGURATION] = TEMP_12_BIT; break;
      	case 11:		sensBuffer[CONFIGURATION] = TEMP_11_BIT; break;
      	case 10:		sensBuffer[CONFIGURATION] = TEMP_10_BIT; break;
      	case 9:			
      	default:		sensBuffer[CONFIGURATION] = TEMP_9_BIT; break;
      }
      
      oneWire[sensorNum].reset();
      oneWire[sensorNum].skip();					// Avoids the need to send the address (only on single device buses)
      oneWire[sensorNum].write(WRITESCRATCH);
      oneWire[sensorNum].write(sensBuffer[HIGH_ALARM_TEMP]);	
      oneWire[sensorNum].write(sensBuffer[LOW_ALARM_TEMP]);
      oneWire[sensorNum].write(sensBuffer[CONFIGURATION]);
      
      delay(10);
          
      break;
    default:
      romFamily[sensorNum] = 0;              // Indicates fault
  }
}


float getTempD(byte sensorNum) {
  unsigned int tConv;
  float tempReturn;
  
  // Start temperature conversion
  oneWire[sensorNum].reset();
  oneWire[sensorNum].skip();	
  delay(10);			
  oneWire[sensorNum].write(STARTCONVO);

  // Work out how long the sensor needs to work out temperature
  switch (romFamily[sensorNum]) {
    case DS18S20:
      tConv = T_CONV_DS18S20;
      break;
    case DS18B20:
    case DS1822:
      switch (TEMPERATURE_PRECISION) {
        case 12:   tConv = T_CONV_12_BIT; break;
        case 11:   tConv = T_CONV_11_BIT; break;
        case 10:   tConv = T_CONV_10_BIT; break;
        case 9:    tConv = T_CONV_9_BIT; break;
      }
      break;
    default: 
      tConv = 1;
  }

  // Wait a bit
  delay(tConv);     

  // Get the data into buffer
  if (readScratchPad(sensorNum)) { 		
    // Load the temperature to single variable and add extra resolution if needed
    unsigned int reading = (((unsigned int)sensBuffer[TEMP_MSB]) << 8) | sensBuffer[TEMP_LSB];  		
  		
    if (romFamily[sensorNum] == DS18S20) {			// Fixed 9 bit resolution expandable using 'extended resolution temperature' algorithm
      reading = (reading << 3) & 0xFFF0;				// Shift to same position as DS18B20 and truncate 0.5C bit
      reading = reading + 12 - sensBuffer[COUNT_REMAIN];		// Simplified version of Dallas algorithm 
    }
  
    // Convert reading to signed 1/10ths of centigrade
    tempReturn = (float)(reading >> 2) * 0.25;
    
    // Do final credibility test
    return (tempReturn > MAX_TEMP || tempReturn < MIN_TEMP) ? ERR_TEMP : tempReturn;
  }
  else return ERR_TEMP;  
}


boolean readScratchPad(byte sensorNum) {
  // Read the temperature
  oneWire[sensorNum].reset();
  oneWire[sensorNum].skip();
  oneWire[sensorNum].write(READSCRATCH);
  		
  for (int i = 0; i < 9; i++) sensBuffer[i] = oneWire[sensorNum].read();
  if (oneWire[sensorNum].crc8(sensBuffer, SCRATCHPAD_CRC) != sensBuffer[SCRATCHPAD_CRC]) {
    return false;
  }
  else return true;
}

void turnRelay(byte relay, byte state) {
  static boolean firstTime = true;
  static byte portState = 0;
  byte _prevState = relayState;

  // Cleanse data
  state &= 0x01;
  relay %= 8;
  
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
							case 'D':	zone = ZONE_DINING; break;
							case 'U': zone = ZONE_UPPER_CORR; break;
							case 'K': zone = ZONE_KITCHEN; break;
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

			case 'S':        // Status request
				displayMode = buffer[1];

				switch (displayMode) {
					case 'D':				// DHW recirc demand
						limit = NUM_DHW - 1;

						BUF_ADD "{\"DA\":\"%c%c\",", meInit, displayMode);
						for (int tag = 0; tag < (NUM_DHW); tag++) {
							BUF_ADD "\"%c\":\"%c\"", DHW_TAG[tag], (anyDHWDemand & _BV(tag)) ? DHW_TAG[tag] : DHW_TAG[tag] + 32); 
							if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
						}
						break;

					case 'T':			// Room temperatures
						limit = NUM_SENSORS - 1;

						BUF_ADD "{\"DA\":\"%c%c\",", meInit, displayMode);
						for (int tag = 0; tag < (NUM_SENSORS); tag++) {
							if (tempC[tag] == ERR_TEMP) BUF_ADD "\"%c\":\"**\"", sensorTag[tag]);
							else {
								unsigned int leftA, rightA;											// To convert temps from float to two-part ints
								leftA = (unsigned int)tempC[tag];								// Take the integer portion
								rightA = (tempC[tag] - (float)leftA) * 100;     // Subtract the integer portion from the total, then decimal shift
								BUF_ADD "\"%c\":\"%u.%u\"", sensorTag[tag], leftA, rightA);
							}
							if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
						}
						break;


					case 'Z':			// Room target temperatures.  '+' at end indicates heat demanded, '=' indicates target temp achieved
						limit = NUM_SENSORS - 1;

						BUF_ADD "{\"DA\":\"%c%c\",", meInit, displayMode);
						for (int tag = 0; tag < (NUM_SENSORS); tag++) {
							BUF_ADD "\"%c\":\"%u%c\"", sensorTag[tag], (int)targetC[tag], (tempC[tag] > targetC[tag]) ? '=' : '+');
							if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
						}
						break;
				}

				// Reply to requestor
				ard.reply(REPLY_PORT, buffer, bufPosn);

				// Copy to syslog
				
				if (sendToSyslog) SENDLOGM('N', buffer);

				break;

			case 'X':
				sendToSyslog = (buffer[1] == '1');
				break;
        
			case 'Z':                                 // Set target temp for zone
				sensorNum = 0;
				while ((buffer[1] != sensorTag[sensorNum]) && (sensorNum < NUM_SENSORS)) sensorNum++;      // Find matching sensor tag
				if (sensorNum < NUM_SENSORS) targetC[sensorNum] = (float)atoi((char*)buffer + 2);          // If found then set target temp
				break;
		}
  }
	RESTORE_CONTEXT
}

