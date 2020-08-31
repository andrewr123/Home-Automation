
/*
BOILER CONTROLLER
-----------------

Uses TWI interface to communicate with the outside world.  Avoids the need for an Ethernet shield on what is a crowded board

Version history
---------------

Apr 13 - v1 - baseline version
Feb 15 - v2 - remove time handler and replace with onPeriod received through basement controller
Apr 15 - v3 - increase temp differential on mixer flow & return
Dec 16 - v4 - shutdown if no contact from outside world in 5 mins
			- remove concept of a global ON and OFF period or special treatment for DHW - if a zone demands heat then it gets it
			- add display of boiler primary circuit pressure
Jan 17 - v4a - turn off burner if pressure low
Nov 18 - v5 - new boiler - remove boiler or DHW temperature monitoring 
Jun 20 - v6 - improve error handling on comms loss - re-start comms
Test line



Uses 74HC595 shift register to switch opto-isolated relay bank (http://www.hobbytronics.co.uk/8-channel-relay-board?keyword=relay) - minimal current draw

                          74HC595 
                       ------------------
  Mixer open relay <- | 1 QB    Vcc   16 | <- 5v
 Mixer close relay <- | 2 QC      QA  15 | -> Burner relay - NOW HEAT DEMAND
     4067 A pin 13 <- | 3 QD      SER 14 | <- Arduino pin 13 (data in)
     4067 B pin 10 <- | 4 QE      ~OE 13 | <- Gnd
     4067 B pin 11 <- | 5 QF     RCLK 12 | <- Arduino pin 12 (latch)
     4067 B pin 14 <- | 6 QG    SRCLK 11 | <- Arduino pin 11 (clock)
     4067 B pin 13 <- | 7 QH   ~SRCLR 10 | <- 5v
               Gnd -> | 8 Gnd      ~QH 9 | -> 74HC595 cascade (N/C)
                       ------------------

LCD display:

Pin     Symbol	Function
1	Vss	Display power ground
2	Vdd	Display power +5V
3	Vo	Contrast Adjust. Altered by adjusting the voltage to this pin, grounding it sets it to maximum contrast.
4	RS	Register select
5	R/W	Data read/write selector. Its possible to read information from a display however there's no need for it here so grounding it sets it permanently write.
6	E	Enable strobe
7	DB0	Data Bus 0
8	DB1	Data Bus 1
9	DB2	Data Bus 2
10	DB3	Data Bus 3
11	DB4	Data Bus 4
12	DB5	Data Bus 5
13	DB6	Data Bus 6
14	DB7	Data Bus 7
15	A       LED backlight power +5V
16	K	LED backlight power ground

Pin usage (max on Uno = 22)


Digital pins
------------

  D0 - D1 - reserved for use by USB link and biased with 1k resistors - effectively off limits

  D2 - Boiler - NOT NEEDED WITH NEW BOILER
  D3 - UFH flow
  D4 - UFH return
  D5 - dhw tank
  D6 - dhw flow
  D7 - spare
 
Display driver using 595 shift register
  D8  - latch
  D9  - data
  D10 - clock
  
Relay driver using 595 shift register
  D11 - latch
  D12 - data 
  D13 - clock
  
  Relays:
  - burner - NOT NEEDED WITH NEW BOILER, REPURPOSED TO HEAT DEMAND
  - mixer open
  - mixer close
  - dhw primary pump - NOT NEEDED WITH NEW BOILER
  - ufh pump
  - primary pump
  - recirc pump
  
Analog pins
-----------

A0 - on-off signal - 5v == ON, 0v == OFF
A1 - spare
A2 - spare
A3 - spare
A4 & A5 - TWI

*/

#include <LiquidCrystal.h>
#include <OneWire.h>
#include <Wire.h>
#include <math.h>
#include "Bitstring.h"

#include "HA_globals.h"

// ***** System mode *****

enum eSystemMode {
	STARTUP,
	ERROR,					  // Comms lost - shutdown
	RUNNING,                  // Normal mode
	QUIESING,                 // Heading to dormant, but boiler temp too high so cooling with pumps
	DORMANT,                  // Waiting for demand signal
	MANUAL                    // Manual override
};

eSystemMode systemMode;
const char modeTag[] = {'S', 'E', 'R', 'Q', 'D', 'M'};

// ***** Identity - on JSON strings *****
const static char meInit = 'B';
  
// *************  Temperature sensors - Dallas DS18B20  ***********

const static byte BOILER            = 0;				// NOT NEEDED WITH NEW BOILER    
const static byte UFH_FLOW          = 1;      
const static byte UFH_RETURN        = 2;      
const static byte DHW_TANK          = 3; 				// NOT NEEDED WITH NEW BOILER   
const static byte PRI_RETURN        = 4;
const char sensorTag[] = { 'B', 'U', 'u', 'D', 'T'};
const static byte NUM_SENSORS       = 5;
  
const static int MIN_TEMP           = 1;            // Minimum credible temperature - less than this suggests sensor is faulty (or house very cold)
const static int MAX_TEMP           = 95;           // Max allowable temp for sensors
const static int ERR_TEMP           = 99;           // To indicate error reading

// Sensor pins
const static byte SENSOR_PIN[NUM_SENSORS] = {2, 3, 4, 5, 6};

// Sensor flags
const static byte BOILER_SENSOR     = B00000001;
const static byte UFH_SENSOR        = B00000110;
const static byte DHW_SENSOR        = B00001000;
const static byte ALL_SENSORS       = 0xff;

// OneWire instances to communicate with any OneWire devices 
OneWire oneWire[NUM_SENSORS];
const static byte TEMPERATURE_PRECISION = 10;

// Sensor data
byte sensBuffer[9];		        // Buffer used for both address and data scratchpad
byte romFamily[NUM_SENSORS];            // Holds the ROM family extracted from buffer
float tempC[NUM_SENSORS];               // Holds the latest temperature from the sensor

// *****************  On/off relays  ********************

const static byte BURNER             = 0;
const static byte MIXER_OPEN         = 1;
const static byte MIXER_CLOSE        = 2;
const static byte DHW_PRIMARY_PUMP   = 3;
const static byte UFH_PUMP           = 4;
const static byte PRI_CIRCUIT_PUMP   = 5;
const static byte DHW_RECIRC_PUMP    = 6;
const static byte INTERNET           = 7;          // Extra relay, not part of boiler control
const char relayTag[] = { 'B', 'O', 'C', 'D', 'U', 'P', 'R', 'I'};
const static byte NUM_RELAYS         = 8;
  
// Relay ports on 595
const static byte RELAY_PORT[NUM_RELAYS] = {0, 7, 6, 5, 4, 3, 2, 1};
byte relayState                      = 0;                          // Initial state - all relays off
byte manualState                     = 0;                          // Target relay state in manual model

// Control lines for relay driver
const unsigned int DATA_PIN          = 13;
const unsigned int CLOCK_PIN         = 11;
const unsigned int LATCH_PIN         = 12;

// ***************** Zone heat demand ***************

volatile byte anyZoneDemand          = 0;                  // Updated through Wire interface - _BV per following flags:
const static byte ZONE_KITCHEN       = 0x01;
const static byte ZONE_GT_HALL       = 0x02;
const static byte ZONE_BASEMENT      = 0x04;
const static byte ZONE_DINING        = 0x08;
const static byte ZONE_STUDY         = 0x10;
const static byte ZONE_DHW           = 0x20;              // Treated as dummy zone
const char zoneTag[]                 = {'K', 'G', 'B', 'D', 'S', 'H'};
const static byte NUM_ZONES          = 6;
const static byte UFH_ZONES			 = 0x1F;			// Mask to test if any UFH zones demanding heat

// *************  Boiler  ************

const static int BOILER_MAX                = 70;           // Degrees C at which to switch off boiler
const static int BOILER_RUN_ON_TEMP        = 75;           // If above this, then activate run-on mode to dissipate heat
const static int BOILER_MIN                = 62;           // If below this then switch on boiler
boolean NEWBOILER = true;  // If new boiler, then ignore boiler temperature checks

// ************** PRIMARY CIRCUIT ************

const static int PUMP_THRESHOLD            = 45;           // Below this don't take heat from boiler

// **************  UFH *************

const static byte STABLE                   = 0;
const static byte OPENING                  = 1;
const static byte CLOSING                  = 2;
const char mixerTag[]                      = {'S', 'T', 't', 'F', 'f', 'Z'};
const static byte NUM_MIXERS               = 6;

const static int UFH_INCREMENT             = 15;           // Added to current return temperature to give target flow temperature
const static int UFH_FLOW_MAX_TEMP         = 50;           // Degrees C at which to turn off mixer

int mixerPosition;                                         // 0 - 35s open/close in theory; in practice seems closer to 40s
byte mixerState;                                           // One of OPENING/CLOSING/STABLE/ZEROING
int targetMixerTemp                        = 30;           // Initial value to prevent odd behaviour if starting with hot boiler

// ********* Domestic Hot Water  *************

byte anyDHWDemand                          = 0;            // Updated through Wire interface
const static byte DHW_KITCHEN              = 0x01;
const static byte DHW_BATHROOM             = 0x02;
const static byte DHW_STUDY                = 0x04;
const static byte DHW_UPPER                = 0x08;
const static byte DHW_ENSUITE              = 0x10;
const char DHWTag[]                        = {'K', 'B', 'S', 'U', 'E'};
const static byte NUM_DHW                  = 5;

byte prevAnyDHWDemand                      = 0;
boolean newDHWDemand                       = false;        // Set in receiveData if have fresh demand for DHW  
int recircStart                            = 0;            // Heartbeat when recirc was started
const static int DHW_TIMEOUT               = 30 * 60;      // Default 30 mins
const static int DHW_NORM_TEMP             = 55;           
const static int DHW_HIGH_TEMP             = 60;
const static int DHW_THRESHOLD_TEMP        = 40;           // Prioritise DHW over other circuits until reaches this temp
const static float TOLERANCE               = 2;            // Tolerance on DHW temp readings to prevent hysteresis
float dhwMaxTemp                           = DHW_NORM_TEMP;  // Max temp allowed in tank; default is 55, but once a week goes to 60 to avoid legionella
boolean onPeriodH						   = FALSE;			// Used to flag that DHW is ON or OFF

// ******* Boiler pressure ******
byte centibars								= 0;			// Boiler primary circuit water pressure
const static byte MIN_PRESSURE = 10;		// 0.1 bar

// *************  Fault handling & startup flags  ***********
byte somethingWrong                        = 0;            // If BV == 1, then display flag (and shutdown if critical)
const char faultTag[]                      = { 'B', 'U', 'u', 'D'};
const static byte NUM_FAULT_FLAGS          = 4;
byte startupPending                        = 0;            // Flags cleared as each relay goes through startup
byte timeReceived						   = FALSE;		   // Supports watchdog action on comms with outside world

// ***********  Adapted version of DALLAS LIB VERSION "3.7.2"   ************

// Model IDs
const static byte DS18S20 = 0x10;
const static byte DS18B20 = 0x28;
const static byte DS1822  = 0x22;
		
// OneWire commands
const static byte STARTCONVO 	  = 0x44;  // Tells device to take a temperature reading and put it on the scratchpad
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

// ******   Timing   *******
//
unsigned long prevTime;
unsigned int heartBeatSecs                   = 0;          
unsigned int minsInMode                      = 0;
unsigned int minsSinceComms = 0;
const unsigned int HEARTBEAT_FREQ_MS         = 1000;     // Heartbeat every second
const unsigned int TEMP_CHECK_FREQ           = 11;       // Check temp sensors 
const unsigned int UFH_TEMP_CHECK_FREQ       = 2;        // Read mixer temps
const unsigned int LCD_RESTART_FREQ          = 30;       // Restart to overcome noise upsetting LCD - to be solved, hopefully, with better hardware design
const unsigned int DHW_DEMAND_CHECK_FREQ     = 5;        // Check if DHW demand
const unsigned int ROUTER_RESTART_CHECK_FREQ = 10;
const unsigned int ONE_MINUTE                = 60;
const unsigned int MANUAL_TIMEOUT_MINS       = 10;        // Minutes allowed in MANUAL; then set to RUNNING
const unsigned int COMMS_TIMEOUT_MINS		 = 5;		  // If no comms for 5 mins then assume something wrong - commence shutdown


// ****************  LCD display  ***************************

const static byte ROW_0 = 0;          // 1st row for temperature
const static byte ROW_1 = 1;
const static byte ROW_2 = 2;
const static byte ROW_3 = 3;

LiquidCrystal lcd(10, 8, 9);             // Modified version of libary using 595 shift registers.  Data, clock, latch  


// ********** Status map - used for output to LCD and to Wire **********

const byte NUM_COLS                   = 20;          // In practice, only 19 cols seem to be usable on the display
const byte NUM_ROWS                   = 4;
char statusMap[NUM_ROWS][NUM_COLS + 1];
BITSTRING statusChanged(NUM_ROWS * NUM_COLS);        // BV == 1 indicates status has changed since last displayed
volatile char statusSelect;                          // Set in receiveData, used in reportStatus
volatile int displaySelect;                          // Set in receiveData, used in reportStatus

// ******  Time - updated every 30 secs in dhm format

unsigned int timeNow = 0;							 // Set by HA_Time functions.  Current time in dhm format; used for display & to test for higher DHW target on Monday's
char timePatternTag[2] = { 'X', 'X' };                       // Set by message for display purposes only.  Same as in basement controller, plus 'F' for force

void setup(void) {

  // Start display
  lcd.begin(NUM_COLS, NUM_ROWS);
  lcd.print("Starting");
  
  // Start up Two Wire Interface, with this as device #2
  Wire.begin(2);
  Wire.onRequest(reportStatus);
  Wire.onReceive(receiveData);

  // Initialise sensors
  for (int i = 0; i < NUM_SENSORS; i++) initSensorD(i, SENSOR_PIN[i]);
  somethingWrong = 0;
    
  // Setup relay pins/driver
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);  
  pinMode(CLOCK_PIN, OUTPUT);
  
  // Kick off initial set of readings and evaluations
  getTempAll(ALL_SENSORS);                             // Read all sensors

  // Go into startup mode
  systemMode = STARTUP;
  startupPending = _BV(BURNER) | _BV (MIXER_OPEN) | _BV(DHW_PRIMARY_PUMP) | _BV(UFH_PUMP) | _BV(PRI_CIRCUIT_PUMP) | _BV(DHW_RECIRC_PUMP);
	startupPending &= ~_BV(DHW_PRIMARY_PUMP);          // Clear startup flag - unused

  // Initial display
  updateStatus();
  displayStatus();
  
  // Capture relative time
  prevTime = millis();
}


void loop(void) { 

  // Increment heartbeat every second 
  if ((millis() - prevTime) >= HEARTBEAT_FREQ_MS) {   
    
    prevTime = prevTime + HEARTBEAT_FREQ_MS;                  // Avoid normal = millis() to allow catchup
    heartBeatSecs++;                                          // Eventual overflow @ 64k not material

	if (heartBeatSecs % ONE_MINUTE == 0) {
		minsInMode++;
		if (timeReceived) {									// Should be received every 30 secs if all well
			minsSinceComms = 0;
			timeReceived = FALSE;
		}
		else {
			minsSinceComms++;
			if (minsSinceComms > COMMS_TIMEOUT_MINS) {			// 5 mins and no time signal; assume comms problem
				systemMode = ERROR;							
				minsInMode = 0;
			}
		}
	}
    
    if (heartBeatSecs % DHW_DEMAND_CHECK_FREQ == 0) controlRecirc();       // If DHW demand then turn on recirc pump & start countdown timer
    
    if (heartBeatSecs % UFH_TEMP_CHECK_FREQ == 0) {           // Runs more frequently than main temp check loop to avoid over/under shoot on mixer valve
      getTempAll(UFH_SENSOR);                                 // Confine to UFH sensors to save time
      controlMixer();                                         // React to actual temperature
    }
        
    if (heartBeatSecs % TEMP_CHECK_FREQ == 0) {               // Main loop - relatively infrequent as temps don't change too rapidly
      // Attempt to restart any faulty digital sensors
      for (int i = 0; i < NUM_SENSORS; i++) if (fault(i)) {
        clearFault(i);          // Clear flag
        initSensorD(i, SENSOR_PIN[i]);            // Attempt to re-start
      }
 
      // Get latest sensor temperatures
      getTempAll(ALL_SENSORS);

      // Check boiler temp is OK and react
      controlBurner();        
     
      // Check domestic hot water 
 //     checkDHWStatus();
      
      // Check underfloor heating 
      controlUFHPump();
      
      // Check primary flow
      controlPriPump();
    }
              
    updateStatus();
    
    if (heartBeatSecs % LCD_RESTART_FREQ == 0) {
      lcd.begin(NUM_COLS, NUM_ROWS);                
      statusChanged.setAll();                              // Causes a refresh in displayStatus
    }
    
    displayStatus();
    
    setSystemMode();
  }

  delay(50);    
}



void setSystemMode() {                                // Sets the system mode.  Also set on entry to manual mode

  eSystemMode prevSystemMode = systemMode;
  
  switch(systemMode) {
    case STARTUP:                                      // Startup mode cleared after a few seconds once all devicies initialised
      if (!startupPending) systemMode = DORMANT;    
      break;

    case RUNNING:
				if (!anyZoneDemand) systemMode = DORMANT;  
      break;

	case ERROR:
		if (minsSinceComms < COMMS_TIMEOUT_MINS) systemMode = DORMANT;		// If comms restored then can restart
    else {                                                            // Continuing error - re-start TWI
        // Re-start Two Wire Interface, with this as device #2 - added Jun 20
        Wire.begin(2);
        Wire.onRequest(reportStatus);
        Wire.onReceive(receiveData);
    }
		break;
      
    case QUIESING:
      if (anyZoneDemand) systemMode = RUNNING;
      else systemMode = DORMANT;
      break;
      
    case DORMANT:
      if (anyZoneDemand) systemMode = RUNNING;    
			else systemMode = DORMANT;
      break;
      
    case MANUAL:
      if (minsInMode > MANUAL_TIMEOUT_MINS) systemMode = RUNNING;
  }
  
  if (systemMode != prevSystemMode) minsInMode = 0;
}


void getTempAll(byte liveSensor) {

  for (int i = 0; i < NUM_SENSORS; i++) {
    
    if ((liveSensor & _BV(i)) && !fault(i)) {
      // Get the temperature
      tempC[i] = getTempD(i);

      // Constrain range if obviously erroneous and raise fault flag 
      if (tempC[i] > MAX_TEMP || tempC[i] < MIN_TEMP) {
        tempC[i] = ERR_TEMP;
        flagFault(i);
      }
    }
  }
}

void controlBurner() {
  
  // If sensor faulty or over-temp or underpressure then shutdown
		/*
  if (centibars < MIN_PRESSURE) {
    turnRelay(BURNER, OFF);
    return;
  }  */

  switch (systemMode) {
    case STARTUP:
      // Make sure boiler is off
      turnRelay(BURNER, OFF);
      startupPending &= ~_BV(BURNER);          // Clear startup flag
      break;

    case RUNNING: 
      /* if (anyZoneDemand && tempC[BOILER] <= BOILER_MIN) */
				turnRelay(BURNER, ON);
      break;

	case ERROR:
    case QUIESING:
    case DORMANT:
      // Make sure boiler is off
      turnRelay(BURNER, OFF);
      break;
      
    case MANUAL:
      turnRelay(BURNER, (manualState & _BV(BURNER)) ? ON : OFF);
      break;
  }
}

void controlPriPump(){
  
  switch (systemMode) {
    
    case STARTUP:
      turnRelay(PRI_CIRCUIT_PUMP, OFF);
      startupPending &= ~_BV(PRI_CIRCUIT_PUMP);          // Clear startup flag
      break; 
     
    case RUNNING:
      // If demand for heat then turn on pump //, then check if boiler & DHW (if demanded) up to temp before taking heat
      if ((anyZoneDemand & (ZONE_DINING | ZONE_STUDY))) turnRelay(PRI_CIRCUIT_PUMP, ON);
/*       && (tempC[BOILER] > PUMP_THRESHOLD)
        && (!(anyZoneDemand & ZONE_DHW) || (tempC[DHW_TANK] > DHW_THRESHOLD_TEMP))) turnRelay(PRI_CIRCUIT_PUMP, ON); */
      else turnRelay(PRI_CIRCUIT_PUMP, OFF);
      break;
      
	case ERROR:
	case QUIESING: 
    case DORMANT:
      turnRelay(PRI_CIRCUIT_PUMP, OFF);
      break;
    
    case MANUAL:
      turnRelay(PRI_CIRCUIT_PUMP, (manualState & _BV(PRI_CIRCUIT_PUMP)) ? ON : OFF);
      break;
  }
}

void controlUFHPump(){
  
  switch (systemMode) {
        
    case STARTUP:
      turnRelay(UFH_PUMP, OFF);
      startupPending &= ~_BV(UFH_PUMP);          // Clear startup flag
      break;
    
    case RUNNING:
      // Shutdown pump if error with UFH sensors  
      if (fault(UFH_FLOW) || fault(UFH_RETURN)) {
        turnRelay(UFH_PUMP, OFF);
        return;
      }
      
      // If demand for heat, then check if boiler & DHW (if demanded) up to temp before taking heat
      if (anyZoneDemand & (ZONE_GT_HALL | ZONE_KITCHEN | ZONE_BASEMENT)) turnRelay(UFH_PUMP, ON);
//        && (!(anyZoneDemand & ZONE_DHW) || (tempC[DHW_TANK] > DHW_THRESHOLD_TEMP))) turnRelay(UFH_PUMP, ON);  // No longer needed
      else turnRelay(UFH_PUMP, OFF);
      break;

	case ERROR:
	case QUIESING: 
    case DORMANT:
      turnRelay(UFH_PUMP, OFF);
      break;
      
    case MANUAL:
      turnRelay(UFH_PUMP, (manualState & _BV(UFH_PUMP)) ? ON : OFF);
      break;
  }
}

void controlMixer() {                                // Adjusts the mixer valve to hit the target flow temperature
  const unsigned int UPPER_THRESHOLD       = 2;      // To avoid hystereses
  const unsigned int LOWER_THRESHOLD       = 1;
  const unsigned int MAX_ACTUATION_TIME    = 10;     // Max length of time to run actuator 
  const unsigned int CHILL_TIME            = 10;     // Length of time to wait for temperature to stabilise
  static unsigned int lastChangeOfState    = 0;
  static unsigned int actuationTime        = 0;      // Length of time to run actuator, before chilling = 0.4 x temp diff + 1
  
  unsigned int timeSinceLast = heartBeatSecs - lastChangeOfState;

  switch (systemMode) {
    
    case STARTUP:
      turnRelay(MIXER_OPEN, OFF);
      turnRelay(MIXER_CLOSE, OFF); 
      mixerState = STABLE;
      startupPending &= ~_BV(MIXER_OPEN);          // Clear startup flag
      break;

    case RUNNING:
      
      // If mixer sensor error, then return
      if (fault(UFH_FLOW) || fault(UFH_RETURN)) return;
      
      // If pump running (ie heat is required), then check temperatures
      if (relayOn(UFH_PUMP)) {
          
        // Figure out the target mixer temp  
        targetMixerTemp = min(UFH_FLOW_MAX_TEMP, (int)tempC[UFH_RETURN] + UFH_INCREMENT);
      
        switch (mixerState) {
          case STABLE:
            if (timeSinceLast >= CHILL_TIME) {
              // If above target then reduce
              if (tempC[UFH_FLOW] >= (targetMixerTemp + UPPER_THRESHOLD)) {
                turnRelay(MIXER_OPEN, OFF);
                turnRelay(MIXER_CLOSE, ON);
                mixerState = CLOSING;
                lastChangeOfState = heartBeatSecs;
                actuationTime = min(0.4 * (tempC[UFH_FLOW] - (float)(targetMixerTemp)) + 1, MAX_ACTUATION_TIME);
              }
              
              // If below target then increase  
              if (tempC[UFH_FLOW] < (targetMixerTemp - LOWER_THRESHOLD)) {
                turnRelay(MIXER_CLOSE, OFF);
                turnRelay(MIXER_OPEN, ON);
                mixerState = OPENING;
                lastChangeOfState = heartBeatSecs;
                actuationTime = min(0.4 * ((float)(targetMixerTemp) - tempC[UFH_FLOW]) + 1, MAX_ACTUATION_TIME);
              }        
            }
            break;
          case OPENING:
          case CLOSING:
            if (timeSinceLast >= actuationTime) {          
              turnRelay(MIXER_CLOSE, OFF);
              turnRelay(MIXER_OPEN, OFF);
              mixerState = STABLE;
              lastChangeOfState = heartBeatSecs;
            }
            break;
        }
        
        return;
      }
      
      // UFH pump is off; just chill until it comes back on
      turnRelay(MIXER_CLOSE, OFF);
      turnRelay(MIXER_OPEN, OFF);
      mixerState = STABLE;
      lastChangeOfState = heartBeatSecs;
      break;  
      
	case ERROR:
	case QUIESING: 
    case DORMANT:
      // Turn off 
      turnRelay(MIXER_OPEN, OFF);
      turnRelay(MIXER_CLOSE, OFF); 
      mixerState = STABLE;
      break;
          
    case MANUAL:
      turnRelay(MIXER_CLOSE, (manualState & _BV(MIXER_CLOSE)) ? ON : OFF);
      turnRelay(MIXER_OPEN, (manualState & _BV(MIXER_OPEN)) ? ON : OFF);
      break;
  }
}

/*  No longer needed
void checkDHWStatus() {

	switch (systemMode) {

	case STARTUP:        // No action
		turnRelay(DHW_PRIMARY_PUMP, OFF);
		startupPending &= ~_BV(DHW_PRIMARY_PUMP);          // Clear startup flag
		break;

	case RUNNING:

		turnRelay(DHW_PRIMARY_PUMP, 
			(!fault(DHW_TANK)          // Check nothing wrong
			&& (anyZoneDemand & ZONE_DHW)              // Check there is heat demand
			&& (tempC[DHW_TANK] < tempC[BOILER]))      // Check boiler is warmer than cylinder
			? ON : OFF);
		break;

	case QUIESING:
		turnRelay(DHW_PRIMARY_PUMP, ON);      // Dump heat to cylinder
		break;

	case ERROR:
		turnRelay(DHW_PRIMARY_PUMP, (tempC[BOILER] > BOILER_RUN_ON_TEMP) ? ON : OFF);
		break;
		
	case DORMANT:
		turnRelay(DHW_PRIMARY_PUMP, OFF);
		break;

	case MANUAL:
		turnRelay(DHW_PRIMARY_PUMP, (manualState & _BV(DHW_PRIMARY_PUMP)) ? ON : OFF);
		break;
	}

	// Set psuedo zone demand for DHW.  This can result in setSystemMode() changing mode from DORMANT to RUNNING
	if (onPeriodH && (tempC[DHW_TANK] + TOLERANCE) < dhwMaxTemp) anyZoneDemand |= ZONE_DHW;             // Call for heat
	else anyZoneDemand &= ~ZONE_DHW;            // Turn off call for heat  
}
*/

void controlRecirc() {
  
  switch (systemMode) {
    
    case STARTUP:
      turnRelay(DHW_RECIRC_PUMP, OFF);
      startupPending &= ~_BV(DHW_RECIRC_PUMP);
      break;

    case MANUAL:
      turnRelay(DHW_RECIRC_PUMP, (manualState & _BV(DHW_RECIRC_PUMP)) ? ON : OFF);
      break;
      
    default:         
      if (anyDHWDemand) {
        if (newDHWDemand) {              // New demand, start pump and countdown
          turnRelay(DHW_RECIRC_PUMP, ON);
          recircStart = heartBeatSecs;
          newDHWDemand = false;
        }
        else if ((heartBeatSecs - recircStart) > DHW_TIMEOUT) turnRelay(DHW_RECIRC_PUMP, OFF);
      }
      else turnRelay(DHW_RECIRC_PUMP, OFF);
  }
}

void initSensorD(byte sensorNum, byte pin) {
  
  oneWire[sensorNum].init(pin);
  delay(10);

  // Reset the bus and get the address of the first (& only) device to determine ROM family
  sensBuffer[ROM_CRC] = 0;
  oneWire[sensorNum].reset();
  oneWire[sensorNum].readROM(sensBuffer);
  
  if (oneWire[sensorNum].crc8(sensBuffer, ROM_CRC) != sensBuffer[ROM_CRC]) flagFault (sensorNum); 

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
      
      if (readPrecision(sensorNum) != TEMPERATURE_PRECISION) flagFault(sensorNum);      
      break;
    default:
      romFamily[sensorNum] = 0;              // Indicates fault
      flagFault(sensorNum); 
  }
}


byte readPrecision(byte sensorNum) {
  if (readScratchPad(sensorNum)) {
    switch (romFamily[sensorNum]) {
      case DS18S20:
        return 9;
      case DS18B20:
      case DS1822:
        switch (sensBuffer[CONFIGURATION]) {
          case TEMP_12_BIT:  return 12; 
          case TEMP_11_BIT:  return 11; 
          case TEMP_10_BIT:  return 10; 
          case TEMP_9_BIT:   return 9; 
          default:           flagFault(sensorNum);

        }
      default: 
        flagFault(sensorNum);
    } 
  }
  else flagFault(sensorNum);
  
  return 0;
}

float getTempD(byte sensorNum) {

  unsigned int tConv;
  
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
        case 12: tConv = T_CONV_12_BIT; break;
        case 11: tConv = T_CONV_11_BIT; break;
        case 10: tConv = T_CONV_10_BIT; break;
        case 9: 
        default: tConv = T_CONV_9_BIT; break;
      }
      break;
    default: 
      flagFault(sensorNum);
      return ERR_TEMP;
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
    return (float)(reading >> 2) * 0.25;
  }
  else {
    flagFault(sensorNum);
    return ERR_TEMP;
  }
}


boolean readScratchPad(byte sensorNum) {
  // Read the temperature
  oneWire[sensorNum].reset();
  oneWire[sensorNum].skip();
  oneWire[sensorNum].write(READSCRATCH);
  		
  for (int i = 0; i < 9; i++) sensBuffer[i] = oneWire[sensorNum].read();
  if (oneWire[sensorNum].crc8(sensBuffer, SCRATCHPAD_CRC) != sensBuffer[SCRATCHPAD_CRC]) {
    flagFault(sensorNum);
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

boolean relayOn(byte relay) {
	return relayState & _BV(relay);
}

void flagFault(byte sensor) {
  if (sensor != PRI_RETURN) somethingWrong |= _BV(sensor);
}

void clearFault(byte sensor) {
	somethingWrong &= ~_BV(sensor);
}

boolean fault(byte sensor) {
	return somethingWrong & _BV(sensor);
}

boolean fault() {
	return somethingWrong;
}

// ****** Refresh the status map, flagging those elements that have changed since last time *******
// Data then used by LCD and by Wire interface

void updateStatus() {
  
  #define BUF_ADD bufPosn += snprintf(statusMap[row] + bufPosn, NUM_COLS - bufPosn,
  char dhmText[15];

  // Take a copy of the old map and clear the new
  char oldStatus[NUM_ROWS][NUM_COLS+1];
 
  for (int i = 0; i < NUM_ROWS; i++) 
	  for (int j = 0; j < NUM_COLS + 1; j++) {
		  oldStatus[i][j] = statusMap[i][j];
		  statusMap[i][j] = ' ';
	  }
  
  // Build the new map  
  for (int row = 0; row < NUM_ROWS; row++) {
    byte bufPosn = 0;
    
    switch (row) { 
      case ROW_0:
        for (int i = 0; i < NUM_ZONES; i++) BUF_ADD "%c", (anyZoneDemand & _BV(i)) ? zoneTag[i] : zoneTag[i] + 32);   
        BUF_ADD "%c", ' ');
        for (int i = 0; i < NUM_DHW; i++) BUF_ADD "%c", (anyDHWDemand & _BV(i)) ? DHWTag[i] : DHWTag[i] + 32); 
 
		BUF_ADD " ");
		bufPosn += putRelayStatus(DHW_RECIRC_PUMP, row, bufPosn);

		if (relayOn(DHW_RECIRC_PUMP)) {
			bufPosn += snprintf(statusMap[row] + bufPosn, NUM_COLS - bufPosn, " %d", (DHW_TIMEOUT - (heartBeatSecs - recircStart)) / 60);
		}
		else bufPosn += snprintf(statusMap[row] + bufPosn, NUM_COLS - bufPosn, "%2c", ' ');
        break;

      case ROW_1:
        bufPosn += putRelayStatus(BURNER, row, bufPosn);
        bufPosn += putTemp(BOILER, row, bufPosn);
		BUF_ADD "%c", '/');
		bufPosn += putTemp(PRI_RETURN, row, bufPosn);

		BUF_ADD " ");
        bufPosn += putRelayStatus(DHW_PRIMARY_PUMP, row, bufPosn);
        bufPosn += putTemp(DHW_TANK, row, bufPosn);
        
		BUF_ADD " ");
        bufPosn += putRelayStatus(PRI_CIRCUIT_PUMP, row, bufPosn);

		BUF_ADD " %d.%d", centibars / 100, centibars % 100);
        break;
        
      case ROW_2:
        bufPosn += putRelayStatus(UFH_PUMP, row, bufPosn);
        bufPosn += putTemp(UFH_FLOW, row, bufPosn);
        BUF_ADD "%c", '/');
        bufPosn += putTemp(UFH_RETURN, row, bufPosn);
        
        BUF_ADD " "); 
        bufPosn += putRelayStatus(MIXER_OPEN, row, bufPosn);   
        bufPosn += putRelayStatus(MIXER_CLOSE, row, bufPosn); 
		
		BUF_ADD " %c", (anyZoneDemand & UFH_ZONES) ? timePatternTag[0] : timePatternTag[0] + 32);
		BUF_ADD "%c", (anyZoneDemand & ZONE_DHW) ? timePatternTag[1] : timePatternTag[1] + 32);
        break;

      case ROW_3: 
        dhmToText(timeNow, dhmText); 
        
        if (minsInMode > 59) {
          BUF_ADD "%s %c%d:%02d ", (timeNow) ? dhmText : "Pending", modeTag[systemMode], minsInMode / 60, minsInMode % 60); 
        }
        else BUF_ADD "%s %c%02d ", dhmText, modeTag[systemMode], minsInMode); 
        
        if (fault()) for (int i = 0; i < NUM_FAULT_FLAGS; i++) BUF_ADD "%c", (fault(i)) ? faultTag[i] : ' ');
        else BUF_ADD "OK"); 
        break;
    }
  }
  
  // Clear the status display change flags
  statusChanged.clearAll();

  // Compare old vs new and flag changes
  for (int i = 0; i < NUM_ROWS; i++) 
    for (int j = 0; j < NUM_COLS + 1; j++) 
      if (oldStatus[i][j] != statusMap[i][j]) statusChanged.put(i * j, true);    
}

unsigned int putRelayStatus(byte relayNum, byte row, byte bufPosn) {
  return snprintf(statusMap[row] + bufPosn, NUM_COLS - bufPosn, "%c", (relayOn(relayNum)) ? relayTag[relayNum] : relayTag[relayNum] + 32);  
}

unsigned int putTemp(byte sensorNum, byte row, byte bufPosn) {
  if (fault(sensorNum)) return snprintf(statusMap[row] + bufPosn, NUM_COLS - bufPosn, "%s", "**");
  else return snprintf(statusMap[row] + bufPosn, NUM_COLS - bufPosn, "%02d", (int)tempC[sensorNum]);
}


// ****** Display the status on the LCD, using flag bits to determine which chars to update *******
//
void displayStatus() {

  // Check each element in turn and update the display if changed
  for (int row = 0; row < NUM_ROWS; row++) 
    for (int col = 0; col < NUM_COLS; col++)                 // Miss out the NUM_COLS + 1 element of each row
      if (statusChanged.get(row * col)) {
        lcd.setCursor(col, row);
        lcd.print((statusMap[row][col] >= 20) ? statusMap[row][col] : ' ');        // Make sure only printable chars
      }
}


// ***** TWI input data *****
// Event handle established in Setup
//
// Messages processed as follows:
//	 Bm		- boiler pressure (centibars)
//   Dzf    - DHW demand for heat - zone z either on (f == '1') or off (f != '1')
//   Mmf    - manual mode - direct control.  On (f == '1') or Off (f != '1')
//   Of     - ON period (n == 1) or OFF period (n != 1) for DHW
//	 Puh	- current programmes for UFH and hot water
//   Ssn    - select type of data to send, s: D = DHW demand, R = Relay, S = Status (n = row), T = Temperature, Z = Zone demand
//   Zzf    - Zone demand for heat - zone z either on (f == '1') or off (f != '1')
//
void receiveData(int howMany) {

	if (Wire.available()) {
    
    char mode = Wire.read(); 
    char command = Wire.read(); 
    byte switchVal = 0;
    
    switch (mode) {
		case 'B':
			centibars = command;
			break;

      case 'D':              // DHW demand - 1 == yes, 0 == no
        switch (command) {
          case 'K':      switchVal = DHW_KITCHEN; break;
          case 'B':      switchVal = DHW_BATHROOM; break;
          case 'S':      switchVal = DHW_STUDY; break;
          case 'U':      switchVal = DHW_UPPER; break;
          case 'E':      switchVal = DHW_ENSUITE; break;
        }     
        (Wire.read() == '1') ? anyDHWDemand |= switchVal : anyDHWDemand &= ~switchVal;
        
        for (int i = 0; i < NUM_DHW; i++) if ((anyDHWDemand & _BV(i)) & ~(prevAnyDHWDemand & _BV(i))) newDHWDemand = true;
        prevAnyDHWDemand = anyDHWDemand;
        break;
        
      case 'M':                // Manual mode.  Exit either when cleared or after MANUAL_TIMEOUT_MINS
        switch (command) {
          case '0':      systemMode = DORMANT; return;                    // and exit
          case '1':      systemMode = MANUAL; minsInMode = 0; return;      // and exit
          case 'B':      switchVal = BURNER; break;
          case 'O':      switchVal = MIXER_OPEN; break;
          case 'C':      switchVal = MIXER_CLOSE; break;
          case 'D':      switchVal = DHW_PRIMARY_PUMP; break;
          case 'U':      switchVal = UFH_PUMP; break;
          case 'P':      switchVal = PRI_CIRCUIT_PUMP; break;
          case 'R':      switchVal = DHW_RECIRC_PUMP; break;
        }
        
        if (Wire.read() == '1') {
          // Make sure we only open the mixer in one direction
          if (switchVal == MIXER_OPEN) manualState &= ~_BV(MIXER_CLOSE);
          if (switchVal == MIXER_CLOSE) manualState &= ~_BV(MIXER_OPEN);
          
          // Set appropriate bit
          manualState |= _BV(switchVal);
        }
        else manualState &= ~_BV(switchVal);   
        
        systemMode = MANUAL;
        minsInMode = 0;

        break;

	  case 'O':
		  onPeriodH = (command == '1');
		  break;

		case 'P':
			timePatternTag[0] = command;
			timePatternTag[1] = Wire.read();
			break;
        
      case 'S':                               // Used by zone_control_basement to select data required
        statusSelect = command;
        displaySelect = Wire.read();
        break; 

      case 'T':                                // Time in dhm format
        timeNow = (unsigned int)command << 8 | Wire.read();
		timeReceived = TRUE;		// Keeps watchdog happy
                
        // Once a week, set DHW temp higher to counter Legionella
        dhwMaxTemp = (dhmGet(timeNow, VAL_DAY) == 2) ? DHW_HIGH_TEMP : DHW_NORM_TEMP;    // Monday is day 2 (can be any day, but avoid weekends when more water consumption)
     
        break;
        
      case 'Z':              // Zone heat demand - yes/no
		switch (command) {
			case 'K': switchVal = ZONE_KITCHEN; break;
			case 'G': switchVal = ZONE_GT_HALL; break;
			case 'B': switchVal = ZONE_BASEMENT; break;
			case 'D': switchVal = ZONE_DINING; break;
			case 'S': switchVal = ZONE_STUDY; break;
			default:  switchVal = 0; break;
		}
		(Wire.read() == '1') ? anyZoneDemand |= switchVal : anyZoneDemand &= ~switchVal;

        break;
    }    
  }
}

// ******** TWI output data - report status to Wire interface *********
// Called to get response to 'Ssn' request, where statusSelect set to 's' and displaySelect set to 'n' in receiveData
// Event handle established in Setup
//

void reportStatus() {

	const byte BUFLEN = 128;        // Align to UDP_TX_PACKET_MAX_SIZE in EthernetUdp.h and BUFFER_LENGTH in Wire.h
	char buffer[BUFLEN];
	byte bufPosn = 0;
	#define BUF_ADD bufPosn += snprintf(buffer + bufPosn, BUFLEN - bufPosn,

	byte limit;

	switch (statusSelect) {
		case 'D':
			limit = NUM_DHW - 1;

			BUF_ADD "{\"DA\":\"%c%c\",", meInit, statusSelect);
			for (int tag = 0; tag < (NUM_DHW); tag++) {
				BUF_ADD "\"%c\":\"%c\",", DHWTag[tag], (anyDHWDemand & _BV(tag)) ? DHWTag[tag] : DHWTag[tag] + 32);
			}
			BUF_ADD "\"T\":\"%d\"}\0", relayOn(DHW_RECIRC_PUMP) ? (DHW_TIMEOUT - (heartBeatSecs - recircStart)) / 60 : 0);
			break;

		case 'M':
			BUF_ADD "{\"DA\":\"%c%c\",\"%c\":\"%c\"}\0", meInit, statusSelect, statusSelect, modeTag[systemMode]);
			break;

		case 'R':
			limit = NUM_RELAYS - 1;

			BUF_ADD "{\"DA\":\"%c%c\",", meInit, statusSelect);
			for (int tag = 0; tag < (NUM_RELAYS); tag++) {
				BUF_ADD "\"%c\":\"%c\"", relayTag[tag], (relayOn(tag)) ? relayTag[tag] : relayTag[tag] + 32);
				if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
			}
			break;

		case 'S':
			for (int i = 0; i < NUM_COLS; i++) buffer[i] = statusMap[displaySelect][i];
			bufPosn = NUM_COLS + 1;
			break;

		case 'T':
			limit = NUM_SENSORS - 1;

			BUF_ADD "{\"DA\":\"%c%c\",", meInit, statusSelect);
			for (int tag = 0; tag < (NUM_SENSORS); tag++) {
				if (fault(tag)) BUF_ADD "\"%c\":\"**\"", sensorTag[tag]);
				else BUF_ADD "\"%c\":\"%02d\"", sensorTag[tag], (int)tempC[tag]);
				if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
			}
			break;

		case 'Z':
			limit = NUM_ZONES - 1;

			BUF_ADD "{\"DA\":\"%c%c\",", meInit, statusSelect);
			for (int tag = 0; tag < (NUM_ZONES); tag++) {
				BUF_ADD "\"%c\":\"%c\"", zoneTag[tag], (anyZoneDemand & _BV(tag)) ? zoneTag[tag] : zoneTag[tag] + 32);
				if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
			}
			break;

		default:					// Error condition - no/bad data received on previous receiveData
			BUF_ADD "{\"DA\":\"%cX\",\"%c\":\"Bad request\"}\0", meInit, statusSelect);
			break;
	}

	// Reply with data
	Wire.write((byte*)buffer, bufPosn + 1);			// Not sure why need the extra char, but seems to be needed
}