
/*
BOILER CONTROLLER
-----------------

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
Nov 20 - v7 - adopt interrupt-driven usage of I2C comms, alternating master/slave relationship to provide duplex comms
              (equivalent changes in Basement controller)
            - re-engineer to split request/read steps in sensor processing to reduce delays - uses 'wakeup'
Jan 21 - v8 - move to Optiboot loader & implement watchdog timer
            - move temperature sensor code to HA_temperature library

To do: provide 5v supply to sensors through dedicated pins, allowing hard reset


Uses TWI/I2C interface to communicate with the outside world.  Avoids the need for an Ethernet shield on what is a crowded board

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

Uno pinout - see https://www.circuito.io/blog/arduino-uno-pinout/
https://components101.com/microcontrollers/arduino-uno
https://www.arduino.cc/en/reference/board

AtMega chip limits: https://electronics.stackexchange.com/questions/67092/how-much-current-can-i-draw-from-the-arduinos-pins

Digital pins
------------

  D0 - D1 - reserved for use by USB/serial - effectively off limits

Temperature sensors

  D2 - Primary flow
  D3 - UFH flow
  D4 - UFH return
  D5 - Boiler
  D6 - Primary return
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
  - burner 
  - mixer open
  - mixer close
  - dhw primary pump - NOT NEEDED WITH NEW BOILER
  - ufh pump
  - primary pump
  - recirc pump
  
Analog pins
-----------

A0 - spare
A1 - spare
A2 - spare
A3 - spare
A4 & A5 - TWI

*/

#define TWI_BUFFER_SIZE 96		// Per override in AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.3\libraries\Wire\src\wire.h & \utilities\twi.
#define BOILER_CONTROLLER

#include <arduino.h>
#include <avr/wdt.h>

#include "HA_globals.h"

#include <LiquidCrystal.h>
#include "Wakeup.h"
#include "TimerOne.h"            // NB: modified version of public TimerOne library
#include <OneWire.h>
#include <Wire.h>
#include "Bitstring.h"

#include "HA_temperature.h"



// ***** Identity - on JSON strings *****

const static char meInit = 'B';

//  ****** Wire/TWI bus for comms with Basement controller ******

volatile boolean incomingI2C;
volatile int incomingSize;
#define I2C_ADDR_BOILER 2
#define I2C_ADDR_BASEMENT 1

// *************  Temperature sensors - Dallas DS18B20  ***********

const static byte TEMP_SENSOR_PIN[NUM_TEMP_SENSORS] = { 2, 3, 4, 5, 6 };  // DIO pins - Dallas sensors
const static byte TEMP_SENSOR_POWER_PIN = A0;												      // DIO pin - 5v power to Dallas temp sensors

HA_temperature tSensor[NUM_TEMP_SENSORS];                       // Sensor objects

const char tSensorTag[] = { 'B', 'U', 'u', 'D', 'T' };
const static byte PRI_FLOW          = 0;
const static byte UFH_FLOW          = 1;      
const static byte UFH_RETURN        = 2;      
const static byte BOILER            = 3; 				  
const static byte PRI_RETURN        = 4;

const static byte TARGET_TEMP_PRECISION = 10;                   // Equates to 0.25C - good enough

// ****** Control flags ******

byte somethingWrong                 = 0;            // If BV == 1, then display flag (and shutdown if critical)
const static byte NUM_FAULT_FLAGS   = NUM_TEMP_SENSORS;
const char faultTag[NUM_FAULT_FLAGS] = { 'P', 'U', 'u', 'B', 'p' };

// *****************  On/off relays  ********************

const static byte NUM_RELAYS = 8; 
const static byte BURNER             = 0;
const static byte MIXER_OPEN         = 1;
const static byte MIXER_CLOSE        = 2;
//const static byte DHW_PRIMARY_PUMP   = 3;           // Not used
const static byte UFH_PUMP           = 4;
const static byte PRI_CIRCUIT_PUMP   = 5;
const static byte DHW_RECIRC_PUMP    = 6;
//const static byte INTERNET           = 7;          // Not used
const char relayTag[NUM_RELAYS]     = { 'B', 'O', 'C', 'D', 'U', 'P', 'R', 'I'};
byte startupPending = 0;            // Flags cleared as each relay goes through startup
  
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
const char zoneTag[]                 = {'K', 'G', 'B', 'D', 'S'};
const static byte NUM_ZONES          = 5;

// **************  UFH *************

const static byte STABLE                   = 0;
const static byte OPENING                  = 1;
const static byte CLOSING                  = 2;
//const char mixerTag[]                      = {'S', 'T', 't', 'F', 'f', 'Z'};
//const static byte NUM_MIXERS               = 6;

const static int UFH_INCREMENT             = 15;           // Added to current return temperature to give target flow temperature
const static int UFH_FLOW_MAX_TEMP         = 50;           // Degrees C at which to turn off mixer

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
boolean onPeriodH						               = FALSE;			// Used to flag that DHW is ON or OFF

// ******* Boiler pressure ******

byte centibars								= 0;			// Boiler primary circuit water pressure
const static byte MIN_PRESSURE = 10;		// 0.1 bar

// ******   Timing   *******
//
unsigned long prevTime;
unsigned int heartBeatSecs                   = 0;          
unsigned int minsInMode                      = 0;
unsigned int minsSinceComms = 0;
const unsigned int HEARTBEAT_FREQ_MS         = 1000;     // Heartbeat every second
const unsigned int TEMP_CHECK_FREQ           = 11;       // Check temp sensors 
const unsigned int UFH_TEMP_CHECK_FREQ       = 3;        // Read mixer temps
const unsigned int LCD_RESTART_FREQ          = 32;       // Restart to overcome noise upsetting LCD - to be solved, hopefully, with better hardware design
const unsigned int DHW_DEMAND_CHECK_FREQ     = 5;        // Check if DHW demand
const unsigned int ROUTER_RESTART_CHECK_FREQ = 10;
const unsigned int ONE_MINUTE                = 60;
const unsigned int MANUAL_TIMEOUT_MINS       = 10;        // Minutes allowed in MANUAL; then set to RUNNING
const byte CHECK_FREQ[NUM_TEMP_SENSORS] = { UFH_TEMP_CHECK_FREQ };


// ****************  LCD display  ***************************

const static byte ROW_0 = 0;          // 1st row for temperature
const static byte ROW_1 = 1;
const static byte ROW_2 = 2;
const static byte ROW_3 = 3;

LiquidCrystal lcd(10, 8, 9);             // Modified version of libary using 595 shift registers.  Data, clock, latch  


// ********** Status map - used for output to LCD and to Wire **********

const static byte NUM_COLS                   = 20;          // In practice, only 19 cols seem to be usable on the display
const static byte NUM_ROWS                   = 4;
char statusMap[NUM_ROWS][NUM_COLS + 1];
BITSTRING statusChanged(NUM_ROWS * NUM_COLS);        // BV == 1 indicates status has changed since last displayed

// ******  Time - updated every 30 secs in dhm format

unsigned int timeNow = 0;							 // Set by HA_Time functions.  Current time in dhm format; used for display only

// ***** System mode *****

enum eSystemMode {
    STARTUP,
    ERROR,					          // Comms lost - shutdown
    RUNNING,                  // Normal mode
    QUIESING,                 // Heading to dormant, but boiler temp too high so cooling with pumps
    DORMANT,                  // Waiting for demand signal
    MANUAL                    // Manual override
};

eSystemMode systemMode;
const char modeTag[] = { 'S', 'E', 'R', 'Q', 'D', 'M' };


// PRE-BOOT CODE
// -------------

// **** Optiboot code to preserve internal registers to determine restart reason ****
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


void setup(void) {
    
    // Reset watchdog

    MCUSR = 0;
    wdt_disable();

    // Initialise wakeup to support background daemons or temperature reading
    wakeup.init();

    // Startup actions
    setupComms();
    logStartupReason();
    setupRelays();
    setupTempSensors();
    setupLCD();

    // Go into startup mode
    systemMode = STARTUP;
    startupPending = _BV(BURNER) | _BV(MIXER_OPEN)| _BV(UFH_PUMP) | _BV(PRI_CIRCUIT_PUMP) | _BV(DHW_RECIRC_PUMP);
    //startupPending &= ~_BV(DHW_PRIMARY_PUMP);          // Clear startup flag - unused

    // Start watchdog with 8 sec timeout
    wdt_enable(WDTO_8S);

    // Start the relative clock
    prevTime = millis();
}

void loop(void) { 

    // Reset watchdog - if fault then this won't happen and watchdog will fire, restarting the sketch
    wdt_reset();

    // Increment heartbeat every second 
    if ((millis() - prevTime) >= HEARTBEAT_FREQ_MS) {   
    
        prevTime = prevTime + HEARTBEAT_FREQ_MS;                  // Avoid normal = millis() to allow catchup
        heartBeatSecs++;                                          // Eventual overflow @ 64k not material

        if (heartBeatSecs % ONE_MINUTE == 0) minsInMode++;

        if (heartBeatSecs % DHW_DEMAND_CHECK_FREQ == 0) controlRecirc();       // If DHW demand then turn on recirc pump & start countdown timer
    
        if (heartBeatSecs % UFH_TEMP_CHECK_FREQ == 0) controlMixer();          // Check status of mixer and adjust to hit target output temperature
        
        if (heartBeatSecs % TEMP_CHECK_FREQ == 0) {                         // Main loop - relatively infrequent as temps don't change too rapidly  
      
            controlUFHPump();                       // Check underfloor heating demand and set pump accordingly

            controlPriPump();                       // Check primary heating demand and set pump accordingly

            controlBurner();                        // Set boiler on/off dependent on state and pump demand
        }
              
        updateStatus();

        displayStatus();
    
        if (heartBeatSecs % LCD_RESTART_FREQ == 0) {
            lcd.begin(NUM_COLS, NUM_ROWS);                
            statusChanged.setAll();                              // Causes a refresh in displayStatus
        }
    
        setSystemMode();
    }

    delay(10);
    wakeup.runAnyPending();

    if (incomingI2C) processIncomingI2C();            // Set by ISR processOnReceive
}

void setupComms() {

    // Start up Two Wire Interface, with this as slave device #2 (alternates with master later)
    Wire.begin(I2C_ADDR_BOILER);
    incomingI2C = false;                         // True when Basement controller sends data
    Wire.onReceive(processOnReceive);             // Sets incomingI2C TRUE if data sent by Basement controller
}

void logStartupReason() {

    // Log reason for startup by testing resetFlag made available on startup - see Optiboot refs above
    // Zone controllers can use SYSLOG, but not available for this controller, so use error reporting instead

    int myMCUSR;    // Startup reason

    // Order of tests is significant; last succesful test is what's reported
    if (resetFlag & (1 << WDRF)) myMCUSR = WDRF;				// Watchdog reset
    if (resetFlag & (1 << EXTRF)) myMCUSR = EXTRF;			// Manual reset
    if (resetFlag & (1 << PORF)) myMCUSR = PORF;				// Power-on reset

    switch (myMCUSR) {
    case WDRF:
        logError(0x01);
        break;
    case EXTRF:
        logError(0x02);
        break;
    case PORF:
        logError(0x03);
        break;
    }
}

void setupRelays() {

    // Setup relay pins/driver
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
}

void setupTempSensors() {

    // Provide power to Dallas sensors
    pinMode(TEMP_SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(TEMP_SENSOR_POWER_PIN, LOW);
    delay(10);
    digitalWrite(TEMP_SENSOR_POWER_PIN, HIGH);
    delay(10);

    // Initialise sensors, get initial read and start regular background refresh - access latest temperature using getTempC()
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        if (!tSensor[i].init(TEMP_SENSOR_PIN[i], TARGET_TEMP_PRECISION, CHECK_FREQ[i])) {
            logError(0xB0 | (i & 0x0F));          // 1st nibble == 'B', 2nd nibble == sensor number response
        }
    }
    somethingWrong = 0;
}

void setupLCD() {

    // Start display
    lcd.begin(NUM_COLS, NUM_ROWS);
    lcd.print("Starting");

    // Initial display
    updateStatus();

    displayStatus();
}

// ***** TWI ISR *****

void processOnReceive(int length) {
    // ISR to flag receipt of data from Basement controller - handle established in Setup
    // Real work then done by processIncomingI2C
    incomingI2C = true;           // Set flag
    incomingSize = length;         // Checked against data received
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

void controlBurner() {

  switch (systemMode) {                         // systemMode set by demand.  Fault flags may override pump action and thus burner action
    case STARTUP:
      // Make sure boiler is off
      turnRelay(BURNER, OFF);
      startupPending &= ~_BV(BURNER);          // Clear startup flag
      break;

    case RUNNING: 
        if (relayOn(PRI_CIRCUIT_PUMP) || relayOn(UFH_PUMP)) turnRelay(BURNER, ON);
        else turnRelay(BURNER, OFF);            // Only arises if a fault prevents pumps running
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
  
  switch (systemMode) {                                  // systemMode set by demand.  Fault flags may override pump action
    
    case STARTUP:
      turnRelay(PRI_CIRCUIT_PUMP, OFF);
      startupPending &= ~_BV(PRI_CIRCUIT_PUMP);          // Clear startup flag
      break; 
     
    case RUNNING:
      // Shutdown pump if error with Primary sensors - not critical, so ignore for now
//      if (fault(PRI_FLOW) || fault(PRI_RETURN)) {
//          turnRelay(PRI_CIRCUIT_PUMP, OFF);
//          return;
//     }

      // If demand for heat from non-UFH zones then turn on pump 
      if ((anyZoneDemand & (ZONE_DINING | ZONE_STUDY))) turnRelay(PRI_CIRCUIT_PUMP, ON);
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
  
  switch (systemMode) {                         // systemMode set by demand.  Fault flags may override pump action
        
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
      
      // If demand for heat from UFH zones then turn on pump
      if (anyZoneDemand & (ZONE_GT_HALL | ZONE_KITCHEN | ZONE_BASEMENT)) turnRelay(UFH_PUMP, ON);
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
        targetMixerTemp = min(UFH_FLOW_MAX_TEMP, (int)tSensor[UFH_RETURN].getTempC() + UFH_INCREMENT);
      
        switch (mixerState) {
          case STABLE:
            if (timeSinceLast >= CHILL_TIME) {
              // If above target then reduce
              if (tSensor[UFH_FLOW].getTempC() >= (targetMixerTemp + UPPER_THRESHOLD)) {
                turnRelay(MIXER_OPEN, OFF);
                turnRelay(MIXER_CLOSE, ON);
                mixerState = CLOSING;
                lastChangeOfState = heartBeatSecs;
                actuationTime = min(0.4 * (tSensor[UFH_FLOW].getTempC() - (float)(targetMixerTemp)) + 1, MAX_ACTUATION_TIME);
              }
              
              // If below target then increase  
              if (tSensor[UFH_FLOW].getTempC() < (targetMixerTemp - LOWER_THRESHOLD)) {
                turnRelay(MIXER_CLOSE, OFF);
                turnRelay(MIXER_OPEN, ON);
                mixerState = OPENING;
                lastChangeOfState = heartBeatSecs;
                actuationTime = min(0.4 * ((float)(targetMixerTemp) - tSensor[UFH_FLOW].getTempC()) + 1, MAX_ACTUATION_TIME);
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


// Relay control
// -------------

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
  somethingWrong |= _BV(sensor);
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
        BUF_ADD " ");
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
        BUF_ADD " ");

        bufPosn += putRelayStatus(PRI_CIRCUIT_PUMP, row, bufPosn);
        bufPosn += putTemp(PRI_FLOW, row, bufPosn);
		    BUF_ADD "/");
		    bufPosn += putTemp(PRI_RETURN, row, bufPosn);
        BUF_ADD " ");
        
        bufPosn += putRelayStatus(UFH_PUMP, row, bufPosn);
        bufPosn += putTemp(UFH_FLOW, row, bufPosn);
        BUF_ADD "/");
        bufPosn += putTemp(UFH_RETURN, row, bufPosn);
        break;
        
      case ROW_2:
        BUF_ADD "%d.%d bar ", centibars / 100, centibars % 100);
        
        bufPosn += putRelayStatus(MIXER_OPEN, row, bufPosn);   
        bufPosn += putRelayStatus(MIXER_CLOSE, row, bufPosn); 

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
  else return snprintf(statusMap[row] + bufPosn, NUM_COLS - bufPosn, "%02d", (int)tSensor[sensorNum].getTempC());
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

void processIncomingI2C() {

/*  Process incoming I2C messages, and respond with data if needed

    Messages processed as follows:
	      Bm		  - boiler pressure (centibars) - info
        Dzf    - DHW demand for heat - zone z either on (f == '1') or off (f != '1') - action needed
        Mmf    - manual mode - direct control.  On (f == '1') or Off (f != '1') - action needed
        Ssn    - status request.  s: D = DHW demand, R = Relay, S = Status (n = row), T = Temperature, Z = Zone demand - response needed
        T      - time - info
        Zzf    - Zone demand for heat - zone z either on (f == '1') or off (f != '1') - action needed

    All messages trigger a response to Basement controller, either "OK" or real data
*/

  incomingI2C = false;           // Reset flag
  int _incomingSize;
  char errBadBoilerReq[] PROGMEM = "Bad boiler request: ";

	if (_incomingSize = Wire.available()) {
    
    char mode = Wire.read(); 
    char command = Wire.read();
    char subCommand = Wire.read();      // Redundant for mode == 'B'
    byte switchVal = 0;
    char OK[] = "OK";
    
    switch (mode) {
		    case 'B':
			      centibars = command;
            basementI2C(OK, 2);
			      break;

        case 'D':              // DHW demand - 1 == yes, 0 == no
            switch (command) {
                case 'K':      switchVal = DHW_KITCHEN; break;
                case 'B':      switchVal = DHW_BATHROOM; break;
                case 'S':      switchVal = DHW_STUDY; break;
                case 'U':      switchVal = DHW_UPPER; break;
                case 'E':      switchVal = DHW_ENSUITE; break;
            }     
            (subCommand == '1') ? anyDHWDemand |= switchVal : anyDHWDemand &= ~switchVal;
        
            for (int i = 0; i < NUM_DHW; i++) if ((anyDHWDemand & _BV(i)) & ~(prevAnyDHWDemand & _BV(i))) newDHWDemand = true;
            prevAnyDHWDemand = anyDHWDemand;
            basementI2C(OK, 2);
            break;
        
        case 'M':                // Manual mode.  Exit either when cleared or after MANUAL_TIMEOUT_MINS
            switch (command) {
                case '0':      systemMode = DORMANT; return;                    // and exit
                case '1':      systemMode = MANUAL; minsInMode = 0; return;      // and exit
                case 'B':      switchVal = BURNER; break;
                case 'O':      switchVal = MIXER_OPEN; break;
                case 'C':      switchVal = MIXER_CLOSE; break;
                //case 'D':      switchVal = DHW_PRIMARY_PUMP; break;
                case 'U':      switchVal = UFH_PUMP; break;
                case 'P':      switchVal = PRI_CIRCUIT_PUMP; break;
                case 'R':      switchVal = DHW_RECIRC_PUMP; break;
            }
        
            if (subCommand == '1') {
                // Make sure we only open the mixer in one direction
                if (switchVal == MIXER_OPEN) manualState &= ~_BV(MIXER_CLOSE);
                if (switchVal == MIXER_CLOSE) manualState &= ~_BV(MIXER_OPEN);
          
                // Set appropriate bit
                manualState |= _BV(switchVal);
            }
            else manualState &= ~_BV(switchVal);   
        
            systemMode = MANUAL;
            minsInMode = 0;
            basementI2C(OK, 2);
            break;
        
        case 'S':                               // Used by zone_control_basement to select data required
            processStatusRequest(command, (int)(subCommand - 48));           // Convert from ASCII to int
            break; 

        case 'T': {                               // Time in dhm format
            char timeBuffer[16];
            timeNow = (unsigned int)command << 8 | subCommand;
            dhmToText(timeNow, timeBuffer);
            basementI2C(OK, 2);
            break;
        }
        
        case 'Z':              // Zone heat demand - yes/no
		        switch (command) {
			          case 'K': switchVal = ZONE_KITCHEN; break;
			          case 'G': switchVal = ZONE_GT_HALL; break;
			          case 'B': switchVal = ZONE_BASEMENT; break;
			          case 'D': switchVal = ZONE_DINING; break;
			          case 'S': switchVal = ZONE_STUDY; break;
			          default:  switchVal = 0; break;
		        }
		        (subCommand == '1') ? anyZoneDemand |= switchVal : anyZoneDemand &= ~switchVal;

            basementI2C(OK, 2);
            break;

        default: {
            const unsigned int BUFLEN = 32;
            char localBuf[BUFLEN];
            snprintf(localBuf, BUFLEN, "Bad boiler request: %c%c%c", mode, command, subCommand);
            basementI2C(localBuf, BUFLEN);
        }
    }    
  }
}

void processStatusRequest(char statusSelect, int displaySelect) {

  // Process response to status requests ('Ssn')
    
	char buffer[TWI_BUFFER_SIZE];
	int bufPosn = 0;

	#define BUF_ADD bufPosn += snprintf(buffer + bufPosn, TWI_BUFFER_SIZE - bufPosn,

	byte limit;
  
	switch (statusSelect) {
		case 'D':                   // DHW demand for recirc
			limit = NUM_DHW - 1;

			BUF_ADD "{\"DA\":\"%c%c\",", meInit, statusSelect);
			for (int tag = 0; tag < (NUM_DHW); tag++) {
				BUF_ADD "\"%c\":\"%c\",", DHWTag[tag], (anyDHWDemand & _BV(tag)) ? DHWTag[tag] : DHWTag[tag] + 32);
			}
			BUF_ADD "\"T\":\"%d\"}\0", relayOn(DHW_RECIRC_PUMP) ? (DHW_TIMEOUT - (heartBeatSecs - recircStart)) / 60 : 0);
			break;

    case 'E':                   // Report any errors to Basement controller
        listErrors(buffer, TWI_BUFFER_SIZE);
        break;
  
		case 'R':                   // Relay/pump status
			limit = NUM_RELAYS - 1;

			BUF_ADD "{\"DA\":\"%c%c\",", meInit, statusSelect);
			for (int tag = 0; tag < (NUM_RELAYS); tag++) {
				BUF_ADD "\"%c\":\"%c\"", relayTag[tag], (relayOn(tag)) ? relayTag[tag] : relayTag[tag] + 32);
				if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
			}
			break;

		case 'S':                   // Status display, line 'displaySelect'
			for (int i = 0; i < NUM_COLS; i++) buffer[i] = statusMap[displaySelect][i];
			bufPosn = NUM_COLS + 1;
			break;

		case 'T':                   // Current temperatures
			limit = NUM_TEMP_SENSORS - 1;

			BUF_ADD "{\"DA\":\"%c%c\",", meInit, statusSelect);
			for (int tag = 0; tag < (NUM_TEMP_SENSORS); tag++) {
        BUF_ADD "\"%c\":\"", tSensorTag[tag]);
				if (fault(tag)) BUF_ADD "**\"");
				else BUF_ADD "%02d\"", (int)tSensor[tag].getTempC());
				if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
			}
			break;

		case 'Z':                   // Target temperatures
			limit = NUM_ZONES - 1;

			BUF_ADD "{\"DA\":\"%c%c\",", meInit, statusSelect);
			for (int tag = 0; tag < (NUM_ZONES); tag++) {
				BUF_ADD "\"%c\":\"%c\"", zoneTag[tag], (anyZoneDemand & _BV(tag)) ? zoneTag[tag] : zoneTag[tag] + 32);
				if (tag == limit) BUF_ADD "}\0"); else BUF_ADD ",");
			}
			break;

		default:					// Error condition - bad data received on previous receiveData
			BUF_ADD "{\"DA\":\"%cX\",\"%c%x\":\"Bad request\"}\0", meInit, statusSelect, statusSelect);
			break;
	}

  basementI2C(buffer, bufPosn);  
}

unsigned int basementI2C(char* sendBuffer, int sendBuflen) {     
    
    unsigned int response;

    // Handle comms with basement over I2C

    // Setup I2C bus as Master
    Wire.begin();

    // Send the data
    Wire.beginTransmission(I2C_ADDR_BASEMENT);
    Wire.write(sendBuffer, sendBuflen);
    response = Wire.endTransmission();
    if (response != 0) logError(0xB0 | (response & 0x0F));          // 1st nibble == 'B', 2nd nibble == Wire.endTransmission response 

    // Swap this controller back to be slave
    Wire.begin(I2C_ADDR_BOILER);

    return response;
}
