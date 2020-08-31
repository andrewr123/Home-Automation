/* Zone control for Basement zones

Version history
---------------

Apr 13 - v1 - baseline version
Feb 15 - v2 - incorporate time pattern handler from boiler control
            - add bi-directional UDP comms to pass back status info
Apr 15 - v3 - add time response
Dec 16 - v4 - revised timing of Allday and Background
			- internal generation of timeNow
			- independent schedule for hot water
			- pressure sensor input
Jan 17 - Dining controller changed to ignore onPeriod signal - independent timing
Jan 18 - MAX_MSGS in acceptInput doubled to avoid missing messages
Jul 20 - Watchdog timout implemented.  ICMPPING references removed to allow compatibility with new Ethernet library. Reduced volume of syslog events


Current functionality 
- limited to passing UDP messages onto boiler and sending boiler status to Syslog
- interrogates water pressure sensor

UDP commands: 
- "Xm"   - where m is '1' to switch on reflection of messages, '0' to turn off
- "Fn"   - force heating ON (1) or OFF (!1)
- "Ss"   - request status
- "Tnn"  - current time, where nn is unsigned int holding dhm-formatted day-hour-time (from Study)

All other UDP messages passed to boiler unchanged

To do:

- implement time test and notify boiler
- adjust to standard .h files
- frost protection mode

*/

#include <avr/wdt.h>
#include <SPI.h>
#include "Ethernet.h"
#define UDP_TX_PACKET_MAX_SIZE 128			// Override the default 24
#include <EthernetUdp.h>
// #include <ICMPPing.h>  // Not needed for Basement and uses classes no longer available in Ethernet library

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

// ICMPPing ping;

char buffer[UDP_TX_PACKET_MAX_SIZE];        // Max allowed by libraries, set in EthernetUdp.h 

// ****** Identity of this controller ******

#define meIP basementIP                     
#define meMac basementMac
char meName[]									= "Basement";
const static char meInit			= 'B';

// ******  Time ******

// Current time is updated every 30 secs

unsigned int timeNow = 0;

// ***************** Zones ***************
/*
const byte ZONE_KITCHEN = 0;
const byte ZONE_GT_HALL = 1;
const byte ZONE_BASEMENT = 2;
const byte ZONE_DINING = 3;
const byte ZONE_STUDY = 4;
const byte ZONE_DHW = 5;
const char zoneTag[] = { 'K', 'G', 'B', 'D', 'S', 'H' };
const byte NUM_ZONES = 6;
*/

// Time patterns define whether UFH is ON or OFF (DHW has only one pattern at present

const byte NUM_INDEP_ZONES			  = 2;			// UFH (generic) and ZONE_DWH
const byte ZONE_UFH = 0;							// UFH zones
const byte ZONE_DHW = 1;							// Domestic Hot Water

const byte NUM_TIME_PATTERNS          = 3;          // Number of different time patterns permitted
const byte ALLDAY = 0;								// 6am - 11pm
const byte BACKGROUND = 1;						// Start and end of day
const byte TURNOFF = 2;	 							// Turn off
const char timePatternTag[] = { 'A', 'B', 'X' };
byte timePatternU = BACKGROUND;						// Initial settings
byte timePatternH = ALLDAY;

const byte MAX_TIME_PERIODS           = 5;           // Number of ON periods allowed per day per time pattern

// Values set in setupHA()
unsigned int timePeriodStart[NUM_INDEP_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];        // Start of time period
unsigned int timePeriodEnd[NUM_INDEP_ZONES][NUM_TIME_PATTERNS][MAX_TIME_PERIODS];          // End of time period
byte numTimePeriods[NUM_INDEP_ZONES][NUM_TIME_PATTERNS];

// Flags calculated with reference to current time and time patterns determine if heating is ON or OFF

boolean onPeriodU = false;                           // True if UFH zones scheduled to be on (now only for S & B)
boolean onPeriodH = false;							 // True if ZONE_DHW scheduled to be on
boolean forceOn = false;                             // True if OFF period overriden (by message).  Reset with next ON period

// ******   Internal timing - nothing to do with real time   *******

unsigned long prevTime;
unsigned int heartBeatSecs                 = 0;                
const unsigned int HEARTBEAT_FREQ_MS       = 200;		// 5 heartbeats per second
const unsigned int CHECK_INPUT_FREQ        = 2;			// Check for messages to pass on to boiler every 400mS
const unsigned int GET_STATUS_FREQ         = 50;		// Get status from boiler every 10 secs
const unsigned int CHECK_MANIFOLD_FREQ     = 5;			// Check if basement needs heat and alert boiler
const unsigned int CHECK_PRESSURE_FREQ     = 35;		// Read pressure sensor
const unsigned int CHECK_TIME_FREQ		   = 30;		// Work out current time every 30 secs and act on it

//  ****** Pin assignments ************

const static byte BASEMENT_SENSOR          = 11;		// High if Basement manifold wants heat
const static byte PRESSURE_SENSOR		   = A0;		// To read 4-20mA signal from pressure sensor using 250ohm shunt (1-5v)

// ****** Manifold sensors ******

byte basementManifold                      = OFF;

// ****** Pressure sensor - based on WIKA A-10 sensor 0-2.5 bar reading using 4-20mA signal across 250ohm series resistor (in practice, is 240ohm) *****

const static unsigned int ZERO_BAR_TICKS = 1023 / 5;	// Analog ports return 1023 for 5v
const static unsigned int DYNAMIC_RANGE_TICKS = 1023 - ZERO_BAR_TICKS;
const static unsigned int DYNAMIC_RANGE_CENTIBAR = 250;			// 2.5 bar
byte centibars = 0;

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

		// Calculate current time and pass it on
		if (heartBeatSecs % CHECK_TIME_FREQ == 0) checkTime();

    // See if basement is demanding heat
    if (heartBeatSecs % CHECK_MANIFOLD_FREQ == 0) checkBasement();
 
    // Get status from boiler and report to syslog
    if (heartBeatSecs % GET_STATUS_FREQ == 0) reportStatus();
    
    // Read the primary circuit pressure
    if (heartBeatSecs % CHECK_PRESSURE_FREQ == 0) checkPressure();

  }
  
  delay(50);
}

void setupHA() {

  int myMCUSR;    // Startup reason
    
  // Load timePeriod patterns for UFH - Day 0 == Everyday, Day 10 == Sat/Sun
  // Pattern ALLDAY 
  timePeriodStart[ZONE_UFH][ALLDAY][0] = dhmMake(0, 6, 0);    // 06:00
  timePeriodEnd[ZONE_UFH][ALLDAY][0] = dhmMake(0, 23, 00);    // 23:00
  numTimePeriods[ZONE_UFH][ALLDAY] = 1;
  
  // Pattern BACKGROUND 
  timePeriodStart[ZONE_UFH][BACKGROUND][0] = dhmMake(0, 6, 0);
  timePeriodEnd[ZONE_UFH][BACKGROUND][0] = dhmMake(0, 9, 0);
  timePeriodStart[ZONE_UFH][BACKGROUND][1] = dhmMake(0, 18, 0);
  timePeriodEnd[ZONE_UFH][BACKGROUND][1] = dhmMake(0, 23, 00);
  timePeriodStart[ZONE_UFH][BACKGROUND][2] = dhmMake(10, 6, 0);	
  timePeriodEnd[ZONE_UFH][BACKGROUND][2] = dhmMake(10, 23, 00);
  numTimePeriods[ZONE_UFH][BACKGROUND] = 3;

  // UFH TURNOFF
  numTimePeriods[ZONE_UFH][TURNOFF] = 0;
  
  // Load patterns for ZONE_DHW
  // DHW ALLDAY
  timePeriodStart[ZONE_DHW][ALLDAY][0] = dhmMake(0, 6, 0);
  timePeriodEnd[ZONE_DHW][ALLDAY][0] = dhmMake(0, 23, 0);
  numTimePeriods[ZONE_DHW][ALLDAY] = 1;

  // DHW BACKGROUND 
  timePeriodStart[ZONE_DHW][BACKGROUND][0] = dhmMake(0, 6, 0);
  timePeriodEnd[ZONE_DHW][BACKGROUND][0] = dhmMake(0, 9, 0);
  timePeriodStart[ZONE_DHW][BACKGROUND][1] = dhmMake(0, 12, 0);
  timePeriodEnd[ZONE_DHW][BACKGROUND][1] = dhmMake(0, 13, 0);
  timePeriodStart[ZONE_DHW][BACKGROUND][2] = dhmMake(0, 18, 0);
  timePeriodEnd[ZONE_DHW][BACKGROUND][2] = dhmMake(0, 23, 00);
  numTimePeriods[ZONE_DHW][BACKGROUND] = 3;

  // DHW TURNOFF
  numTimePeriods[ZONE_DHW][TURNOFF] = 0;

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
  
  // Initialise basement demand sensor & pressure sensor input
  analogReference(DEFAULT);			// 5v max
  pinMode(BASEMENT_SENSOR, INPUT);			// In practice not connected - noise on line made it unreliable
  pinMode(PRESSURE_SENSOR, INPUT);

  // Start the i2c bus
  Wire.begin();

  // Capture relative time
  prevTime = millis();
}

/* ***** acceptInput from network *****

Valid inputs:

Fn    Force heating ON (n == '1') or OFF (n != '1').  Auto cancels at start of next programmed ON period
Pzp   Set time pattern for zone type, z == U or H, p == A, B or X
Ss    Status request; some answered directly, most answered by boiler controller.  All answers in JSON.  Valid values for 's' request:
      - B = bar pressure in primary circuit
	    - D = DHW demand
      - P = current timePattern (A, B etc) - answered here
      - R = Relay settings
      - T = Temperature
      - Z = Zone demand
Ttt   Time as an unsigned integer (high and low) in dhm format - no longer used, but Study still sends so make noop
Xn    Switch ON or OFF syslog reporting

Others passed directly to Boiler Control

Dzf    - DHW demand for heat - zone z either on (f == '1') or off (f != '1')
Mmf    - manual mode - direct control.  On (f == '1') or Off (f != '1')
Zzf    - Zone demand for heat - zone z either on (f == '1') or off (f != '1')

*/

void acceptInput() {
	SAVE_CONTEXT("aInp")
  
	const byte MAX_MSGS = 10;						  // Max msgs to process before exiting while loop; avoids continual loop
	byte numMsgs = 0;
	unsigned int bufPosn, dataLen;							
	char timeText[30];
	char zone;
	#define BUF_ADD bufPosn += snprintf(buffer + bufPosn, UDP_TX_PACKET_MAX_SIZE - bufPosn,      // Take care; buffer used for input & output
	
	while (dataLen = ard.get(buffer, UDP_TX_PACKET_MAX_SIZE)) {        // If data available - multiple messages potentially
	
		/* Jul 20 - omit for now
		if (sendToSyslog) {
			if (dataLen <= UDP_TX_PACKET_MAX_SIZE) buffer[dataLen] = 0x00;
			SENDLOGM('N', buffer);
		}
		*/

		if (numMsgs++ > MAX_MSGS) {
			SENDLOGM('W', "Max msgs exceeded");
			return;
		}

		bufPosn = 0;										// Position to start of buffer for output & overwriting redundant input

		switch (buffer[0]) {

			case 'F':								// Force heating on/off  
				forceOn = (buffer[1] == '1');		// Gets picked up at next time refresh
				break;

			case 'P':        // Set timePattern for UFH & DHW - gets sent to boiler controller at next time refresh
				zone = buffer[1]; 
				
				switch (buffer[2]) {
					case 'A':    (zone == 'U') ? timePatternU = ALLDAY : timePatternH = ALLDAY; break;
					case 'B':    (zone == 'U') ? timePatternU = BACKGROUND : timePatternH = BACKGROUND; break;
					case 'X':	 (zone == 'U') ? timePatternU = TURNOFF : timePatternH = TURNOFF; break;
					default:     SENDLOGM('W', "Invalid timePattern");
				}
				break;

			case 'S':        // External status request.  Two answered directly, the rest by boiler controller.  

				switch (buffer[1]) {
					case 'B':
						BUF_ADD "{\"DA\":\"%c%c\",\"%c\":\"%d.%d\"}\0", meInit, 'B', 'B', centibars / 100, centibars % 100);
						break;

					case 'P':            // Current timePattern & time - handled locally
						dhmToText(timeNow, timeText);
						BUF_ADD "{\"DA\":\"%c%c\",\"%c\":\"%c%c\"", meInit, 'P', 'P', (forceOn) ? 'F' : (onPeriodU) ? timePatternTag[timePatternU] : timePatternTag[timePatternU] + 32, (onPeriodH) ? timePatternTag[timePatternH] : timePatternTag[timePatternH] + 32);
						BUF_ADD ",\"T\":\"%s\"}\0", timeText);
						break;

					default:            // All others handled by boiler controller
						Wire.beginTransmission(2);   // transmit to device #2
						for (int i = 0; i < 2; i++) Wire.write(buffer[i]);
						Wire.endTransmission();      // stop transmitting

						// Get response from controller        
						if (bufPosn = Wire.requestFrom(2, UDP_TX_PACKET_MAX_SIZE)) {
							for (int i = 0; i < bufPosn; i++) buffer[i] = Wire.read();
						};

						bufPosn = strchr(buffer, '\0') - buffer;		// Shouldn't be needed, but Wire always seems to return full buffer length

						break;
				}

				// Reply to requestor
				ard.reply(REPLY_PORT, buffer, bufPosn);

				// Copy to syslog
				if (sendToSyslog) SENDLOGM('N', buffer);

				break;

			case 'T':			// Ignore redundant timestamp from Study
				break;

			case 'X':
				sendToSyslog = (buffer[1] == '1');
				break;

			default:
				// Send anything else onto the boiler controller
				Wire.beginTransmission(2);  
				for (int i = 0; i < 3; i++) Wire.write(buffer[i]);
				Wire.endTransmission();     
		}
	}

	RESTORE_CONTEXT
}

void checkTime() {		// Get current time, pass it to boiler, and determine if anything needs to happen
	SAVE_CONTEXT("CheckT")

	// Get current time & pack into dhm format
	timeNow = dhmMake(weekday(), hour(), minute());

	// Send time to boiler controller
	buffer[0] = 'T';
	buffer[1] = timeNow >> 8;
	buffer[2] = timeNow & 0x00ff;
	buffer[3] = 0x00;

	Wire.beginTransmission(2); 
	for (int i = 0; i < 3; i++) Wire.write(buffer[i]);
	Wire.endTransmission();  

	// . . . and to syslog
	/* Jul 20 - omit for now 
	if (sendToSyslog) {
		char timeText[30];
		int bufEnd = snprintf(timeText, 30, "T %04X ", timeNow);
		dhmToText(timeNow, timeText + bufEnd);
		SENDLOGM('N', timeText);
	}
	*/

	// Test if UFH zones are on or off; may change value of forecOn
	onPeriodU = isOn(ZONE_UFH, timePatternU, onPeriodU);		
													
	// Send to other arudinos (which then send zone demand if required)
	buffer[0] = 'O';
	buffer[1] = (onPeriodU) ? '1' : '0';
	buffer[2] = 0x00;

	ard.put(studyIP, UdpArdPort, buffer, 3);
	ard.put(diningIP, UdpArdPort, buffer, 3);  // Remove - this is redundant

	// Send to syslog
	// if (sendToSyslog) SENDLOGM('N', buffer);

	// Convenient point to tell boiler what time pattern to display (& copy to syslog)
	buffer[0] = 'P';
	buffer[1] = (forceOn) ? 'F' : timePatternTag[timePatternU];
	buffer[2] = timePatternTag[timePatternH];
	buffer[3] = 0x00;

	Wire.beginTransmission(2);
	for (int i = 0; i < 3; i++) Wire.write(buffer[i]);
	Wire.endTransmission();

	// if (sendToSyslog) SENDLOGM('N', buffer);
	
	// Test if DHW is scheduled to be on or off & tell boiler (which then determines if DHW 'zone' has demand)
	onPeriodH = isOn(ZONE_DHW, timePatternH, onPeriodH);			// Used directly to turn on/off DHW

	buffer[0] = 'O';
	buffer[1] = onPeriodH ? '1' : '0';					
	buffer[2] = 0x00;

	Wire.beginTransmission(2);  
	for (int i = 0; i < 2; i++) Wire.write(buffer[i]);
	Wire.endTransmission();  

	// if (sendToSyslog) SENDLOGM('N', buffer);

	RESTORE_CONTEXT
}

boolean isOn(byte zone, byte programme, boolean currentStatus) {  // Check if zone is scheduled to be on or not

	boolean newStatus = false;

	// Loop through on/off periods for zone and programme to find match
	for (int i = 0; i < numTimePeriods[zone][programme]; i++) {
		if (newStatus = dhmBetween(timeNow, timePeriodStart[zone][programme][i], timePeriodEnd[zone][programme][i])) break;    // Exit on match
	}

	// Set/clear flags
	if (zone == ZONE_UFH && !currentStatus && newStatus) forceOn = false;  // Clear force flag if moving from OFF to ON
	
	return forceOn || newStatus;
}

void checkBasement() {      // See if basement wants more heat - tell boiler
  // Basement status based on input to motorised valve
	SAVE_CONTEXT("cBas")

  char message[] = "ZB ";
  // basementManifold = digitalRead(BASEMENT_SENSOR);   // Jul 20 - Remove for the moment
	basementManifold = 0;																	// Jul 20 - temporary
  message[2] = (basementManifold && onPeriodU) ? '1' : '0';      // On or off

  // Send it onto the boiler controller
  Wire.beginTransmission(2);   
  for (int i = 0; i < 3; i++) Wire.write(message[i]);
  Wire.endTransmission();     

	RESTORE_CONTEXT
}


void reportStatus() {


	SAVE_CONTEXT("rStat")

	int msgLen = 0;
  
  for (int i = 0; i < 4; i++) {
    // Get latest status
    Wire.beginTransmission(2);
    Wire.write('S');                // Set displayRow in Boiler_control
    Wire.write('S');
    Wire.write(i);
    Wire.endTransmission();
    
	if (msgLen = Wire.requestFrom(2, UDP_TX_PACKET_MAX_SIZE)) {
		for (int i = 0; i < msgLen; i++) buffer[i] = Wire.read();
	}

	buffer[20] = 0x00;				// Mark end of data
    
    if (sendToSyslog) SENDLOGM('N', buffer);
  }

	RESTORE_CONTEXT
}

void checkPressure() {					// Read primary circuit pressure & display
	SAVE_CONTEXT("cPres")

	// Get ticks - 0-1023; 
	unsigned int ticks = analogRead(PRESSURE_SENSOR);

	// Do the maths to get pressure
	centibars = ((float)(ticks - ZERO_BAR_TICKS) / (float)DYNAMIC_RANGE_TICKS) * DYNAMIC_RANGE_CENTIBAR;

    // Construct message
    buffer[0] = 'B';
		buffer[1] = centibars;
    buffer[2] = 0x00;  
    
    // Send message to boiler controller
    Wire.beginTransmission(2);  
    for (int i = 0; i < 2; i++) Wire.write(buffer[i]);
    Wire.endTransmission();     
    
    // Alert syslog
		/*
		if (sendToSyslog) {
				buffer[1] = centibars / 100;
				buffer[2] = '.';
				buffer[3] = centibars % 100;
				buffer[4] = 0x00;

				SENDLOGM('N', buffer);
		}
		*/

	RESTORE_CONTEXT
}
