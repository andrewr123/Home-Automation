/* 
ZONE CONTROLLER FOR BASEMENT
----------------------------

Version history
---------------

Apr 13	- v1 - baseline version
Feb 15	- v2 - incorporate time pattern handler from boiler control
            - add bi-directional UDP comms to pass back status info
Apr 15	- v3 - add time response
Dec 16	- v4 - revised timing of Allday and Background
				- internal generation of timeNow
				- independent schedule for hot water
				- pressure sensor input
Jan 17	- Dining controller changed to ignore onPeriod signal - independent timing
Jan 18	- MAX_MSGS in acceptInput doubled to avoid missing messages
Jul 20	- Watchdog timout implemented.  ICMPPING references removed to allow compatibility with new Ethernet library. Reduced volume of syslog events
Nov 20	- sendToSyslog Y/N replaced with syslogLevel
				- DWH zone removed (now done by boiler)
				- Basement zone removed (restore when reliable)
				- TWI interface	with boiler controller re-engineered

				TO DO:
				- infrequent pressure reading


Current functionality 
- limited to passing UDP messages onto boiler and sending boiler status to Syslog
- interrogates water pressure sensor

UDP commands: 
- "Xs"   - where s is one of XACEWNID to indicate severity of logs to be logged
- "Ss"   - request status
- "Tnn"  - current time, where nn is unsigned int holding dhm-formatted day-hour-time

All other UDP messages passed to boiler unchanged

*/


#include <SD.h>
#define UDP_TX_PACKET_MAX_SIZE 128			// Per override in OneDrive\Documents\Arduino\Libraries\Ethernet\EthernetUDP.h
#define TWI_BUFFER_SIZE 128							// Per override in AppData\Local\Arduino15\packages\MegaCore\hardware\avr\2.0.3\libraries\Wire\src\utility\twi.h

#include <avr/wdt.h>
#include <SPI.h>
#include "Ethernet.h"
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
	
// SPECIFIC ELEMENTS - FOR THIS CONTROLLER
// ---------------------------------------

// ****** Identity of this controller ******

#define meIP basementIP                     
#define meMac basementMac
char meName[] = "Basement";
const static char meInit = 'B';

/* ****** Pressure sensor - based on WIKA A-10 sensor 0-2.5 bar reading using 4-20mA signal across 250ohm series resistor *****
		20mA x 250ohm = 5v drop across resistor
		 4mA x 250ohm = 1v drop

		12v --->  --|
								|
						 sensor
								|
								|
								|----> Analog input via 250ohm resistor (for safety)
								|
								\
								/		250ohm resistor (in practice 225 + 22.5 = 247.5)
								\
		0v --->  -- |
*/


const static byte PRESSURE_SENSOR = A0;		// To read 4-20mA signal from pressure sensor using 250ohm shunt (1-5v)

const static unsigned int ZERO_BAR_TICKS = 1023 / 5;										// 4mA = 20% of full scale.  Analog ports return 1023 for 5v
const static unsigned int DYNAMIC_RANGE_TICKS = 1023 - ZERO_BAR_TICKS;	// Deduct to set zero
const static unsigned int DYNAMIC_RANGE_CENTIBAR = 250;			// 2.5 bar
byte centibars = 0;

// GENERIC ELEMENTS - COMMON TO ALL CONTROLLERS
// --------------------------------------------

// ***** External comms *****

char syslogLevel = 'W';				
const char ALLOWABLE_SYSLOG_LEVELS[] = "XACEWNID";

// ***** Wire *****

volatile boolean incomingI2C;
volatile int incomingSize;
#define I2C_ADDR_BOILER 2
#define I2C_ADDR_BASEMENT 1
 
// ******  Time ******
// Current time is updated every 30 secs ...

unsigned int timeNow = 0;

// Time patterns not used in this controller

// ******   Internal (relative) timing  *******

unsigned long prevTime;
unsigned int heartBeatSecs									= 0;                
const unsigned int HEARTBEAT_FREQ_MS				= 200;		// 5 heartbeats per second
const unsigned int CHECK_INPUT_FREQ					= 2;			// Check for messages to pass on to boiler every 400mS
const unsigned int GET_STATUS_FREQ					= 50;			// Get status from boiler every 10 secs
const unsigned int CHECK_ERRORS_FREQ				= 25;			// See if boiler reporting any error - 5 secs
const unsigned int CHECK_MANIFOLD_FREQ			= 25;			// Check if basement needs heat and alert boiler
const unsigned int CHECK_PRESSURE_FREQ			= 50;			// Read pressure sensor every 10 secs
const unsigned int CHECK_TIME_FREQ					= 150;		// Work out current time every 30 secs and act on it

// **** Code to preserve internal registers to determine restart reason ****
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

	// Startup actions

	setupComms();
	logStartupReason();
	setupBoilerSupport();

  // Start watchdog
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

		// Increment heartbeat every 200 mS
		if ((millis() - prevTime) >= HEARTBEAT_FREQ_MS) {   

				prevTime = millis();                  
				heartBeatSecs++;                                          

				// See if any commands from console or errors from boiler
				if (heartBeatSecs % CHECK_INPUT_FREQ == 0) processIncomingUDP();

				// Calculate current time and pass it on
				if (heartBeatSecs % CHECK_TIME_FREQ == 0) checkTime();
 
				// Get status from boiler and report to syslog
				if (heartBeatSecs % GET_STATUS_FREQ == 0) reportStatus();
		
				// Check for any errors and report to syslog
				if (heartBeatSecs % CHECK_ERRORS_FREQ == 0) checkErrors();

				// Send dummy signal re Basement to prevent timeout from boiler controller
				if (heartBeatSecs % CHECK_MANIFOLD_FREQ == 0) checkManifold();
    
				// Read the primary circuit pressure
				if (heartBeatSecs % CHECK_PRESSURE_FREQ == 0) checkPressure();

				// Report any errors from this controller (boiler errors reported by checkErrors())
				listErrors(buffer, BUFLEN);
				if (strstr(buffer, "OK") == NULL) SENDLOGM('W', buffer);
		}
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

		// Use NTP server to initialise time
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

void setupBoilerSupport () {

		// Initialise pressure sensor input
		analogReference(DEFAULT);			// 5v max
		pinMode(PRESSURE_SENSOR, INPUT);
}

void processIncomingUDP() {

		/* ***** process incoming from network *****

		Valid inputs:

		Rn		Reset request.  Valid values for 'n':
						- 8 = boiler
						- 9 = this controller
		Ss    Status request; most answered by boiler controller.  All answers in JSON.  Valid values for 's' request:
						- B = bar pressure in primary circuit (answered here)
						- M = current mode & time
						- R = Relay settings
						- S = Screen readout
						- T = Temperature
						- Z = Zone demand
		Xn    Set syslog reporting level

		Others passed directly to Boiler Control

		Dzf    - DHW demand for heat - zone z either on (f == '1') or off (f != '1')
		Mmf    - manual mode - direct control.  On (f == '1') or Off (f != '1')
		Zzf    - Zone demand for heat - zone z either on (f == '1') or off (f != '1')

		*/

		SAVE_CONTEXT("aInp")
  
		const byte MAX_MSGS = 10;						  // Max msgs to process before exiting while loop; avoids continual loop
		byte numMsgs = 0;
		unsigned int dataLen;							
		char timeText[30];
		char zone;
		int wireStatus;

		byte bufPosn;
		char recvBuffer[UDP_TX_PACKET_MAX_SIZE];
		char sendBuffer[TWI_BUFFER_SIZE];

		#define BUF_ADD bufPosn += snprintf(sendBuffer + bufPosn, TWI_BUFFER_SIZE - bufPosn,    
	
		while (dataLen = ard.get(recvBuffer, UDP_TX_PACKET_MAX_SIZE)) {        // If data available - multiple messages potentially
	
				if (dataLen <= UDP_TX_PACKET_MAX_SIZE) recvBuffer[dataLen] = 0x00;
		
				if (numMsgs++ > MAX_MSGS) {
					SENDLOG('W', "Max msgs exceeded - dropped: ", recvBuffer);
					RESTORE_CONTEXT
					return;
				}

				bufPosn = 0;														// Position to start of buffer for output & overwriting redundant input

				switch (recvBuffer[0]) {
						case 'R':														// Reset boiler or this controller
								switch (recvBuffer[1]) {
										case '8':										// Boiler - response should be "BR", gets trapped and reported as a NACK
												boilerI2C(recvBuffer, 2, NULL, NULL);	
												break;
										case '9':										// This controller
												while (1);							// Infinite loop - triggers watchdog to reset this controller
												break;
								}
								break;

						case 'S': {													// External status request.  Two answered directly, the rest by boiler controller. 
								switch (recvBuffer[1]) {				
										case 'B':										// System pressure - bars
												BUF_ADD "{\"DA\":\"%cB\",\"B\":\"%d.%d\"}\0", meInit, centibars / 100, centibars % 100);
												break;

										case 'X':										// Syslog notification level
												BUF_ADD "{\"DA\":\"%cX\", \"X\":\"%c\"}\0", meInit, syslogLevel);
												break;

										default:										// All others handled by boiler controller
												bufPosn = boilerI2C(recvBuffer, 2, sendBuffer, TWI_BUFFER_SIZE);
												break;
								}

								// Reply to requestor
								ard.reply(REPLY_PORT, sendBuffer, bufPosn);

								break;
						}

						case 'X':														// Set SYSLOG level
								if (strchr(ALLOWABLE_SYSLOG_LEVELS, recvBuffer[1])) {
										syslogLevel = recvBuffer[1];
										syslog.adjustLevel(syslogLevel);
								}
								else SENDLOG('W', "Invalid syslog level: ", recvBuffer[1]);
						break;

						default:														// Send anything else onto the boiler controller
								boilerI2C(recvBuffer, 3, NULL, NULL);
				}
		}

		RESTORE_CONTEXT
}

void checkTime() {		// Get current time and pass it to boiler
		SAVE_CONTEXT("CheckT")

	const byte SEND_BUFFER_SIZE = 32;
	const byte RECV_BUFFER_SIZE = 32;
	char sendBuffer[SEND_BUFFER_SIZE];
	char recvBuffer[RECV_BUFFER_SIZE];

	// Get current time & pack into dhm format
	timeNow = dhmMake(weekday(), hour(), minute());

	sendBuffer[0] = 'T';
	sendBuffer[1] = timeNow >> 8;
	sendBuffer[2] = timeNow & 0xff;

	// Send time to boiler controller
	boilerI2C(sendBuffer, 3, NULL, NULL);

	// . . . and to syslog 
	int bufEnd = snprintf(sendBuffer, SEND_BUFFER_SIZE, "T %04X ", timeNow);
	dhmToText(timeNow, sendBuffer + bufEnd);
	SENDLOGM('D', sendBuffer);

	RESTORE_CONTEXT
}

void reportStatus() {

	SAVE_CONTEXT("rStat")

	int msgLen = 0;
	int wireStatus;
	char recvBuffer[32];
	char sendBuffer[4] = "SSx";
  
  for (int i = 0; i < 4; i++) {
		// Get latest status
		sendBuffer[2] = (char)i + 48;									// Select row (displaySelect in Boiler Control) - convert to ASCII to assist tracing
		boilerI2C(sendBuffer, 3, recvBuffer, TWI_BUFFER_SIZE);

		recvBuffer[20] = 0x00;				// Force end of data mark

		SENDLOGM('I', recvBuffer);								// Reflection of boiler LCD
  }

	RESTORE_CONTEXT
}

void checkErrors() {

		SAVE_CONTEXT("cErr")

		char sendBuffer[] = "SE";
		int response;
		const byte BUFLEN = 64;
		char recvBuffer[BUFLEN];
		memset(recvBuffer, 0x00, BUFLEN);
		
		response = boilerI2C(sendBuffer, 2, recvBuffer, BUFLEN);

		if (strstr(recvBuffer, "OK") == NULL) {									// Boiler is reporting an error
				if (strstr(recvBuffer, "1:02") != NULL) {
						SENDLOGM('W', "Boiler restarted");
				}
				else {
						SENDLOG('W', "Boiler errs: ", recvBuffer);
				}
		}

		RESTORE_CONTEXT
}

void checkManifold() {					// Send dummy message to boiler controller to demonstrate still alive
		SAVE_CONTEXT("cManif")

		char buffer[] = "ZB0";			

		boilerI2C(buffer, 4, NULL, NULL);

		RESTORE_CONTEXT
}

void checkPressure() {					// Read primary circuit pressure & display
	SAVE_CONTEXT("cPres")

	char buffer[8];
	int wireStatus;
	
	// Get ticks - 0-1023; 
	unsigned int ticks = analogRead(PRESSURE_SENSOR);

	// Do the maths to get pressure
	centibars = ((float)(ticks - ZERO_BAR_TICKS) / (float)DYNAMIC_RANGE_TICKS) * DYNAMIC_RANGE_CENTIBAR;

	// Construct message
	buffer[0] = 'B';
	buffer[1] = centibars;
	buffer[2] = 0x00;  
    
	// Send message to boiler controller
	boilerI2C(buffer, 3, NULL, NULL);

	RESTORE_CONTEXT
}

unsigned int boilerI2C(char* sendBuffer, unsigned int sendBuflen, char* recvBuffer, unsigned int recvBuflen) {     
		
		/*
		Bidirectional comms with boiler over I2C

		Send message then check reply, returning size of data received.  
		
		Calling routine sets recvBuffer to indicate nature of reply expected:
		- NULL recvBuffer: ACK ("OK") expected and ignored.  If NACK then reported to SYSLOG as error
		- valid pointer to recvBuffer: data stream expected and returned to calling routine

		*/ 
		
		SAVE_CONTEXT("I2C")
		SENDLOG('D', "Tx: ", sendBuffer)

		unsigned int dataAvailable, wireStatus, pollCount;
		const unsigned int POLLING_DELAY = 20;					// Time to wait for response
		const unsigned int POLL_LIMIT = 100;					// Max iterations == 2 seconds
		const unsigned int REPORT_THRESHOLD = 4;
		const unsigned int BUFLEN = 32;
		char localBuf[BUFLEN];
		
		// Setup I2C bus - set as Master
		Wire.begin();

		// Send the data
		Wire.beginTransmission(I2C_ADDR_BOILER);
		Wire.write(sendBuffer, sendBuflen);
		wireStatus = Wire.endTransmission();
		if (wireStatus > 0) SENDLOG('W', "Tx NACK: ", wireStatus);

		// Swap this controller to be slave and establish onReceive ISR
		Wire.begin(I2C_ADDR_BASEMENT);
		incomingI2C = false;
		Wire.onReceive(processOnReceive);						// Sets incomingI2C == true when boiler controller responds

		// Loop until data received/timeout
		pollCount = 0;
		while (pollCount++ < POLL_LIMIT && !incomingI2C) delay(POLLING_DELAY);
		if (pollCount > REPORT_THRESHOLD) {
				SENDLOG('W', "Rx ms: ", pollCount * 20);
				SENDLOG('W', "Msg = ", sendBuffer);
		}

		// Read the data - often not interested, but need a positive acknowledgement from boiler
		if (dataAvailable = Wire.available()) {
				if (recvBuffer == NULL) {												// No data expected by calling routine, but we check handshake here
						dataAvailable = constrain(dataAvailable, 0, BUFLEN);

						for (int i = 0; i < dataAvailable; i++) localBuf[i] = Wire.read();
						localBuf[dataAvailable] = 0x00;

						if (strstr(localBuf, "OK") == NULL) {
								SENDLOG('W', "NACK:", localBuf);
						}
				}
				else {											// Substantive response expected - typically Status request or potentially Error report
						if (dataAvailable > recvBuflen) dataAvailable = recvBuflen;

						for (int i = 0; i < dataAvailable; i++) recvBuffer[i] = Wire.read();

						recvBuffer[dataAvailable] = 0x00;
						SENDLOG('D', "Rx: ", recvBuffer)
				}
		}
		else {
				snprintf(localBuf, BUFLEN, "No Rx to %s", sendBuffer);
				SENDLOGM('W', localBuf);
		}

		RESTORE_CONTEXT

		return dataAvailable;
}

// ***** TWI input ISR *****

void processOnReceive(int length) {
		// ISR to flag receipt of data from Boiler controller
		// Real work then done by boilerI2C
		incomingI2C = true;           // Set flag
		incomingSize = length;         // Checked against data received
}