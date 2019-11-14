#include "TrackDAQ.h"

#include "Arduino.h"
#include "DMAChannel.h"
#include "ADC.h"
#include "FreqMeasureMulti.h"
#ifdef EM7180_LIB
#include "EM7180_Master.h"
#endif
#include "i2c_t3.h"
#include "TinyGPS++.h"
#include "TimeLib.h"
#include "SdFat.h"
#include <WS2812Serial.h>

const float _VERSION = 1.70;

#if defined(USB_MTPDISK)
#include "MTP.h"
#endif
#include "BikeData.h"



/*  The PINS list includes pins that I can use for future data logging
 *  Future pins were tested and confirmed to work but not needed for current project
 PIN      Signal Type     Purpose           PCB Location
 ===      ==============  ================  ================
 0       Serial1 RX      GPS In            Internal
 1       Serial1 TX      GPS In            Internal
 3       CAN TX          Future            N/A
 4       CAN RX          Future            N/A
 5       Freq            F Wheel RPM       DB25 - 5
 6       Freq            R Wheel RPM       DB25 - 3
 7       Serial3 RX      ------------      N/A
 8       LED/Digial      Enable LED        DB9 - 8
 9       Serial2 RX      Data Out          DB9 - 7
 10      Serial2 TX      Data Out          DB9 - 2
 15/A1   Analog          Future            N/A
 16/A2   Analog          Brake Pressure    DB25 - 8
 18      I2C             SDA IMU           Internal
 19      I2C             SCL IMU           Internal
 21      Freq            Engine RPM        DB25 - 7
 22      Freq            Future            N/A
 23      Freq            Future            N/A
 24      Digital         F Brake Switch    DB25 - 6
 25      Digital         Neutral           DB25 - 4
 26      Digital         Future            N/A
 27      Digital         Future            N/A
 28      Digital         Future            N/A
 29      Enable/Digital  Enable Logging    DB9 - 3
 33/A14  Analog          Rear Suspension   DB25 - 22
 34/A15  Analog          Gear Indicator    DB25 - 9
 35/A16  Analog          Front Suspension  DB25 - 23
 36/A17  Analog          TPS               DB25 - 24
 37/A18  Analog          Future            N/A
 38/A19  Analog          O2 Sensor         DB25 - 21
 */

enum outputTypeSet {
	NONE, GPS, GPS_RC3, RC3_ONLY, BINARY
};
static const enum outputTypeSet outputType = GPS_RC3;
static const bool displayData = true;
static const bool fileLogGPS = false;
static const bool fileLogVBO = true;
static const bool fileLogCSV = true;
static const bool fileLogBIN = false;
bool fileLogRPM = false;

static const bool enableFrontWheel = false;
static const bool enableRearWheel = false;
static const bool enableNeutral = false;

const int numled = 1;
byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED
WS2812Serial leds(numled, displayMemory, drawingMemory, ENABLE_LED, WS2812_GRB);
int currentRGBLED = 0;


#ifdef EM7180_LIB
static const uint8_t MAG_RATE = 100;  // Hz
static const uint16_t ACCEL_RATE = 200;  // Hz
static const uint16_t GYRO_RATE = 200;  // Hz
static const uint8_t BARO_RATE = 50;   // Hz
static const uint8_t Q_RATE_DIVISOR = 1;    // 1/3 gyro rate
EM7180_Master em7180 = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);
#endif

#if defined(USB_MTPDISK)
MTPStorage_SD storage;
MTPD          mtpd(&storage);
#endif

#if !defined(USB_MTPDISK)
SdFatSdioEX SD;
#endif
File fileGPS;
File fileBIN;
File fileVBO;
File fileCSV;
File fileRPM;

ADC *adc = new ADC(); // adc object

DMAChannel* dma0 = new DMAChannel(false);
DMAChannel* dma1 = new DMAChannel(false);
DMAChannel* dma2 = new DMAChannel(false);
DMAChannel* dma3 = new DMAChannel(false);

FreqMeasureMulti freq0;   // Engine RPM
FreqMeasureMulti freq1;   // Front Wheel RPM
FreqMeasureMulti freq2;   // Rear Wheel RPM
bool engineRPMSet = false;
bool frontRPMSet = false;
bool rearRPMSet = false;
unsigned long currentRPMTime;
unsigned long lastRPMTime = millis();
long rpmCounter = 0;


// NOTE: number of DMA must be multiple of 2
//ChannelsCfg order must be {CH1, CH2, CH3, CH0 }, adcbuffer output will be CH0, CH1, CH2, CH3
//Order must be {Second . . . . . . . . First} no matter the number of channels used.
// forum post about DMA ADC reading : https://forum.pjrc.com/threads/35101-Using-two-ADCs-to-sample-8-signals
// hex comes from table on page 274 of K66 manual (0x40 = ADC0, 0x41 = ADC1) + ID of channel page 948 (ie A14 = ADC0_SE17 = 10001 = 0x11)

const uint16_t ChannelsCfg_0[] = { 0x51, 0x52, 0x4E, 0x48 }; //ADC0: CH1 A14 [F Sus]  CH2 A15[Gear]   CH3 A1 [Spare 1]    CH0 A2 [Brake]
const uint16_t ChannelsCfg_1[] = { 0x44, 0x47, 0x46, 0x45 }; //ADC1: CH1 A16 [R Sus]  CH2 A19[O2]     CH3 A18[Spare 2]    CH0 A17[TPS]

DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE + 0))) adcbuffer_0[BUF_SIZE];
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE + 0))) adcbuffer_1[BUF_SIZE];

TinyGPSPlus gps;

TrackDAQ::BikeData bike = TrackDAQ::BikeData();
unsigned long currentFlash;
unsigned long lastFlash = millis();
long loopCounter = 0;

static const unsigned long IMU_INTERVAL = 2; // in ms.  5 = 200hz  10 = 100hz
unsigned long currentIMUTime;
unsigned long lastIMUTime = millis();
int itemsIMUPerSecond = 0;

static const unsigned long BIN_INTERVAL = 10; // in ms.  5 = 200hz  10 = 100hz
unsigned long currentBINSendTime;
unsigned long lastBINSendTime = millis();

static const unsigned long RC3_INTERVAL = 20; // in ms.  5 = 200hz  10 = 100hz   20 = 50Hz
unsigned long currentRC3SendTime;
unsigned long lastRC3SendTime = millis();
char rc3Buffer[200];
char vboBuffer[500];
char csvBuffer[500];
int rc3Counter = 0;

static const unsigned long VBO_INTERVAL = 10; // in ms.  5 = 200hz  10 = 100hz
unsigned long currentVBOSendTime;
unsigned long lastVBOSendTime = millis();

static const unsigned long CSV_INTERVAL = 10; // in ms.  5 = 200hz  10 = 100hz
unsigned long currentCSVSendTime;
unsigned long lastCSVSendTime = millis();

static const unsigned long DISPLAY_INTERVAL = 1000; // in ms.  1/s
unsigned long currentDisplayTime;
unsigned long lastDisplayTime = millis();

static int itemsGPSPerSecond = 0;
static int itemsBINPerSecond = 0;
static int itemsRC3PerSecond = 0;
static int itemsVBOPerSecond = 0;
static int itemsCSVPerSecond = 0;

bool rtcSet = false;
int rtcCount = 0;
bool loggingEnabled = false;

char gpsLine[100];
int gpsCount = 0;

void setup() {
	Serial.begin(115200);
	Serial1.begin(115200);
	Serial2.begin(115200);

	Serial.println("Setup - Start");
	if( RGB_LED ) {
		leds.begin();
		setLEDColour( LED_BLUE );
	}
	else if( !RGB_LED ){
		pinMode( ENABLE_LED, OUTPUT);
		digitalWrite( ENABLE_LED, HIGH);
	}

	pinMode( INTERNAL_LED, OUTPUT);
	digitalWrite( INTERNAL_LED, HIGH);

	if (displayData)
		delay(10000);
	else
		delay(5000);

	Serial.println("Setup - Clear buffers");
	memset(gpsLine, 0, sizeof(gpsLine));

	// clear buffer
	for (int i = 0; i < BUF_SIZE; ++i) {
		adcbuffer_0[i] = 50000;
		adcbuffer_1[i] = 50000;

	}

	Serial.println("Setup - Frequency");
	pinMode( RPM_ENGINE_PIN, INPUT_PULLUP);
	pinMode( RPM_FWHEEL_PIN, INPUT_PULLUP);
	pinMode( RPM_RWHEEL_PIN, INPUT_PULLUP);
	freq0.begin( RPM_ENGINE_PIN);
	if( enableFrontWheel )
		freq1.begin( RPM_FWHEEL_PIN);
	if( enableRearWheel )
		freq2.begin( RPM_RWHEEL_PIN);

	Serial.println("Setup - ADC");
	setup_adc();
	Serial.println("Setup - DMA");
	setup_dma();
	Serial.println("Setup - IMU");
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
	Serial.println("Setup - IMU - scan");
	I2Cscan();
	Serial.println("Setup - IMU - configure");
#ifdef EM7180_LIB
	if (!em7180.begin()) {
		Serial.println(em7180.getErrorString());
	}
#endif

#ifdef EM7180
	setupEM7180();
#endif

#ifdef BNO055
	setupBNO();
#endif

	Serial.println("Setup - Digital Pins");
	pinMode(DIGITAL_FBRAKE_PIN, INPUT_PULLUP);
	pinMode(DIGITAL_NEUTRAL_PIN, INPUT_PULLUP);

	Serial.println("Setup - Digital ISR");
	attachInterrupt(DIGITAL_FBRAKE_PIN, digitalPinISR0, CHANGE);
	if (enableNeutral )
		attachInterrupt(DIGITAL_NEUTRAL_PIN, digitalPinISR1, CHANGE);

	Serial.println("Setup - Enable Pins");
	pinMode( ENABLE_PIN, INPUT_PULLDOWN);

	Serial.println("Setup - Start SD slot");
	SD.begin();
	SD.chvol();

	Serial.println("\nSetup - End");
	digitalWrite( INTERNAL_LED, LOW);
	if( RGB_LED) {
		setLEDColour( LED_OFF);
	}
	else if( !RGB_LED ){
		digitalWrite( ENABLE_LED, LOW);
	}
}

void loop() {
	if( RGB_LED && rtcSet && !loggingEnabled )
		setLEDColour( LED_LIGHT_YELLOW );
	if( RGB_LED && !rtcSet && !loggingEnabled )
		setLEDColour( LED_LIGHT_RED );
	if (digitalReadFast( ENABLE_PIN)) {
		if (!loggingEnabled) {
			loggingEnabled = true;
			if( RGB_LED )
				setLEDColour( LED_LIGHT_GREEN);
			else if( !RGB_LED )
				digitalWrite( ENABLE_LED, HIGH);
		}
	}
	if (!digitalReadFast( ENABLE_PIN)) {
		if (loggingEnabled) {
			loggingEnabled = false;
			if( RGB_LED )
				setLEDColour( LED_OFF);
			else if (!RGB_LED )
				digitalWrite( ENABLE_LED, LOW);
		}
	}
	currentFlash = millis();
	if (!RGB_LED && !loggingEnabled && gps.satellites.value() > 3 && currentFlash - lastFlash >= 500) {
		lastFlash = currentFlash;
		if (digitalReadFast( ENABLE_LED))
			digitalWrite( ENABLE_LED, LOW);
		else
			digitalWrite( ENABLE_LED, HIGH);
	}

#if defined(USB_MTPDISK)
  if( !loggingEnabled ) {
    mtpd.loop();
  }
#endif

    if( fileLogRPM && loggingEnabled && !fileRPM ) {
		String fileName = getFileName("rpm");
		Serial.println("Creating file " + fileName);
		fileRPM.open(fileName.c_str(), O_RDWR | O_CREAT);
    }
    if (fileLogGPS && loggingEnabled && !fileGPS && rtcSet) {
		String fileName = getFileName("gps");
		Serial.println("Creating file " + fileName);
		fileGPS.open(fileName.c_str(), O_RDWR | O_CREAT);
	}

	if (fileLogBIN && loggingEnabled && !fileBIN && rtcSet) {
		String fileName = getFileName("run");
		Serial.println("Creating file " + fileName);
		fileBIN.open(fileName.c_str(), O_RDWR | O_CREAT);

		char *bin = NULL;
		size_t length = bike.encodeLoggerDetails(&bin);
		if (outputType == BINARY) {
			Serial2.write(bin, length);
		}
		if (fileLogBIN && fileBIN) {
			digitalWrite( INTERNAL_LED, HIGH);
			fileBIN.write(bin, length);
			digitalWrite( INTERNAL_LED, LOW);
		}
		free(bin);
	}

	if (fileLogVBO && loggingEnabled && !fileVBO && rtcSet) {
		String fileName = getFileName("vbo");
		Serial.println("Creating file " + fileName);
		fileVBO.open(fileName.c_str(), O_RDWR | O_CREAT);
		addVBOHeader();
	}

	if (fileLogCSV && loggingEnabled && !fileCSV && rtcSet) {
		String fileName = getFileName("csv");
		Serial.println("Creating file " + fileName);
		fileCSV.open(fileName.c_str(), O_RDWR | O_CREAT);
		addCSVHeader();
	}

	if (fileLogRPM && !loggingEnabled && fileRPM) {
		fileRPM.flush();
		fileRPM.close();
	}
	if (fileLogGPS && !loggingEnabled && fileGPS) {
		fileGPS.flush();
		fileGPS.close();
	}
	if (fileLogBIN && !loggingEnabled && fileBIN) {
		fileBIN.flush();
		fileBIN.close();
	}
	if (fileLogVBO && !loggingEnabled && fileVBO) {
		fileVBO.flush();
		fileVBO.close();
	}
	if (fileLogCSV && !loggingEnabled && fileCSV) {
		fileCSV.flush();
		fileCSV.close();
	}

	if (freq0.available()) {
		bike.setEngineSpeed(30.0 * freq0.countToFrequency(freq0.read()));
		rpmCounter ++;
		engineRPMSet = true;
	}
	if (enableFrontWheel && freq1.available()) {
		bike.setFWheelSpeed(freq1.countToFrequency(freq1.read()));
		frontRPMSet = true;
	}
	if (enableRearWheel && freq2.available()) {
		bike.setRWheelSpeed(freq2.countToFrequency(freq2.read()));
		rearRPMSet = true;
	}

	currentRPMTime = millis();
	if (currentRPMTime - lastRPMTime >= 250) {
		lastRPMTime = currentRPMTime;
		if(!engineRPMSet) {
			bike.setEngineSpeed( 0.0 );
		}
		if(enableFrontWheel && !frontRPMSet) {
			bike.setFWheelSpeed( 0.0 );
		}
		if(enableRearWheel && !rearRPMSet) {
			bike.setRWheelSpeed( 0.0 );
		}
		engineRPMSet = false;
		frontRPMSet = false;
		rearRPMSet = false;
	}


	currentIMUTime = millis();
	if (currentIMUTime - lastIMUTime >= IMU_INTERVAL) {
		lastIMUTime = currentIMUTime;

#ifdef BNO055
		loopBNO();
#endif

#ifdef EM7180
		loopEM7180();
#endif


#ifdef EM7180_LIB
		em7180.checkEventStatus();

		if (em7180.gotError()) {
			Serial.print("ERROR: ");
			Serial.println(em7180.getErrorString());
		}
		if (em7180.gotQuaternion()) {
			itemsIMUPerSecond++;

			float qw, qx, qy, qz;

			em7180.readQuaternion(qw, qx, qy, qz);

			float roll = atan2(2.0f * (qw * qx + qy * qz),
					qw * qw - qx * qx - qy * qy + qz * qz);
			float pitch = -asin(2.0f * (qx * qz - qw * qy));
			float yaw = atan2(2.0f * (qx * qy + qw * qz),
					qw * qw + qx * qx - qy * qy - qz * qz);

			pitch *= 180.0f / PI;
			yaw *= 180.0f / PI;
			yaw += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
			if (yaw < 0)
				yaw += 360.0f; // Ensure yaw stays between 0 and 360
			roll *= 180.0f / PI;

			bike.setQuaternion(yaw, pitch, roll);
			if(!bike.hasPitchRollStart() && rtcSet ) {
				bike.setPitchRollStart( pitch, roll );
			}
		}

		if (em7180.gotAccelerometer()) {
			float ax, ay, az;
			em7180.readAccelerometer(ax, ay, az);
			bike.setAccelerometer(ax, ay, az);
		}

		if (em7180.gotGyrometer()) {
			float gx, gy, gz;
			em7180.readGyrometer(gx, gy, gz);
			bike.setGyroscope(gx, gy, gz);
		}

		if (em7180.gotMagnetometer()) {

			float mx, my, mz;
			em7180.readMagnetometer(mx, my, mz);
			bike.setMagnetometer(mx, my, mz);
		}

		if (em7180.gotBarometer()) {
			float temperature, pressure;

			em7180.readBarometer(pressure, temperature);
			float altitude = (1.0f - powf(pressure / 1013.25f, 0.190295f)) * 44330.0f;
			bike.setPressure(temperature, pressure, altitude);
		}
#endif
	}

	if (Serial1.available()) {
		while (Serial1.available()) {
			char c = Serial1.read();
			gps.encode(c);
			//Serial.write( c );

			if (rtcSet) {
				gpsLine[gpsCount] = c;
				gpsCount++;
				if (c == '\n') {
					gpsLine[gpsCount] = 0;
					if (!strncmp(gpsLine, "$GPRMC", 6)) {
						itemsGPSPerSecond++;
					}

					if (outputType == GPS || outputType == GPS_RC3) {
						Serial2.print(gpsLine);
					}
					if (loggingEnabled && fileLogGPS && fileGPS) {
						digitalWrite( INTERNAL_LED, HIGH);
						if (fileGPS.write(gpsLine, gpsCount) == -1) {
							Serial.println(
									"=-=-=-=-=-=-=-=-=-=-=- ERROR Writting to GPS file =-=-=-=-=-=-=-=-=-=-=-");
						}
						digitalWrite( INTERNAL_LED, LOW);
					}
					memset(gpsLine, 0, sizeof(gpsLine));
					gpsCount = 0;
				}
			}
		}
	}

	if (!rtcSet && gps.location.age() < 1000 && gps.time.isUpdated()
			&& gps.time.age() < 1000 && gps.date.year() > 2000) {
		rtcCount++;
		if (rtcCount > 10) {
			Serial.print("Setting time to: ");
			Serial.print(gps.date.year());
			Serial.print("/");
			Serial.print(gps.date.month());
			Serial.print("/");
			Serial.print(gps.date.day());
			Serial.print(" ");
			Serial.print(gps.time.hour());
			Serial.print(":");
			Serial.print(gps.time.minute());
			Serial.print(":");
			Serial.println(gps.time.second());

			setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
					gps.date.day(), gps.date.month(), gps.date.year());
			adjustTime(-14400);
			rtcSet = true;
		}
	}

	currentRC3SendTime = millis();
	if (rtcSet && ( outputType == GPS_RC3 || outputType == RC3_ONLY )
			&& currentRC3SendTime - lastRC3SendTime >= RC3_INTERVAL) {
		lastRC3SendTime = currentRC3SendTime;
		itemsRC3PerSecond++;
		populateRC3Buffer();
		Serial2.print(rc3Buffer);
		if (loggingEnabled && fileLogGPS && fileGPS) {
			digitalWrite( INTERNAL_LED, HIGH);
			fileGPS.write(rc3Buffer);
			digitalWrite( INTERNAL_LED, LOW);
		}
	}

	currentVBOSendTime = millis();
	if (fileLogVBO && fileVBO
			&& currentVBOSendTime - lastVBOSendTime >= VBO_INTERVAL) {
		lastVBOSendTime = currentVBOSendTime;
		itemsVBOPerSecond++;
		digitalWrite( INTERNAL_LED, HIGH);
		populateVBOLine();
		fileVBO.println(vboBuffer);
		digitalWrite( INTERNAL_LED, LOW);
	}

	currentCSVSendTime = millis();
	if (fileLogCSV && fileCSV
			&& currentCSVSendTime - lastCSVSendTime >= CSV_INTERVAL) {
		lastCSVSendTime = currentCSVSendTime;
		itemsCSVPerSecond++;
		digitalWrite( INTERNAL_LED, HIGH);
		populateCSVLine();
		fileCSV.println(csvBuffer);
		digitalWrite( INTERNAL_LED, LOW);
	}

	currentBINSendTime = millis();
	if ((fileLogBIN || outputType == BINARY)
			&& currentBINSendTime - lastBINSendTime >= BIN_INTERVAL
			&& bike.isDataAvailable()) {
		lastBINSendTime = currentBINSendTime;
		itemsBINPerSecond++;
		char *bin = NULL;
		size_t length = bike.encodeBINDirty(&bin);
		if (outputType == BINARY) {
			Serial2.write(bin, length);
		}
		if (fileLogBIN && fileBIN) {
			digitalWrite( INTERNAL_LED, HIGH);
			fileBIN.write(bin, length);
			digitalWrite( INTERNAL_LED, LOW);
		}
		free(bin);
		bike.dataSent();
	}

	loopCounter++;

	currentDisplayTime = millis();
	if (currentDisplayTime - lastDisplayTime >= DISPLAY_INTERVAL) {
		lastDisplayTime = currentDisplayTime;
		refreshDisplay();
	}

}

void setup_dma() {
	dma0->begin(true);              // allocate the DMA channel
	dma0->TCD->SADDR = &ADC0_RA;    // where to read from
	dma0->TCD->SOFF = 0;            // source increment each transfer
	dma0->TCD->ATTR = 0x101;
	dma0->TCD->NBYTES = 2;     // bytes per transfer
	dma0->TCD->SLAST = 0;
	dma0->TCD->DADDR = &adcbuffer_0[0];     // where to write to
	dma0->TCD->DOFF = 2;
	dma0->TCD->DLASTSGA = -2 * BUF_SIZE;
	dma0->TCD->BITER = BUF_SIZE;
	dma0->TCD->CITER = BUF_SIZE;
	dma0->triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
	dma0->disableOnCompletion();    // require restart in code
	dma0->interruptAtCompletion();
	dma0->attachInterrupt(dma0_isr);

	dma1->begin(true);              // allocate the DMA channel
	dma1->TCD->SADDR = &ChannelsCfg_0[0];
	dma1->TCD->SOFF = 2;            // source increment each transfer (n bytes)
	dma1->TCD->ATTR = 0x101;
	dma1->TCD->SLAST = -8;          // num ADC0 samples * 2
	dma1->TCD->BITER = 4;           // num of ADC0 samples
	dma1->TCD->CITER = 4;           // num of ADC0 samples
	dma1->TCD->DADDR = &ADC0_SC1A;
	dma1->TCD->DLASTSGA = 0;
	dma1->TCD->NBYTES = 2;
	dma1->TCD->DOFF = 0;
	dma1->triggerAtTransfersOf(*dma0);
	dma1->triggerAtCompletionOf(*dma0);

	dma2->begin(true);              // allocate the DMA channel
	dma2->TCD->SADDR = &ADC1_RA;    // where to read from
	dma2->TCD->SOFF = 0;            // source increment each transfer
	dma2->TCD->ATTR = 0x101;
	dma2->TCD->NBYTES = 2;     // bytes per transfer
	dma2->TCD->SLAST = 0;
	dma2->TCD->DADDR = &adcbuffer_1[0];     // where to write to
	dma2->TCD->DOFF = 2;
	dma2->TCD->DLASTSGA = -2 * BUF_SIZE;
	dma2->TCD->BITER = BUF_SIZE;
	dma2->TCD->CITER = BUF_SIZE;
	dma2->triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);
	dma2->disableOnCompletion();    // require restart in code
	dma2->interruptAtCompletion();
	dma2->attachInterrupt(dma2_isr);

	dma3->begin(true);              // allocate the DMA channel
	dma3->TCD->SADDR = &ChannelsCfg_1[0];
	dma3->TCD->SOFF = 2;            // source increment each transfer (n bytes)
	dma3->TCD->ATTR = 0x101;
	dma3->TCD->SLAST = -8;          // num ADC1 samples * 2
	dma3->TCD->BITER = 4;           // num of ADC1 samples
	dma3->TCD->CITER = 4;           // num of ADC1 samples
	dma3->TCD->DADDR = &ADC1_SC1A;
	dma3->TCD->DLASTSGA = 0;
	dma3->TCD->NBYTES = 2;
	dma3->TCD->DOFF = 0;
	dma3->triggerAtTransfersOf(*dma2);
	dma3->triggerAtCompletionOf(*dma2);

	dma0->enable();
	dma1->enable();

	dma2->enable();
	dma3->enable();

}

void setup_adc() {
	//ADC0
	adc->adc0->setAveraging(16);
	adc->adc0->setResolution(12); // set bits of resolution
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_0); // change the conversion speed
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED, ADC_0); // change the sampling speed
	//adc->adc0->setReference(ADC_REFERENCE::REF_3V3);

	//ADC1
	adc->adc1->setAveraging(16);
	adc->adc1->setResolution(12); // set bits of resolution
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_1); // change the conversion speed
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED, ADC_1); // change the sampling speed
	//adc->adc1->setReference(ADC_REFERENCE::REF_3V3);`

	bike.maxADC = 0.01 * adc->adc0->getMaxValue();

	ADC1_CFG2 |= ADC_CFG2_MUXSEL;

	adc->adc0->enableDMA(); //ADC0_SC2 |= ADC_SC2_DMAEN;  // using software trigger, ie writing to ADC0_SC1A
	adc->adc1->enableDMA();

}

void dma0_isr(void) {
	bike.setFBrakePressure(adcbuffer_0[0]);
	bike.setFSuspension(adcbuffer_0[1]);
	bike.setGear(adcbuffer_0[2]);
	//bike.setAnalog1(adcbuffer_0[3]);

	dma0->TCD->DADDR = &adcbuffer_0[0];
	dma0->clearInterrupt();
	dma0->enable();
}

void dma2_isr(void) {
	bike.setTps(adcbuffer_1[0]);
	bike.setRSuspension(adcbuffer_1[1]);
	bike.setO2Sensor(adcbuffer_1[2]);
	//bike.setAnalog3(adcbuffer_1[3]);

	dma2->TCD->DADDR = &adcbuffer_1[0];
	dma2->clearInterrupt();
	dma2->enable();
}

void digitalPinISR0(void) {
	bike.setFBrakeSwitch(!(digitalReadFast( DIGITAL_FBRAKE_PIN) == HIGH));
}

void digitalPinISR1(void) {
	bike.setNeutralSwitch((digitalReadFast( DIGITAL_NEUTRAL_PIN) == HIGH));
}

void refreshDisplay() {
	if (!displayData) {
		Serial.print("Date: ");
		Serial.print(month());
		Serial.print("/");
		Serial.print(day());
		Serial.print("/");
		Serial.print(year());
		Serial.print("  Time: ");
		Serial.print(hour());
		Serial.print(":");
		Serial.print(minute());
		Serial.print(":");
		Serial.println(second());

		Serial.print("\nRTC Set:\t");
		Serial.println((rtcSet) ? "Yes" : "No");
		Serial.print("GPS per second:\t");
		Serial.print(itemsGPSPerSecond);
		Serial.print("\tBIN per second:\t");
		Serial.print(itemsBINPerSecond);
		Serial.print("\tLOOP per second: ");
		Serial.print(loopCounter);
		Serial.print("\tRPM per second:\t");
		Serial.println(rpmCounter);
		Serial.print("RC3 per second:\t");
		Serial.print(itemsRC3PerSecond);
		Serial.print("\tVBO per second:\t");
		Serial.print(itemsVBOPerSecond);
		Serial.print("\tCSV per second:\t");
		Serial.print(itemsCSVPerSecond);
		Serial.print("\tIMU per second:\t");
		Serial.println(itemsIMUPerSecond);

		Serial.println();


		itemsGPSPerSecond = 0;
		itemsBINPerSecond = 0;
		itemsRC3PerSecond = 0;
		itemsVBOPerSecond = 0;
		itemsCSVPerSecond = 0;
		itemsIMUPerSecond = 0;
		loopCounter = 0;
		rpmCounter = 0;
		return;
	}
	if (!loggingEnabled) {
		digitalWrite( INTERNAL_LED, HIGH);
	}

	Serial.println(
			"========================================================================================================================");
	Serial.print("Logging Enabled:\t");
	Serial.print((loggingEnabled) ? "Yes" : "No");
	Serial.print("\tRTC Set:\t\t");
	Serial.print((rtcSet) ? "Yes" : "No");
	Serial.print("\tVersion:\t");
	Serial.println( _VERSION,3 );

	Serial.print("\nGPS Logging:\t\t");
	Serial.print((fileGPS) ? "True" : "False");
	Serial.print("\tRUN Logging:\t\t");
	Serial.print((fileBIN) ? "True" : "False");
	Serial.println();

	Serial.print("VBO Logging:\t\t");
	Serial.print((fileVBO) ? "True" : "False");
	Serial.print("\tCSV Logging:\t\t");
	Serial.print((fileCSV) ? "True" : "False");
	Serial.println();

	Serial.print("\nMax ADC:\t\t");
	Serial.println(bike.maxADC, 4);

	Serial.print("\nTPS:\t\t");
	Serial.print(bike.getTps());
	Serial.print("(");
	Serial.print(bike.getTpsRaw());
	Serial.print(")");
	Serial.print("\tGear:\t\t");
	Serial.print(bike.getGearPercent());
	Serial.print("(");
	Serial.print(bike.getGearRaw());
	Serial.print(")");
	Serial.print("\tBrake:\t\t");
	Serial.print(bike.getFBrakePressure());
	Serial.print("(");
	Serial.print(bike.getFBrakePressureRaw());
	Serial.println(")");

	Serial.print("F Suspension:\t");
	Serial.print(bike.getFSuspension());
	Serial.print("(");
	Serial.print(bike.getFSuspensionRaw());
	Serial.print(")");
	Serial.print("\tR Suspension:\t");
	Serial.print(bike.getRSuspension());
	Serial.print("(");
	Serial.print(bike.getRSuspensionRaw());
	Serial.print(")");
	Serial.print("\tO2 Sensor:\t");
	Serial.print(bike.getO2Sensor());
	Serial.print("(");
	Serial.print(bike.getO2SensorRaw());
	Serial.println(")");

	Serial.print("\nFront Brake:\t");
	Serial.print(bike.getFBrakeSwitch());
	Serial.print("\tNeutral:\t");
	Serial.println(bike.getNeutralSwitch());

	Serial.print("\nEgine RPM:\t");
	Serial.print(bike.getEngineSpeed());
	Serial.print("\tF Wheel RPM:\t");
	Serial.print(bike.getFWheelSpeed());
	Serial.print("\tR Wheel RPM:\t");
	Serial.println(bike.getRWheelSpeed());

	struct QuaternionData quaternions = bike.getQuaternion();
	struct AccelerometerData accelerometers = bike.getAccelerometer();
	struct GyroscopeData gyros = bike.getGyroscope();
	struct PressureData pressure = bike.getPressure();

	Serial.print("\nTemp:\t\t");
	Serial.print(pressure.temperature);
	Serial.print("\tPressure:\t\t");
	Serial.print(pressure.pressure);
	Serial.print("\tIMU Altitude:\t\t");
	Serial.println(pressure.altitude);

	Serial.print("\nYaw:\t\t");
	Serial.print(quaternions.yaw);
	Serial.print("\tPitch:");
	Serial.print( "(");Serial.print( bike.getPitchStart() ); Serial.print( ")\t");
	Serial.print(quaternions.pitch);
	Serial.print("\tRoll:");
	Serial.print( "(");Serial.print( bike.getRollStart() ); Serial.print( ")\t");
	Serial.println(quaternions.roll);
	Serial.print("A-X:\t\t");
	Serial.print(accelerometers.ax);
	Serial.print("\tA-Y:\t\t");
	Serial.print(accelerometers.ay);
	Serial.print("\tA-Z:\t\t");
	Serial.println(accelerometers.az);
	Serial.print("G-X:\t\t");
	Serial.print(gyros.gx);
	Serial.print("\tG-Y:\t\t");
	Serial.print(gyros.gy);
	Serial.print("\tG-Z:\t\t");
	Serial.println(gyros.gz);

	Serial.print("\nGPS per second:\t");
	Serial.print(itemsGPSPerSecond);
	Serial.print("\tBIN per second:\t");
	Serial.print(itemsBINPerSecond);
	Serial.print("\tLOOP per second: ");
	Serial.print(loopCounter);
	Serial.print("\tRPM per second:\t");
	Serial.println(rpmCounter);
	Serial.print("RC3 per second:\t");
	Serial.print(itemsRC3PerSecond);
	Serial.print("\tVBO per second:\t");
	Serial.print(itemsVBOPerSecond);
	Serial.print("\tCSV per second:\t");
	Serial.print(itemsCSVPerSecond);
	Serial.print("\tIMU per second:\t");
	Serial.println(itemsIMUPerSecond);

	Serial.println();

	Serial.print("Date: ");
	Serial.print(month());
	Serial.print("/");
	Serial.print(day());
	Serial.print("/");
	Serial.print(year());
	Serial.print("  Time: ");
	Serial.print(hour());
	Serial.print(":");
	Serial.print(minute());
	Serial.print(":");
	Serial.println(second());
	Serial.println();

	itemsGPSPerSecond = 0;
	itemsBINPerSecond = 0;
	itemsRC3PerSecond = 0;
	itemsVBOPerSecond = 0;
	itemsCSVPerSecond = 0;
	itemsIMUPerSecond = 0;
	loopCounter = 0;
	rpmCounter = 0;
	gpsdump();
	if (!loggingEnabled) {
		digitalWrite( INTERNAL_LED, LOW);
	}
}

void gpsdump() {
	Serial.print(F("LOCATION   Fix Age="));
	Serial.print(gps.location.age());
	Serial.print(F("ms Raw Lat="));
	Serial.print(gps.location.rawLat().negative ? "-" : "+");
	Serial.print(gps.location.rawLat().deg);
	Serial.print("[+");
	Serial.print(gps.location.rawLat().billionths);
	Serial.print(F(" billionths],  Raw Long="));
	Serial.print(gps.location.rawLng().negative ? "-" : "+");
	Serial.print(gps.location.rawLng().deg);
	Serial.print("[+");
	Serial.print(gps.location.rawLng().billionths);
	Serial.print(F(" billionths],  Lat="));
	Serial.print(gps.location.lat(), 6);
	Serial.print(F(" Long="));
	Serial.println(gps.location.lng(), 6);

	Serial.print(F("DATE       Fix Age="));
	Serial.print(gps.date.age());
	Serial.print(F("ms Raw="));
	Serial.print(gps.date.value());
	Serial.print(F(" Year="));
	Serial.print(gps.date.year());
	Serial.print(F(" Month="));
	Serial.print(gps.date.month());
	Serial.print(F(" Day="));
	Serial.println(gps.date.day());

	Serial.print(F("TIME       Fix Age="));
	Serial.print(gps.time.age());
	Serial.print(F("ms Raw="));
	Serial.print(gps.time.value());
	Serial.print(F(" Hour="));
	Serial.print(gps.time.hour());
	Serial.print(F(" Minute="));
	Serial.print(gps.time.minute());
	Serial.print(F(" Second="));
	Serial.print(gps.time.second());
	Serial.print(F(" Hundredths="));
	Serial.println(gps.time.centisecond());

	Serial.print(F("SPEED      Fix Age="));
	Serial.print(gps.speed.age());
	Serial.print(F("ms Raw="));
	Serial.print(gps.speed.value());
	Serial.print(F(" Knots="));
	Serial.print(gps.speed.knots());
	Serial.print(F(" MPH="));
	Serial.print(gps.speed.mph());
	Serial.print(F(" m/s="));
	Serial.print(gps.speed.mps());
	Serial.print(F(" km/h="));
	Serial.println(gps.speed.kmph());

	Serial.print(F("COURSE     Fix Age="));
	Serial.print(gps.course.age());
	Serial.print(F("ms Raw="));
	Serial.print(gps.course.value());
	Serial.print(F(" Deg="));
	Serial.println(gps.course.deg());

	Serial.print(F("ALTITUDE   Fix Age="));
	Serial.print(gps.altitude.age());
	Serial.print(F("ms Raw="));
	Serial.print(gps.altitude.value());
	Serial.print(F(" Meters="));
	Serial.print(gps.altitude.meters());
	Serial.print(F(" Miles="));
	Serial.print(gps.altitude.miles());
	Serial.print(F(" KM="));
	Serial.print(gps.altitude.kilometers());
	Serial.print(F(" Feet="));
	Serial.println(gps.altitude.feet());

	Serial.print(F("SATELLITES Fix Age="));
	Serial.print(gps.satellites.age());
	Serial.print(F("ms Value="));
	Serial.println(gps.satellites.value());

	Serial.print(F("HDOP       Fix Age="));
	Serial.print(gps.hdop.age());
	Serial.print(F("ms raw="));
	Serial.print(gps.hdop.value());
	Serial.print(F(" hdop="));
	Serial.println(gps.hdop.hdop());

	Serial.print(F("DIAGS      Chars="));
	Serial.print(gps.charsProcessed());
	Serial.print(F(" Sentences-with-Fix="));
	Serial.print(gps.sentencesWithFix());
	Serial.print(F(" Failed-checksum="));
	Serial.print(gps.failedChecksum());
	Serial.print(F(" Passed-checksum="));
	Serial.println(gps.passedChecksum());

	if (gps.charsProcessed() < 10)
		Serial.println(F("WARNING: No GPS data.  Check wiring."));
}

String getFileName(String ext) {
	char fileName[20];
	memset(fileName, 0, sizeof(fileName));
	sprintf(fileName, "%04d%02d%02d-%02d%02d%02d.%s", year(), month(), day(),
			hour(), minute(), second(), ext.c_str());
	return String(fileName);
}

void addCSVHeader(void) {
	char tempStr[100];
	if (fileCSV) {
		sprintf(tempStr, "File created on %02d/%02d/%04d at %02d:%02d:%02d",
				day(), month(), year(), hour(), minute(), second());
		fileCSV.println(tempStr);
		fileCSV.println("sats,"
				"date,"
				"time,"
				"lat,"
				"long,"
				"velocity,"
				"heading,"
				"altitude,"
				"temperature,"
				"pressure,"
				"x-accelerometer,"
				"y-accelerometer,"
				"z-accelerometer,"
				"x-gyro,"
				"y-gyro,"
				"z-gyro,"
				"roll,"
				"pitch,"
				"yaw,"
				"tps,"
				"tps-raw,"
				"rpm,"
				"neutral,"
				"gear-count,"
				"gear,"
				"f-brake-switch,"
				"f-brake-pressure,"
				"f-suspension,"
				"f-rpm,"
				"r-suspension,"
				"r-rpm");
	}
}

void addVBOHeader(void) {
	char tempStr[100];
	if (fileVBO) {
		sprintf(tempStr, "File created on %02d/%02d/%04d at %02d:%02d:%02d",
				day(), month(), year(), hour(), minute(), second());
		fileVBO.println(tempStr);
		fileVBO.println("\n[header]");
		fileVBO.println("satellites");
		fileVBO.println("time");
		fileVBO.println("latitude");
		fileVBO.println("longitude");
		fileVBO.println("velocity kmh");
		fileVBO.println("heading");
		fileVBO.println("height");
		fileVBO.println("long accel g");
		fileVBO.println("lat accel g");
		fileVBO.println("device-update-rate");
		fileVBO.println("lean-angle");
		fileVBO.println("fix-type");
		fileVBO.println("coordinate-precision");
		fileVBO.println("altitude-precision");
		fileVBO.println("x-acceleration-datalogger g");
		fileVBO.println("y-acceleration-datalogger g");
		fileVBO.println("z-acceleration-datalogger g");
		fileVBO.println("x-rotation-datalogger");
		fileVBO.println("y-rotation-datalogger");
		fileVBO.println("z-rotation-datalogger");
		fileVBO.println("device-update-rate-datalogger");
		fileVBO.println("rpm-datalogger");
		fileVBO.println("roll-datalogger");
		fileVBO.println("pitch-datalogger");
		fileVBO.println("yaw-datalogger");
		fileVBO.println("tps-datalogger");
		fileVBO.println("neutral-datalogger");
		fileVBO.println("brake-switch-datalogger");
		fileVBO.println("gear-raw-datalogger");
		fileVBO.println("gear-num-datalogger");
		fileVBO.println("front-brake-pressure-datalogger");
		fileVBO.println("front-suspension-datalogger");
		fileVBO.println("front-rpm-datalogger");
		fileVBO.println("rear-suspension-datalogger");
		fileVBO.println("rear-rpm-datalogger");
		fileVBO.println("temp-datalogger");
		fileVBO.println("pressure-datalogger");
		fileVBO.println("\n[avi]");
		fileVBO.println("\n[comments]");
		sprintf(tempStr, "Generated by TrackDAQ v%0.03f", _VERSION);
		fileVBO.println(tempStr);

		addTrackDetails();

		fileVBO.println("\n[column names]");
		fileVBO.println( "sats time lat long velocity heading height longacc latacc device-update-rate lean-angle fix-type coordinate-precision altitude-precision x-acceleration-datalogger y-acceleration-datalogger z-acceleration-datalogger x-rotation-datalogger y-rotation-datalogger z-rotation-datalogger device-update-rate-datalogger rpm-datalogger roll-datalogger pitch-datalogger yaw-datalogger tps-datalogger neutral-datalogger brake-switch-datalogger gear-raw-datalogger gear-num-datalogger front-brake-pressure-datalogger front-suspension-datalogger front-rpm-datalogger rear-suspension-datalogger rear-rpm-datalogger temp-datalogger pressure-datalogger");
		fileVBO.println("\n[data]");
	}
}

void populateVBOLine() {
	struct QuaternionData quaternions = bike.getQuaternion();
	struct AccelerometerData accelerometers = bike.getAccelerometer();
	struct GyroscopeData gyros = bike.getGyroscope();
	struct PressureData pressure = bike.getPressure();

	int seconds = gps.time.second();
	long milliSeconds = (gps.time.centisecond() * 10) + ((gps.time.age() < 10) ? 0 : gps.time.age());
	if( milliSeconds >= 1000 ) {
		milliSeconds = milliSeconds - (milliSeconds - 1000) -1;
	}

	sprintf(vboBuffer,
			"%02lu "	// satellites
					"%02d%02d%02d.%03lu "// time
					"%f "// latitude
					"%f "// longitude
					"%f "// velocity kmh
					"%f "// heading
					"%f "// height
					"%f "// long accel g
					"%f "// lat accel g
					"%d "// device-update-rate
					"%f "// lean-angle
					"%d "// fix-type
					"%f "// coordinate-precision
					"%f "// altitude-precision
					"%f "// x-acceleration-datalogger g
					"%f "// y-acceleration-datalogger g
					"%f "// z-acceleration-datalogger g
					"%f "// x-rotation-datalogger
					"%f "// y-rotation-datalogger
					"%f "// z-rotation-datalogger
					"%d "// device-update-rate-datalogger
					"%f "// rpm-datalogger
					"%f "// roll-datalogger
					"%f "// pitch-datalogger
					"%f "// yaw-datalogger
					"%f "// tps-datalogger
					"%d "// neutral-datalogger
					"%d "// brake-switch-datalogger
					"%d "// gear-raw-datalogger
					"%d "// gear-num-datalogger
					"%f "// front-brake-pressure-datalogger
					"%f "// front-suspension-datalogger
					"%f "// front-rpm-datalogger
					"%f "// rear-suspension-datalogger
					"%f "// rear-rpm-datalogger
					"%f "// temp-datalogger
					"%f "// pressure-datalogger
			,
			gps.satellites.value(),		// satellites
			hour(), minute(), seconds, milliSeconds,		// time
			(gps.location.lat() < 0) ? gps.location.lat() * -60 : gps.location.lat() * 60,	// latitude
			(gps.location.lng() < 0) ? gps.location.lng() * -60 : gps.location.lng() * 60,	// longitude
			gps.speed.kmph(),			// velocity kmh
			gps.course.deg(),			// heading
			gps.altitude.meters(),		// height
			0.0, 						// long accel g
			0.0, 						// lat accel g
			int(1000.0 / VBO_INTERVAL),	// device-update-rate
			quaternions.roll,			// lean-angle
			1,							// fix-type
			gps.hdop.hdop(), 			// coordinate-precision
			gps.hdop.hdop(),			// altitude-precision
			accelerometers.ax,			// x-acceleration-datalogger g
			accelerometers.ay,			// y-acceleration-datalogger g
			accelerometers.az,			// z-acceleration-datalogger g
			gyros.gx,					// x-rotation-datalogger
			gyros.gy,					// y-rotation-datalogger
			gyros.gz,					// z-rotation-datalogger
			int(1000.0 / VBO_INTERVAL),	// device-update-rate-datalogger
			bike.getEngineSpeed(),		// rpm-datalogger
			quaternions.roll,			// roll-datalogger
			quaternions.pitch,			// pitch-datalogger
			quaternions.yaw,			// yaw-datalogger
			bike.getTps(),				// tps-datalogger
			(bike.getNeutralSwitch() ? 1 : 0),	// neutral-datalogger
			(bike.getFBrakeSwitch() ? 1 : 0),		// brake-switch-datalogger
			bike.getGearRaw(),			// gear-raw-datalogger
			bike.getGearNumber(),		// gear-num-datalogger
			bike.getFBrakePressure(),	// front-brake-pressure-datalogger
			bike.getFSuspension(),		// front-suspension-datalogger
			bike.getFWheelSpeed(),		// front-rpm-datalogger
			bike.getRSuspension(),		// rear-suspension-datalogger
			bike.getRWheelSpeed(),		// rear-rpm-datalogger
			pressure.temperature,		// temp-datalogger
			pressure.pressure			// pressure-datalogger
			);

}

void addTrackDetails() {
	char ch;
	if (SD.exists("track.vbo")) {
		File file = SD.open("track.vbo", FILE_READ);
		file.seek(0);
		 while (file.available()) {
			 if (file.read(&ch, 1) == 1) {
				 fileVBO.write( ch );
			 }
		 }
		 file.close();
	}
}

void populateCSVLine() {
	struct QuaternionData quaternions = bike.getQuaternion();
	struct AccelerometerData accelerometers = bike.getAccelerometer();
	struct GyroscopeData gyros = bike.getGyroscope();
	struct PressureData pressure = bike.getPressure();

	int seconds = gps.time.second();
	long milliSeconds = (gps.time.centisecond() * 10) + ((gps.time.age() < 10) ? 0 : gps.time.age());
	if( milliSeconds >= 1000 ) {
		milliSeconds = milliSeconds - (milliSeconds - 1000) -1;
	}


	sprintf(csvBuffer,
			"%02lu,"				// satelites
					"%02d%02d%04d,"// date
					"%02d%02d%02d.%03lu,"// time
					"%f,"// lat
					"%f,"// lon
					"%03.02f,"// speed
					"%03.02f,"// direction
					"%04.02f,"// altitude
					"%03.02f,"// temp
					"%04.02f,"// pressure
					"%03.02f,"// accelerometer x
					"%03.02f,"// accelerometer y
					"%03.02f,"// accelerometer z
					"%03.02f,"// gyro x
					"%03.02f,"// gyro y
					"%03.02f,"// gyro z
					"%03.02f,"// roll
					"%03.02f,"// pitch
					"%03.02f,"// yaw
					"%03.02f,"// TPS
					"%d,"// TPS-RAW
					"%05f,"// RPM
					"%d,"// Neutral
					"%d,"// Gear V
					"%d,"// Gear Number
					"%d,"// F Brake on/off
					"%03.02f,"// Brake pressure
					"%03.02f,"// F Suspension
					"%03.02f,"// F Wheel Speed
					"%03.02f,"// R Suspension
					"%03.02f",// R Wheel Speed
			gps.satellites.value(), day(), month(), year(), hour(), minute(),
			seconds, milliSeconds,
			gps.location.lat(), gps.location.lng(), gps.speed.kmph(),
			gps.course.deg(), gps.altitude.meters(), pressure.temperature,
			pressure.pressure, accelerometers.ax, accelerometers.ay,
			accelerometers.az, gyros.gx, gyros.gy, gyros.gz, quaternions.roll,
			quaternions.pitch, quaternions.yaw, bike.getTps(),bike.getTpsRaw(),
			bike.getEngineSpeed(), (bike.getNeutralSwitch() ? 1 : 0),
			bike.getGearRaw(), bike.getGearNumber(),
			(bike.getFBrakeSwitch() ? 1 : 0), bike.getFBrakePressure(),
			bike.getFSuspension(), bike.getFWheelSpeed(), bike.getRSuspension(),
			bike.getRWheelSpeed());
}

void populateRC3Buffer() {
	char bufffer[200];
	if (rc3Counter == 65535)
		rc3Counter = 0;
	// $RC3,[time],[count],[xacc],[yacc],[zacc],[gyrox],[gyroy],[gyroz],rpm,front brake,front brake pressure,TPS,Front Suspension,Rear Suspension,Front Wheel RPM,Rear Wheel RPM,Gear,[a8],[a9],[a10],[a11],[a12],Roll,Pitch,Yaw*checksum

	struct QuaternionData quaternions = bike.getQuaternion();
	struct AccelerometerData accelerometers = bike.getAccelerometer();
	struct GyroscopeData gyros = bike.getGyroscope();
	struct PressureData pressure = bike.getPressure();

	int seconds = gps.time.second();
	long milliSeconds = (gps.time.centisecond() * 10) + ((gps.time.age() < 10) ? 0 : gps.time.age());
	if( milliSeconds >= 1000 ) {
		milliSeconds = milliSeconds - (milliSeconds - 1000) -1;
	}

	sprintf(bufffer,
			"RC3,"					// $RC3
					"%02d%02d%02d.%03lu,"// [time]
					"%i,"				// [count]
					"%+01.2f,"			// [xacc]
					"%+01.2f,"			// [yacc]
					"%+01.2f,"			// [zacc]
					"%+01.2f,"			// [gyrox]
					"%+01.2f,"			// [gyroy]
					"%+01.2f,"			// [gyroz]
					"%05.0f,"			// rpm [d1]
					"%i,"				// front brake [d2]
					"%03.2f,"			// front brake pressure [a1]
					"%03.2f,"			// TPS [a2]
					"%03.2f,"			// Front Suspension [a3]
					"%03.0f,"			// Rear Suspension [a4]
					"%03.0f,"			// Front Wheel RPM [a5]
					"%03.2f,"			// Rear Wheel RPM [a6]
					"%d,"				// Gear Number [a7]
					"%d,"				// Gear Raw [a8]
					"%d,"				// TPS Raw [a9]
					"%02.3f,"			// Logger Version [a10]
					"0,"				// [a11]
					"0,"				// [a12]
					"%01.2f,"			// Roll [a13]
					"%01.2f,"			// Pitch [a14]
					"%01.2f",			// Yaw [a15]
			gps.time.hour(), gps.time.minute(), seconds, milliSeconds,   // [time]
			rc3Counter,							// [count]
			accelerometers.ax,					// [xacc]
			accelerometers.ay,					// [yacc]
			accelerometers.az, 					// [zacc]
			gyros.gx,							// [gyrox]
			gyros.gy,							// [gyroy]
			gyros.gz, 							// [gyroz]
			bike.getEngineSpeed(),				// rpm [d1]
			(bike.getFBrakeSwitch()) ? 1 : 0, 	// front brake [d2]
			bike.getFBrakePressure(),			// front brake pressure [a1]
			bike.getTps(),						// TPS [a2]
			bike.getFSuspension(),				// Front Suspension [a3]
			bike.getRSuspension(), 				// Rear Suspension [a4]
			bike.getFWheelSpeed(),				// Front Wheel RPM [a5]
			bike.getRWheelSpeed(), 				// Rear Wheel RPM [a6]
			bike.getGearNumber(), 				// Gear Number [a7]
			bike.getGearRaw(),					// Gear Raw [a8]
			bike.getTpsRaw(),					// TPS Raw [a9]
			_VERSION,							// Version [a10]
			quaternions.roll,					// Roll [a13]
			quaternions.pitch, 					// Pitch [a14]
			quaternions.yaw						// Yaw [a15]
			);

	char crc = getRC3CheckSum(bufffer);
	sprintf(rc3Buffer, "$%s*%02hhX\r\n", bufffer, crc);
	rc3Counter++;
}

char getRC3CheckSum(char *theseChars) {
	char check = 0;

	// iterate over the string, XOR each byte with the total sum:
	int len = strlen(theseChars);
	for (int i = 0; i < len; i++) {
		check = check ^ theseChars[i];
	}
	// return the result
	return check;
}

void setLEDColour(int colour) {
	if( currentRGBLED != colour ) {
		leds.setPixel(0, colour);
		leds.show();
		currentRGBLED = colour;
	}

}
