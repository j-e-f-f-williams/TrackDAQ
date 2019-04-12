#include "Arduino.h"
#include "DMAChannel.h"
#include "ADC.h"
#include "FreqMeasureMulti.h"
#include "EM7180_Master.h"
#include "i2c_t3.h"
#include "TinyGPS++.h"
#include "TimeLib.h"
#include "SdFat.h"

#if defined(USB_MTPDISK)
#include "MTP.h"
#endif
#include "BikeData.h"


#define NOSTOP I2C_NOSTOP
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
      8       Serial3 TX      ------------      N/A
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
      30      LED/Digial      Enable LED        DB9 - 8
      33/A14  Analog          Rear Suspension   DB25 - 22
      34/A15  Analog          Gear Indicator    DB25 - 9
      35/A16  Analog          Front Suspension  DB25 - 23
      36/A17  Analog          TPS               DB25 - 24
      37/A18  Analog          Future            N/A
      38/A19  Analog          O2 Sensor         DB25 - 21
*/

enum outputTypeSet { NONE, GPS, GPS_RC3, BINARY };
static const enum outputTypeSet outputType = GPS_RC3;
static const bool displayData = true;
static const bool fileLogGPS = true;
static const bool fileLogVOB = false;
static const bool fileLogBIN = true;

#define RPM_ENGINE_PIN     21  // Engine RPM
#define RPM_FWHEEL_PIN     5 // Front Wheel RPM
#define RPM_RWHEEL_PIN     6 // Rear Wheel RPM

#define DIGITAL_FBRAKE_PIN    24  // Front Brake Switch
#define DIGITAL_NEUTRAL_PIN    25  // Neutral Switch

#define ENABLE_PIN      29
#define INTERNAL_LED    13
#define ENABLE_LED      30

#define BUF_SIZE 4

static const uint8_t MAG_RATE = 100;  // Hz
static const uint16_t ACCEL_RATE = 200;  // Hz
static const uint16_t GYRO_RATE = 200;  // Hz
static const uint8_t BARO_RATE = 50;   // Hz
static const uint8_t Q_RATE_DIVISOR = 2;    // 1/3 gyro rate
EM7180_Master em7180 = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

#if defined(USB_MTPDISK)
MTPStorage_SD storage;
MTPD          mtpd(&storage);
#endif

#if !defined(USB_MTPDISK)
SdFatSdioEX SD;
#endif
File fileGPS;
File fileBIN;
File fileVOB;


ADC *adc = new ADC(); // adc object

DMAChannel* dma0 = new DMAChannel(false);
DMAChannel* dma1 = new DMAChannel(false);
DMAChannel* dma2 = new DMAChannel(false);
DMAChannel* dma3 = new DMAChannel(false);

FreqMeasureMulti freq0;   // Engine RPM
FreqMeasureMulti freq1;   // Front Wheel RPM
FreqMeasureMulti freq2;   // Rear Wheel RPM

// NOTE: number of DMA must be multiple of 2
//ChannelsCfg order must be {CH1, CH2, CH3, CH0 }, adcbuffer output will be CH0, CH1, CH2, CH3
//Order must be {Second . . . . . . . . First} no matter the number of channels used.
const uint16_t ChannelsCfg_0[] = { 0x51, 0x52, 0x4E, 0x48 }; //ADC0: CH1 A14 [F Sus]  CH2 A15[Gear]   CH3 A1 [Spare 1]    CH0 A2 [Brake]
const uint16_t ChannelsCfg_1[] = { 0x44, 0x47, 0x46, 0x45 }; //ADC1: CH1 A16 [R Sus]  CH2 A19[O2]     CH3 A18[Spare 2]    CH0 A17[TPS]

DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE + 0))) adcbuffer_0[BUF_SIZE];
DMAMEM static volatile uint16_t __attribute__((aligned(BUF_SIZE + 0))) adcbuffer_1[BUF_SIZE];

TinyGPSPlus gps;

TrackDAQ::BikeData bike = TrackDAQ::BikeData();

static const unsigned long BIN_INTERVAL = 9; // in ms.  5 = 200hz  10 = 100hz
unsigned long currentBINSendTime;
unsigned long lastBINSendTime = millis();

static const unsigned long RC3_INTERVAL = 20; // in ms.  5 = 200hz  10 = 100hz   20 = 50Hz
unsigned long currentRC3SendTime;
unsigned long lastRC3SendTime = millis();
char rc3Buffer[200];
int rc3Counter = 0;

static const unsigned long VOB_INTERVAL = 10; // in ms.  5 = 200hz  10 = 100hz
unsigned long currentVOBSendTime;
unsigned long lastVOBSendTime = millis();

static const unsigned long DISPLAY_INTERVAL = 1000; // in ms.  1/s
unsigned long currentDisplayTime;
unsigned long lastDisplayTime = millis();

int itemsGPSPerSecond = 0;
int itemsBINPerSecond = 0;
int itemsRC3PerSecond = 0;
int itemsVOBPerSecond = 0;

bool rtcSet = false;
bool loggingEnabled = false;

char gpsLine[100];
int gpsCount = 0;


void setup() {
  Serial.println("Setup - Start");
  // initialize the digital pin as an output.
  if( displayData )
    delay(10000);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

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
  freq1.begin( RPM_FWHEEL_PIN);
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
  if (!em7180.begin()) {
    Serial.println(em7180.getErrorString());
  }
  Serial.println("Setup - Digital Pins");
  pinMode(DIGITAL_FBRAKE_PIN, INPUT_PULLUP);
  pinMode(DIGITAL_NEUTRAL_PIN, INPUT_PULLUP);

  Serial.println("Setup - Digital ISR");
  attachInterrupt(DIGITAL_FBRAKE_PIN, digitalPinISR0, CHANGE);
  attachInterrupt(DIGITAL_NEUTRAL_PIN, digitalPinISR1, CHANGE);

  Serial.println("Setup - Enable Pins");
  pinMode( ENABLE_PIN, INPUT_PULLUP);
  pinMode( INTERNAL_LED, OUTPUT);
  pinMode( ENABLE_LED, OUTPUT);

  Serial.println("Setup - Start SD slot");
  SD.begin();
  
  Serial.println("\nSetup - End");

}

void loop() {
  if ( digitalReadFast( ENABLE_PIN ) ) {
    if ( !loggingEnabled ) {
      loggingEnabled = true;
      digitalWrite( ENABLE_LED, HIGH );
    }
  }
  if ( !digitalReadFast( ENABLE_PIN ) ) {
    if ( loggingEnabled ) {
      loggingEnabled = false;
      digitalWrite( ENABLE_LED, LOW );
    }
  }

#if defined(USB_MTPDISK)
  if( !loggingEnabled ) {
    mtpd.loop();
  }
#endif

  if ( fileLogGPS && loggingEnabled && !fileGPS && rtcSet ) {
    String fileName = getFileName( "gps" );
    Serial.println( "Creating file " + fileName );
    fileGPS.open( fileName.c_str(), O_RDWR | O_CREAT);
  }

  if ( fileLogBIN && loggingEnabled && !fileBIN && rtcSet ) {
    String fileName = getFileName( "run" );
    Serial.println( "Creating file " + fileName );
    fileBIN.open( fileName.c_str(), O_RDWR | O_CREAT);

    char *bin = NULL;
    size_t length = bike.encodeLoggerDetails(&bin);
    if( outputType == BINARY ) {
      Serial2.write(bin,length);
    }
    if( fileLogBIN && fileBIN ) {
      digitalWrite( INTERNAL_LED, HIGH );
      fileBIN.write( bin, length );
      digitalWrite( INTERNAL_LED, LOW );
    }
    free(bin);
  }

  if ( fileLogVOB && loggingEnabled && !fileVOB && rtcSet ) {
    String fileName = getFileName( "csv" );
    Serial.println( "Creating file " + fileName );
    fileVOB.open( fileName.c_str(), O_RDWR | O_CREAT);
    addVOBHeader();
  }


  if ( fileLogGPS && !loggingEnabled && fileGPS ) {
    fileGPS.flush();
    fileGPS.close();
  }
  if ( fileLogBIN && !loggingEnabled && fileBIN ) {
    fileBIN.flush();
    fileBIN.close();
  }
  if ( fileLogVOB && !loggingEnabled && fileVOB ) {
    fileVOB.flush();
    fileVOB.close();
  }


  if (freq0.available()) {
    bike.setEngineSpeed(30.0 * freq0.countToFrequency(freq0.read()));
  }
  if (freq1.available()) {
    bike.setFWheelSpeed(freq1.countToFrequency(freq1.read()));
  }
  if (freq2.available()) {
    bike.setRWheelSpeed(freq2.countToFrequency(freq2.read()));
  }

  em7180.checkEventStatus();

  if (em7180.gotError()) {
    Serial.print("ERROR: ");
    Serial.println(em7180.getErrorString());
  }
  if (em7180.gotQuaternion()) {

    float qw, qx, qy, qz;

    em7180.readQuaternion(qw, qx, qy, qz);

    float roll = atan2(2.0f * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
    float pitch = -asin(2.0f * (qx * qz - qw * qy));
    float yaw = atan2(2.0f * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);

    pitch *= 180.0f / PI;
    yaw *= 180.0f / PI;
    yaw += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if (yaw < 0)
      yaw += 360.0f; // Ensure yaw stays between 0 and 360
    roll *= 180.0f / PI;

    bike.setQuaternion(yaw, pitch, roll);
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

  if (Serial1.available()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      gps.encode(c);
      if( rtcSet ) {
        gpsLine[gpsCount] = c;
        gpsCount ++;
        if ( c == '\n') {
          itemsGPSPerSecond ++;
          gpsLine[ gpsCount ] = 0;
  
          if( outputType == GPS || outputType == GPS_RC3 ) {
            Serial2.print( gpsLine );
          }
          if ( loggingEnabled && fileLogGPS && fileGPS ) {
            digitalWrite( INTERNAL_LED, HIGH );
            if( fileGPS.write( gpsLine, gpsCount ) == -1 ) {
              Serial.println( "=-=-=-=-=-=-=-=-=-=-=- ERROR Writting to GPS file =-=-=-=-=-=-=-=-=-=-=-" );
            }
            digitalWrite( INTERNAL_LED, LOW );
          }
          memset(gpsLine, 0, sizeof(gpsLine));
          gpsCount = 0;
        }
      }
    }
  }

  if ( !rtcSet && gps.time.isUpdated() && gps.time.age() < 1000 && gps.date.year() > 2000 ) {
    Serial.print( "Setting time to: " );
    Serial.print(  gps.date.year() );
    Serial.print( "/" );
    Serial.print( gps.date.month() );
    Serial.print( "/" );
    Serial.print( gps.date.day() );
    Serial.print( " " );
    Serial.print( gps.time.hour() );
    Serial.print( ":" );
    Serial.print( gps.time.minute() );
    Serial.print( ":" );
    Serial.println( gps.time.second() );

    setTime( gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year() );
    adjustTime( -14400 );
    rtcSet = true;
  }


  currentRC3SendTime = millis();
  if ( rtcSet && outputType == GPS_RC3 && currentRC3SendTime - lastRC3SendTime >= RC3_INTERVAL ) {
    lastRC3SendTime = currentRC3SendTime;
    itemsRC3PerSecond++;
    populateRC3Buffer();
    Serial2.print( rc3Buffer );
    if ( loggingEnabled && fileLogGPS && fileGPS ) {
      digitalWrite( INTERNAL_LED, HIGH );
      fileGPS.write( rc3Buffer);
      digitalWrite( INTERNAL_LED, LOW );
    }
  }

  
  currentBINSendTime = millis();
  if ((fileLogBIN ||  outputType == BINARY ) && currentBINSendTime - lastBINSendTime >= BIN_INTERVAL && bike.isDataAvailable()) {
    lastBINSendTime = currentBINSendTime;
    itemsBINPerSecond++;
    char *bin = NULL;
    size_t length = bike.encodeBINDirty(&bin);
    if( outputType == BINARY ) {
      Serial2.write(bin, length);
    }
    if( fileLogBIN && fileBIN ) {
      digitalWrite( INTERNAL_LED, HIGH );
      fileBIN.write( bin, length );
      digitalWrite( INTERNAL_LED, LOW );
    }
    free(bin);
    bike.dataSent();
  }

  currentDisplayTime = millis();
  if (currentDisplayTime - lastDisplayTime >= DISPLAY_INTERVAL ) {
    lastDisplayTime = currentDisplayTime;
    refreshDisplay();
  }

  //delay(1);
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

  bike.maxADC = 0.01 * adc->adc0->getMaxValue() ;

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
  bike.setFBrakeSwitch((digitalReadFast( DIGITAL_FBRAKE_PIN) == HIGH));
}

void digitalPinISR1(void) {
  bike.setNeutralSwitch((digitalReadFast( DIGITAL_NEUTRAL_PIN) == HIGH));
}



void refreshDisplay() {
  if ( !displayData ) {
    return;
  }
  Serial.println("========================================================================================================================");
  Serial.print("Logging Enabled:\t");
  Serial.print( ( loggingEnabled ) ? "Yes" : "No" );
  Serial.print("\tRTC Set:\t\t");
  Serial.println( ( rtcSet ) ? "Yes" : "No" );

  Serial.print("\nGPS File:\t\t");
  Serial.print(fileGPS);
  if ( fileGPS ) {
    char fileName[20];
    fileGPS.getName( fileName, 20 );
    Serial.print( "\t" );
    Serial.print( fileName );
  }
  Serial.print("\tRUN File:\t\t");
  Serial.print(fileBIN);
  if ( fileGPS ) {
    char fileName[20];
    fileBIN.getName( fileName, 20 );
    Serial.print( "\t" );
    Serial.print( fileName );
  }
  Serial.println();

  Serial.print("VOB File:\t\t");
  Serial.print(fileVOB);
  if ( fileVOB ) {
    char fileName[20];
    fileVOB.getName( fileName, 20 );
    Serial.print( "\t" );
    Serial.print( fileName );
  }
  Serial.println();

  Serial.print("\nMax ADC:\t\t");
  Serial.println(bike.maxADC,4);

  Serial.print("\nTPS:\t\t");
  Serial.print(bike.getTps());
  Serial.print("\tGear:\t\t");
  Serial.print(bike.getGearPercent());
  Serial.print("\tBrake:\t\t");
  Serial.print(bike.getFBrakePressure());

  Serial.print("F Suspension:\t");
  Serial.print(bike.getFSuspension());
  Serial.print("\tR Suspension:\t");
  Serial.print(bike.getRSuspension());
  Serial.print("\tO2 Sensor:\t");
  Serial.println(bike.getO2Sensor());

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

  Serial.print("\nYaw:\t\t");
  Serial.print(quaternions.yaw);
  Serial.print("\tPitch:\t\t");
  Serial.print(quaternions.pitch);
  Serial.print("\tRoll:\t\t");
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
  Serial.println(itemsBINPerSecond);
  Serial.print("RC3 per second:\t");
  Serial.print(itemsRC3PerSecond);
  Serial.print("\tVOB per second:\t");
  Serial.println(itemsVOBPerSecond);
  Serial.println();

  Serial.print("Date: ");
  Serial.print(month());
  Serial.print("/");
  Serial.print(day());
  Serial.print("/");
  Serial.print(year());
  Serial.print("  Time: ");
  Serial.print(hour() );
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());
  Serial.println();

  itemsGPSPerSecond = 0;
  itemsBINPerSecond = 0;
  itemsRC3PerSecond = 0;
  itemsVOBPerSecond = 0;
  gpsdump();
}

// simple function to scan for I2C devices on the bus
void I2Cscan() {
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Serial.print("Scanning ");
    Serial.println(address);
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
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

String getFileName( String ext ) {
  char fileName[20];
  memset(fileName, 0, sizeof(fileName));
  sprintf( fileName, "%04d%02d%02d-%02d%02d%02d.%s", year(), month(), day(), hour(), minute(), second(), ext.c_str() );
  return String( fileName );
}

void addVOBHeader( void ) {


}

void populateRC3Buffer( ) {
  char bufffer[200];
  if( rc3Counter == 65535 )
    rc3Counter = 0;   
  // $RC3,[time],[count],[xacc],[yacc],[zacc],[gyrox],[gyroy],[gyroz],rpm,front brake,front brake pressure,TPS,Front Suspension,Rear Suspension,Front Wheel RPM,Rear Wheel RPM,Gear,[a8],[a9],[a10],[a11],[a12],Roll,Pitch,Yaw*checksum

//  sprintf(bufffer, "RC3,%02d%02d%02d.%03d,%i", gps.time.hour(), gps.time.minute(), gps.time.second(), (gps.time.centisecond()*10) + gps.time.age(), rc3Counter );
  sprintf(bufffer,
            "RC3,%02d%02d%02d.%03u,%i,%+01.2f,%+01.2f,%+01.2f,%+01.2f,%+01.2f,%+01.2f,%05.0f,%i,%03.2f,%03.2f,%03.2f,%03.0f,%03.0f,%03.2f,0,0,0,0,0,0,%01.2f,%01.2f,%01.2f",
            gps.time.hour(), gps.time.minute(), gps.time.second(), (gps.time.centisecond()*10) + gps.time.age(), rc3Counter,
            bike.getAccelerometer().ax, bike.getAccelerometer().ay, bike.getAccelerometer().az,
            bike.getGyroscope().gx, bike.getGyroscope().gy, bike.getGyroscope().gz,
            bike.getEngineSpeed(), (bike.getFBrakeSwitch()) ? 1 : 0, bike.getTps(),
            bike.getFSuspension(), bike.getRSuspension(), bike.getFWheelSpeed(), bike.getRWheelSpeed(),bike.getGearPercent(),
            bike.getQuaternion().roll,bike.getQuaternion().pitch, bike.getQuaternion().yaw);

// below is experiment if my own conversion functions are faster than sprintf
//  strcpy( bufffer, "RC3," );
//  strcat( bufffer, itoa( gps.time.hour(), 2 ) );
//  strcat( bufffer, itoa( gps.time.minute(), 2 ) );
//  strcat( bufffer, itoa( gps.time.second(), 2 ) );
//  strcat( bufffer, "." );
//  strcat( bufffer, itoa( (gps.time.centisecond()*10) + gps.time.age(), 3 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, itoa( rc3Counter, 0 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getAccelerometer().ax, 1, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getAccelerometer().ay, 1, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getAccelerometer().az, 1, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getGyroscope().gx, 1, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getGyroscope().gy, 1, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getGyroscope().gz, 1, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, itoa(bike.getEngineSpeed(), 5 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, itoa(bike.getFBrakeSwitch(), 0 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getFBrakePressure(), 3, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getTps(), 3, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getFSuspension(), 3, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getRSuspension(), 3, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, itoa(bike.getFWheelSpeed(), 0 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, itoa(bike.getRWheelSpeed(), 0 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getGearPercent(), 3, 2 ) );
//  strcat( bufffer, ",0,0,0,0,0," );
//  strcat( bufffer, ftoa( bike.getQuaternion().roll, 1, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getQuaternion().pitch, 1, 2 ) );
//  strcat( bufffer, "," );
//  strcat( bufffer, ftoa( bike.getQuaternion().yaw, 1, 2 ) );
  
  char crc = getRC3CheckSum(bufffer); 
  sprintf(rc3Buffer, "$%s*%02hhX\r\n", bufffer, crc); 
  rc3Counter ++;
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

char *itoa ( int n, int padding) {
  int neg = 0;
  if( n < 0 ) {
    neg = 1;
    n = n * -1;
  }
    char s[32];
    static char rv[32];
    int i = 0, j,k;
// pop one decimal value at a time starting
// with the least significant
    while (n > 0) {
        s[i++] = '0' + n%10;
        n /= 10;
    }
// digits will be in reverse order
  int ss = ((padding - i) > 0 ) ? padding - i : 0;
  if( neg ) {
    rv[0] = '-';
  }
  for( k = 0; k < ( ss ); k++ ) {
    rv[neg+k] = '0';
  }
    for (j = 0; j < i; j++) {
    rv[neg+ss+j] = s[i-j-1];
  }
    rv[neg+ss+j] = '\0';
    return rv;
}
 
char *ftoa (float f, int padding, unsigned int places) {
// i is the integer portion
    unsigned int i = (int)f, digits = 0;
    static char rv[64];
    strcpy(rv, itoa(i, padding));
    strcat(rv, ".");
  if( f < 0 ) {
    f = f * -1;
    i = i * -1;
  }
 
// turn fraction into whole number
    float r = f - (float)i;
    i = 0;
    while (r > 0.0f) {
        r *= 10;
        i += (int)r;
        r -= (int)r;
    // do not use a "places" greater than 8 or i
    // could wrap around!
    // (this is enough to capture the precision of 
    // a float anyway)
        if (++digits == places) break;
        i *= 10;
    }
    if( i == 0 ) 
      strcat(rv, itoa( i, places ) );
    else
      strcat(rv, itoa(i,0));
    return rv;
};
