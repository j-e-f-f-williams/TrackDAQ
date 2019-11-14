#include "TrackDAQ.h"
#include <stdint.h>
/*
 * BikeData.h
 *
 *  Created on: Jan 10, 2019
 *      Author: jeff
 */

#ifndef BIKEDATA_H_
#define BIKEDATA_H_

#define BIKE_VERSION	1.0
#define	BOOT_VERSION	1.0
#define SERIAL_NUMBER	000001

#define MAX_VOLTAGE			5

#define NUM_ENGINE_SPEED	10
#define MAX_RPM				18000
#define MAX_RPM_DELTA		2000


/*
 * ADC0:
 * 		Analog 0 A2 [Brake]
 * 		Analog 1 A14 [F Sus]
 * 		Analog 2 A15[Gear]
 * ADC1:
 * 		Analog 4 A17[TPS]
 * 		Analog 5 A16 [R Sus]
 * 		Analog 6 A19[O2]
 */

struct QuaternionData {
	volatile float yaw;
	volatile float pitch;
	volatile float roll;
};

struct AccelerometerData {
	volatile float ax;
	volatile float ay;
	volatile float az;
};

struct GyroscopeData {
	volatile float gx;
	volatile float gy;
	volatile float gz;
};

struct MagnetometerData {
	volatile float mx;
	volatile float my;
	volatile float mz;
};

struct PressureData {
	volatile float pressure;
	volatile float temperature;
	volatile float altitude;
};


namespace TrackDAQ {

struct DirtyFlag {
	volatile bool dirty;
	volatile bool tps;
	volatile bool gear;
	volatile bool fSuspension;
	volatile bool rSuspension;
	volatile bool fBrakePressure;
	volatile bool o2Sensor;
	volatile bool fWheelSpeed;
	volatile bool rWheelSpeed;
	volatile bool engineSpeed;
	volatile bool fBrakeSwitch;
	volatile bool neutralSwitch;
	struct IMU {
		volatile bool quaternion;
		volatile bool accelerometer;
		volatile bool gyroscope;
		volatile bool magnetometer;
		volatile bool pressure;
	} imu;
};

struct IMUData {
	struct QuaternionData quaternion;
	struct AccelerometerData accelerometer;
	struct GyroscopeData gyro;
	struct MagnetometerData magnetometer;
	struct PressureData pressure;
};


class CheckSum {
public:
	CheckSum( FILE*);
	char calculate(void);
	void add( char );

private:
	FILE* output;
	int sum;
};

char getByte( long , int );

class BikeData {
public:
	BikeData();
	virtual ~BikeData();
	float maxADC;
	bool isDataAvailable();
	void dataSent( void );
	size_t encodeBINDirty( char ** );
  size_t encodeLoggerDetails( char ** );

 
    void setPitchRollStart( float, float );
    bool hasPitchRollStart( );
    float getRollStart( );
    float getPitchStart( );
    void setQuaternion( float, float, float );
	struct QuaternionData getQuaternion( void );
	void setAccelerometer( float, float, float );
	struct AccelerometerData getAccelerometer( void );
	void setGyroscope( float, float, float );
	struct GyroscopeData getGyroscope( void );
	void setMagnetometer( float, float, float );
	struct MagnetometerData getMagnetometer( void );
	void setPressure( float, float, float );
	struct PressureData getPressure( void );

	float getTps( void );
	uint16_t getTpsRaw( void );
	void setTps( uint16_t );
	int getGearNumber( void );
	int getGearNumber( int );
	float getGearPercent( void );
	uint16_t getGearRaw( void );
	void setGear( uint16_t );
	float getFSuspension( void );
	uint16_t getFSuspensionRaw( void );
	void setFSuspension( uint16_t );
	float getRSuspension( void );
	uint16_t getRSuspensionRaw( void );
	void setRSuspension( uint16_t );
	float getFBrakePressure( void );
	uint16_t getFBrakePressureRaw( void );
	void setFBrakePressure( uint16_t );
	float getO2Sensor( void );
	uint16_t getO2SensorRaw( void );
	void setO2Sensor( uint16_t );

	float getFWheelSpeed( void );
	void setFWheelSpeed( float );
	float getRWheelSpeed( void );
	void setRWheelSpeed( float );
	float getEngineSpeed( void );
	void setEngineSpeed( float );

	bool getFBrakeSwitch( void );
	void setFBrakeSwitch( bool );
	bool getNeutralSwitch( void );
	void setNeutralSwitch( bool );

private:
	struct DirtyFlag dirtyFields;
	volatile uint16_t tps = 0;
	volatile uint16_t gear = 0;
	volatile uint16_t fSuspension = 0;
	volatile uint16_t rSuspension = 0;
	volatile uint16_t fBrakePressure = 0;
	volatile uint16_t o2Sensor = 0;

	float fWheelSpeed = 0.0;
	float rWheelSpeed = 0.0;
	float engineSpeed = 0.0;
	float engineSpeeds[ NUM_ENGINE_SPEED ];
	double engineSpeedsTotal = 0.0;
	int engineSpeedsCount = 0;
	bool startPitchRollSet = false;
	float startPitch = 0;
	float startRoll = 0;

	volatile bool fBrakeSwitch = false;
	volatile bool neutralSwitch = false;

	struct IMUData imu;

	unsigned long timeStamp = 0;

	void resetDirty( bool );
	void encodeAnalog( FILE *, int, float );
	void encodeFrequency( FILE *,int, float );
	void encodeTimeStamp( FILE * );
	void encodeAccelerations( FILE * );
	void encodeLoggerStorage( FILE * );
	void encodeBeaconPulse( FILE * );
	void encodeGPSLocation( FILE * );
	void encodeGPSTime( FILE * );
	void encodeGPSSpeed( FILE * );
	void encodeGPSCourse( FILE * );
	void encodeGPSAltitude( FILE * );
	bool setRPMHeader = false;
	void logRPM( String, float );
};

} /* namespace TrackDAQ */

#endif /* BIKEDATA_H_ */
