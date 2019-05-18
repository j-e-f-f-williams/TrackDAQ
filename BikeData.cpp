/*
 * BikeData.cpp
 *
 *  Created on: Jan 10, 2019
 *      Author: jeff
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "TinyGPS++.h"

#include "BikeData.h"

extern TinyGPSPlus gps;

namespace TrackDAQ {

CheckSum::CheckSum(FILE* output) {
	CheckSum::output = output;
	CheckSum::sum = 0;
}

void CheckSum::add(char ch) {
	putc(ch, CheckSum::output);
	sum = sum + ch;
}

char CheckSum::calculate(void) {
	char checkSum = CheckSum::sum & 0xFF;
	putc(checkSum, CheckSum::output);
	return checkSum;
}

BikeData::BikeData() {
	BikeData::resetDirty(false);
}

BikeData::~BikeData() {

}

bool BikeData::isDataAvailable() {
	return BikeData::dirtyFields.dirty;
}

void BikeData::dataSent(void) {
	BikeData::resetDirty(false);
}

size_t BikeData::encodeLoggerDetails(char ** buffer) {
	size_t bufferSize = 0;
	FILE* output = open_memstream(buffer, &bufferSize);

	int serial = SERIAL_NUMBER;
	float version = BIKE_VERSION;
	float bootloader = BOOT_VERSION;

	CheckSum checksum = CheckSum(output);
	checksum.add(6);
	checksum.add(getByte(serial, 2));
	checksum.add(getByte(serial, 1));
	checksum.add(getByte(version, 1));
	checksum.add(getByte(bootloader, 1));
	checksum.calculate();
	fclose(output);
	return bufferSize;
}

size_t BikeData::encodeBINDirty(char ** buffer) {
	size_t bufferSize = 0;

	FILE* output = open_memstream(buffer, &bufferSize);

	encodeTimeStamp(output);
	if (BikeData::dirtyFields.tps) {
		BikeData::encodeAnalog(output, 20, BikeData::getTps());
	}
	if (BikeData::dirtyFields.gear) {
		BikeData::encodeAnalog(output, 21, BikeData::getGearPercent());
	}
	if (BikeData::dirtyFields.gear) {
		int iValue = int(BikeData::getGearNumber());
		CheckSum checksum = CheckSum(output);
		checksum.add(22);
		checksum.add(getByte(iValue, 2));
		checksum.add(getByte(iValue, 1));
		checksum.calculate();
	}
	if (BikeData::dirtyFields.fBrakePressure) {
		BikeData::encodeAnalog(output, 23, BikeData::getFBrakePressure());
	}
	if (BikeData::dirtyFields.fSuspension) {
		BikeData::encodeAnalog(output, 24, BikeData::getFSuspension());
	}
	if (BikeData::dirtyFields.rSuspension) {
		BikeData::encodeAnalog(output, 25, BikeData::getRSuspension());
	}
	if (BikeData::dirtyFields.o2Sensor) {
		BikeData::encodeAnalog(output, 26, BikeData::getO2Sensor());
	}
	if (BikeData::dirtyFields.neutralSwitch) {
		BikeData::encodeAnalog(output, 30,
				(BikeData::getNeutralSwitch() ? float( MAX_VOLTAGE) : 0.0));
	}
	if (BikeData::dirtyFields.fBrakeSwitch) {
		BikeData::encodeAnalog(output, 31,
				(BikeData::getFBrakeSwitch() ? float( MAX_VOLTAGE) : 0.0));
	}
	if (gps.location.isUpdated() && gps.location.isValid()) {
		BikeData::encodeGPSLocation(output);
	}
	if (gps.time.isUpdated() && gps.time.isValid()) {
		BikeData::encodeGPSTime(output);
	}
	if (gps.speed.isUpdated() && gps.speed.isValid()) {
		BikeData::encodeGPSSpeed(output);
	}
	if (gps.course.isUpdated() && gps.course.isValid()) {
		BikeData::encodeGPSCourse(output);
	}
	if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
		BikeData::encodeGPSAltitude(output);
	}
	if (BikeData::dirtyFields.imu.accelerometer) {
		BikeData::encodeAccelerations(output);
	}
	if (BikeData::dirtyFields.imu.gyroscope) {

	}
	if (BikeData::dirtyFields.imu.quaternion) {
		struct QuaternionData quaternions = getQuaternion();

		int roll =
				(quaternions.roll < 0) ?
						quaternions.roll * -100 : quaternions.roll * 100;
		int pitch =
				(quaternions.pitch < 0) ?
						quaternions.pitch * -100 : quaternions.pitch * 100;
		int yaw =
				(quaternions.yaw < 0) ?
						quaternions.yaw * -100 : quaternions.yaw * 100;
		CheckSum checksum = CheckSum(output);
		checksum.add(84);
		checksum.add(
				(quaternions.roll < 0) ?
						(getByte(roll, 2) & 0x7F) | 0x80 :
						getByte(roll, 2) & 0x7F);
		checksum.add(getByte(roll, 1));
		checksum.add((0 & 0x7F) | 0x80);
		checksum.calculate();
		checksum = CheckSum(output);
		checksum.add(82);
		checksum.add(
				(quaternions.pitch < 0) ?
						(getByte(pitch, 2) & 0x7F) | 0x80 :
						getByte(pitch, 2) & 0x7F);
		checksum.add(getByte(pitch, 1));
		checksum.add((0 & 0x7F) | 0x80);
		checksum.calculate();
		checksum = CheckSum(output);
		checksum.add(80);
		checksum.add(
				(quaternions.yaw < 0) ?
						(getByte(yaw, 2) & 0x7F) | 0x80 :
						getByte(yaw, 2) & 0x7F);
		checksum.add(getByte(yaw, 1));
		checksum.add((0 & 0x7F) | 0x80);
		checksum.calculate();
	}
	fclose(output);
	return bufferSize;
}

void BikeData::encodeAnalog(FILE* output, int channel, float value) {
	int iValue = int(((value * MAX_VOLTAGE) * 1000.0));
	CheckSum checksum = CheckSum(output);
	checksum.add(channel);
	checksum.add(getByte(iValue, 2));
	checksum.add(getByte(iValue, 1));
	checksum.calculate();
}

void BikeData::encodeGPSLocation(FILE *output) {
	CheckSum checksum = CheckSum(output);
	checksum.add(10);
	long lon = gps.location.lng() * 10000000;
	long lat = gps.location.lat() * 10000000;
	checksum.add((lon < 0) ? getByte(lon, 4) | 0x80 : getByte(lon, 4));
	checksum.add(getByte(lon, 3));
	checksum.add(getByte(lon, 2));
	checksum.add(getByte(lon, 1));
	checksum.add((lat < 0) ? getByte(lat, 4) | 0x80 : getByte(lat, 4));
	checksum.add(getByte(lat, 3));
	checksum.add(getByte(lat, 2));
	checksum.add(getByte(lat, 1));
	long hdop = 3000 * gps.hdop.hdop();
	checksum.add(getByte(hdop, 4));
	checksum.add(getByte(hdop, 3));
	checksum.add(getByte(hdop, 2));
	checksum.add(getByte(hdop, 1));
	checksum.calculate();
}

void BikeData::encodeGPSTime(FILE *output) {

}

void BikeData::encodeGPSSpeed(FILE *output) {
	long speed = gps.speed.mps() * 100;
	CheckSum checksum = CheckSum(output);
	checksum.add(11);
	checksum.add(getByte(speed, 4));
	checksum.add(getByte(speed, 3));
	checksum.add(getByte(speed, 2));
	checksum.add(getByte(speed, 1));
	checksum.add(0);
	checksum.add(0);
	checksum.add(0);
	checksum.add(0);
	checksum.calculate();
}

void BikeData::encodeGPSCourse(FILE *output) {
	long course = gps.course.deg() * 100000;
	CheckSum checksum = CheckSum(output);
	checksum.add(56);
	checksum.add(getByte(course, 4));
	checksum.add(getByte(course, 3));
	checksum.add(getByte(course, 2));
	checksum.add(getByte(course, 1));
	checksum.add(0);
	checksum.add(0);
	checksum.add(0);
	checksum.add(0);
	checksum.calculate();

}

void BikeData::encodeGPSAltitude(FILE *output) {

}

void BikeData::encodeFrequency(FILE* output, int channel, float value) {
}

void BikeData::encodeTimeStamp(FILE* output) {
	CheckSum checksum = CheckSum(output);
	checksum.add(9);
	checksum.add(BikeData::timeStamp >> 16);
	checksum.add(BikeData::timeStamp >> 8);
	checksum.add(BikeData::timeStamp & 0xFF);
	checksum.calculate();
	BikeData::timeStamp++;
}
void BikeData::encodeAccelerations(FILE* output) {
	struct AccelerometerData accelerometers = getAccelerometer();
	int ax =
			(accelerometers.ax < 0) ?
					accelerometers.ax * -100 : accelerometers.ax * 100;
	int ay =
			(accelerometers.ay < 0) ?
					accelerometers.ay * -100 : accelerometers.ay * 100;
	CheckSum checksum = CheckSum(output);
	checksum.add(8);
	checksum.add(
			(accelerometers.ax < 0) ? getByte(ax, 2) | 0x80 : getByte(ax, 2));
	checksum.add(getByte(ax, 1));
	checksum.add(
			(accelerometers.ay < 0) ? getByte(ay, 2) | 0x80 : getByte(ay, 2));
	checksum.add(getByte(ay, 1));
	checksum.calculate();
}

void BikeData::encodeBeaconPulse(FILE* output) {

}

void BikeData::resetDirty(bool value) {
	BikeData::dirtyFields.dirty = value;
	BikeData::dirtyFields.tps = value;
	BikeData::dirtyFields.gear = value;
	BikeData::dirtyFields.fSuspension = value;
	BikeData::dirtyFields.rSuspension = value;
	BikeData::dirtyFields.fBrakePressure = value;
	BikeData::dirtyFields.o2Sensor = value;
	BikeData::dirtyFields.fWheelSpeed = value;
	BikeData::dirtyFields.rWheelSpeed = value;
	BikeData::dirtyFields.engineSpeed = value;
	BikeData::dirtyFields.fBrakeSwitch = value;
	BikeData::dirtyFields.neutralSwitch = value;
	BikeData::dirtyFields.imu.quaternion = value;
	BikeData::dirtyFields.imu.accelerometer = value;
	BikeData::dirtyFields.imu.gyroscope = value;
	BikeData::dirtyFields.imu.magnetometer = value;
	BikeData::dirtyFields.imu.pressure = value;
}

void BikeData::setAccelerometer(float ax, float ay, float az) {
	if (BikeData::imu.accelerometer.ax != ax
			|| BikeData::imu.accelerometer.az != az
			|| BikeData::imu.accelerometer.az != az) {
		BikeData::imu.accelerometer.ax = ax;
		BikeData::imu.accelerometer.az = az;
		BikeData::imu.accelerometer.az = az;
		BikeData::dirtyFields.imu.accelerometer = true;
	}
}

struct AccelerometerData BikeData::getAccelerometer(void) {
	return BikeData::imu.accelerometer;
}

void BikeData::setGyroscope(float gx, float gy, float gz) {
	if (BikeData::imu.gyro.gx != gx || BikeData::imu.gyro.gy != gy
			|| BikeData::imu.gyro.gz != gz) {
		BikeData::imu.gyro.gx = gx;
		BikeData::imu.gyro.gy = gy;
		BikeData::imu.gyro.gz = gz;
		BikeData::dirtyFields.imu.gyroscope = true;
	}
}

struct GyroscopeData BikeData::getGyroscope(void) {
	return BikeData::imu.gyro;
}

void BikeData::setMagnetometer(float mx, float my, float mz) {
	if (BikeData::imu.magnetometer.mx != mx
			|| BikeData::imu.magnetometer.my != my
			|| BikeData::imu.magnetometer.mz != mz) {
		BikeData::imu.magnetometer.mx = mx;
		BikeData::imu.magnetometer.my = my;
		BikeData::imu.magnetometer.mz = mz;
		BikeData::dirtyFields.imu.magnetometer = true;
	}
}

struct MagnetometerData BikeData::getMagnetometer(void) {
	return BikeData::imu.magnetometer;
}

void BikeData::setPressure(float temperature, float pressure, float altitude) {
	if (BikeData::imu.pressure.temperature != temperature
			|| BikeData::imu.pressure.pressure != pressure
			|| BikeData::imu.pressure.altitude != altitude) {
		BikeData::imu.pressure.temperature = temperature;
		BikeData::imu.pressure.pressure = pressure;
		BikeData::imu.pressure.altitude = altitude;
		BikeData::dirtyFields.imu.pressure = true;
	}
}

struct PressureData BikeData::getPressure(void) {
	return BikeData::imu.pressure;
}

void BikeData::setQuaternion(float yaw, float pitch, float roll) {
	if (BikeData::imu.quaternion.yaw != yaw
			|| BikeData::imu.quaternion.pitch != pitch
			|| BikeData::imu.quaternion.roll != roll) {
		BikeData::imu.quaternion.yaw = yaw;
		BikeData::imu.quaternion.pitch = pitch;
		BikeData::imu.quaternion.roll = roll;
		BikeData::dirtyFields.imu.quaternion = true;
		BikeData::dirtyFields.dirty = true;
	}
}

struct QuaternionData BikeData::getQuaternion(void) {
	return BikeData::imu.quaternion;
}

float BikeData::getTps(void) {
	return BikeData::tps / BikeData::maxADC;
}
uint16_t BikeData::getTpsRaw(void) {
	return BikeData::tps;
}
void BikeData::setTps(uint16_t value) {
	if (BikeData::tps != value) {
		BikeData::tps = value;
		BikeData::dirtyFields.tps = true;
	}
}
float BikeData::getGearPercent(void) {
	return BikeData::gear / BikeData::maxADC;
}

int BikeData::getGearNumber(void) {
	if (BikeData::getNeutralSwitch())
		return 0;
	return BikeData::getGearRaw();
}

uint16_t BikeData::getGearRaw(void) {
	return BikeData::gear;
}
void BikeData::setGear(uint16_t value) {
	if (BikeData::gear != value) {
		BikeData::gear = value;
		BikeData::dirtyFields.gear = true;
	}
}
float BikeData::getFSuspension(void) {
	return BikeData::fSuspension / BikeData::maxADC;
}
uint16_t BikeData::getFSuspensionRaw(void) {
	return BikeData::fSuspension;
}
void BikeData::setFSuspension(uint16_t value) {
	if (BikeData::fSuspension != value) {
		BikeData::fSuspension = value;
		BikeData::dirtyFields.fSuspension = true;
	}
}
float BikeData::getRSuspension(void) {
	return BikeData::rSuspension / BikeData::maxADC;
}
uint16_t BikeData::getRSuspensionRaw(void) {
	return BikeData::rSuspension;
}

void BikeData::setRSuspension(uint16_t value) {
	if (BikeData::rSuspension != value) {
		BikeData::rSuspension = value;
		BikeData::dirtyFields.rSuspension = true;
	}
}
float BikeData::getFBrakePressure(void) {
	return BikeData::fBrakePressure / BikeData::maxADC;
}

uint16_t BikeData::getFBrakePressureRaw(void) {
	return BikeData::fBrakePressure;
}

void BikeData::setFBrakePressure(uint16_t value) {
	if (BikeData::fBrakePressure != value) {
		BikeData::fBrakePressure = value;
		BikeData::dirtyFields.fBrakePressure = true;
	}
}
float BikeData::getO2Sensor(void) {
	return BikeData::o2Sensor / BikeData::maxADC;
}
uint16_t BikeData::getO2SensorRaw(void) {
	return BikeData::o2Sensor;
}
void BikeData::setO2Sensor(uint16_t value) {
	if (BikeData::o2Sensor != value) {
		BikeData::o2Sensor = value;
		BikeData::dirtyFields.o2Sensor = true;
	}
}

float BikeData::getFWheelSpeed(void) {
	return BikeData::fWheelSpeed;
}
void BikeData::setFWheelSpeed(float value) {
	if (BikeData::fWheelSpeed != value) {
		BikeData::fWheelSpeed = value;
		BikeData::dirtyFields.fWheelSpeed = true;
	}
}
float BikeData::getRWheelSpeed(void) {
	return BikeData::rWheelSpeed;
}
void BikeData::setRWheelSpeed(float value) {
	if (BikeData::rWheelSpeed != value) {
		BikeData::rWheelSpeed = value;
		BikeData::dirtyFields.rWheelSpeed = true;
	}
}
float BikeData::getEngineSpeed(void) {
	return BikeData::engineSpeed;
}
void BikeData::setEngineSpeed(float value) {
	if (BikeData::engineSpeed != value) {
		BikeData::engineSpeed = value;
		BikeData::dirtyFields.engineSpeed = true;
		BikeData::dirtyFields.dirty = true;
	}
}

bool BikeData::getFBrakeSwitch(void) {
	return BikeData::fBrakeSwitch;
}
void BikeData::setFBrakeSwitch(bool value) {
	if (BikeData::fBrakeSwitch != value) {
		BikeData::fBrakeSwitch = value;
		BikeData::dirtyFields.fBrakeSwitch = true;
	}
}
bool BikeData::getNeutralSwitch(void) {
	return BikeData::neutralSwitch;
}
void BikeData::setNeutralSwitch(bool value) {
	if (BikeData::neutralSwitch != value) {
		BikeData::neutralSwitch = value;
		BikeData::dirtyFields.neutralSwitch = true;
	}
}

char getByte(long value, int byte) {
	return (value >> (8 * (byte - 1))) & 0xFF;
}

} /* namespace TrackDAQ */
