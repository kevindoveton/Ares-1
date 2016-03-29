// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-


#include "RTMath.h"

#include <Arduino.h>
#include <Wire.h>
#include <LSM303.h> // Compass
#include <L3G.h> // Gyro

//Sensor Library

#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
#define GYRO_SCALE 0.07f
#define betaDef    0.08f

#define compassXMax 216.0f
#define compassXMin -345.0f
#define compassYMax 210.0f
#define compassYMin -347.0f
#define compassZMax 249.0f
#define compassZMin -305.0f
#define inverseXRange (float)(2.0 / (compassXMax - compassXMin))
#define inverseYRange (float)(2.0 / (compassYMax - compassYMin))
#define inverseZRange (float)(2.0 / (compassZMax - compassZMin))

class Sensors {
public:
	bool init();
	
	//Read RTMath.h for the RTVector3 def, basically: (scalar, x, y, z).
	RTVector3 readSensors();
	RTVector3 readGyro();

	long timer, printTimer;

private:
	void IMUupdate(float *dt);
	void AHRSupdate(float *dt);
	
	float fastAtan2( float y, float x);
	float invSqrt(float number);
	void Smoothing(float *raw, float *smooth);

	LSM303 compass;
	L3G gyro;

	void readGyro(RTVector3& gyr);

	float G_Dt;
	int loopCount;

	float q0;
	float q1;
	float q2;
	float q3;
	float beta;
	float magnitude;
	float pitch,roll,yaw;
	float gyroSumX,gyroSumY,gyroSumZ;
	float offSetX,offSetY,offSetZ;
	float floatMagX,floatMagY,floatMagZ;
	float floatAccX,floatAccY,floatAccZ;
	float smoothAccX,smoothAccY,smoothAccZ;
	float accToFilterX,accToFilterY,accToFilterZ;
	int i;
	unsigned long lastDisplay;
	unsigned long lastRate;
	int sampleCount;
};

