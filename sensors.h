#include <LSM303.h>
#include <L3G.h>


//#include "RTFusionRTQF.h"
#include "RTMath.h"

#include <Arduino.h>
#include <Wire.h>

#define RTARDULINK_MODE


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
//	RTVector3 readAcel();
//	RTVector3 readCompass();
  long timer, printTimer;

private:
//	RTFusionRTQF fusion;                                  // the fusion object
  void IMUupdate(float *dt);
  void AHRSupdate(float *dt);
//  void GetEuler(void);
  float fastAtan2( float y, float x);
  float invSqrt(float number);
  void Smoothing(float *raw, float *smooth);
  
	LSM303 compass;
	L3G gyro;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
//  const float slerp = 0.9;

	void readGyro(RTVector3& gyr);
//	void readCompass(RTVector3& acel, RTVector3& comp);
  
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

