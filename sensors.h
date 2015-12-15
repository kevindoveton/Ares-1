#include <LSM303.h>
#include <L3G.h>
#include <SFE_BMP180.h>


#include "RTFusionRTQF.h"
#include "RTMath.h"

#include <Arduino.h>
#include <Wire.h>

#define RTARDULINK_MODE


//Sensor Library
//https://github.com/richards-tech/RTIMULib-Arduino


class Sensors {
public:
	void init();
	//Read RTMath.h for the RTVector3 def, basically: (scalar, x, y, z).
	RTVector3 readSensors();
	RTVector3 readGyro();
	RTVector3 readAcel();
	RTVector3 readCompass();

private:
	RTFusionRTQF fusion;                                  // the fusion object

	LSM303 compass;
	L3G gyro;

	void readGyro(RTVector3& gyr);
	void readCompass(RTVector3& acel, RTVector3& comp);


	unsigned long lastDisplay;
	unsigned long lastRate;
	int sampleCount;
};
