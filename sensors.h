#include <I2Cdev.h>
#include <RTFusionRTQF.h>
#include <RTIMU.h>
#include <RTIMUBNO055.h>
#include <RTIMUGD20HM303D.h>
#include <RTIMUGD20HM303DLHC.h>
#include <RTIMUGD20M303DLHC.h>
#include <RTIMULibDefs.h>
#include <RTIMULSM9DS0.h>
#include <RTIMUMPU9150.h>
#include <RTIMUMPU9250.h>
#include <RTIMUSettings.h>
#include <RTMath.h>
#include <RTPressure.h>
#include <RTPressureBMP180.h>
#include <RTPressureDefs.h>
#include <RTPressureLPS25H.h>
#include <RTPressureMS5611.h>
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

private:
    //Define the sensor types in Arduino Libraries - RTIMU Constants
    RTIMU *imu;                                           // the IMU object
    RTPressure *pressure;                                 // the pressure object
    RTFusionRTQF fusion;                                  // the fusion object
    RTIMUSettings settings;                               // the settings object

    unsigned long lastDisplay;
    unsigned long lastRate;
    int sampleCount;
};
