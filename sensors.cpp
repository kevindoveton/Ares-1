#include "sensors.h"

#define DISPLAY_INTERVAL  300                         // interval between pose displays



void Sensors :: init()
{

    #ifdef DEBUG
        Serial.println("Sensors :: init()");
	#endif

    int errcode;

    Serial.begin(9600);
    Wire.begin();
    imu = RTIMU::createIMU(&settings);                        // create the imu object
    pressure = RTPressure::createPressure(&settings);         // create the pressure sensor
    
    if (pressure == 0) {
    	Serial.println("No pressure sensor has been configured - terminating"); 
    	while (1) ;
    }
    
    Serial.print("ArduinoIMU10 starting using IMU "); Serial.print(imu->IMUName());
    Serial.print(", pressure sensor "); Serial.println(pressure->pressureName());
    if ((errcode = imu->IMUInit()) < 0) {
    	Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }

    if ((errcode = pressure->pressureInit()) < 0) {
    	Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);
    }

    if (imu->getCalibrationValid())
    	Serial.println("Using compass calibration");
    else
    	Serial.println("No valid compass calibration data");

    lastDisplay = lastRate = millis();
    sampleCount = 0;

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.02);

    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
}

//Not sure how or if this will work...
RTVector3 Sensors :: readSensors() 
{
	unsigned long now = millis();
    unsigned long delta;
    float latestPressure;
    float latestTemperature;
    int loopCount = 1;

    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;

        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
        sampleCount++;
        if ((delta = now - lastRate) >= 1000) {
            Serial.print("Sample rate: "); Serial.print(sampleCount);
            if (imu->IMUGyroBiasValid())
                Serial.println(", gyro bias valid");
            else
                Serial.println(", calculating gyro bias");

            sampleCount = 0;
            lastRate = now;
        }
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
            // RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
            return (RTVector3&)fusion.getFusionPose();
            
            // if (pressure->pressureRead(latestPressure, latestTemperature)) {
            //     Serial.print(", pressure: "); Serial.print(latestPressure);
            //     Serial.print(", temperature: "); Serial.print(latestTemperature);
            // }
            // Serial.println();
        }
    }
}


RTVector3 Sensors :: readGyro() 
{
  unsigned long now = millis();
  unsigned long delta;
  float latestPressure;
  float latestTemperature;
  int loopCount = 1;
  
    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;

        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
        sampleCount++;
        if ((delta = now - lastRate) >= 1000) {
            Serial.print("Sample rate: "); Serial.print(sampleCount);
            if (imu->IMUGyroBiasValid())
                Serial.println(", gyro bias valid");
            else
                Serial.println(", calculating gyro bias");

            sampleCount = 0;
            lastRate = now;
        }
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
//          RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
//          RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
//          RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
            // RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
            return (RTVector3&)imu->getGyro();

            // if (pressure->pressureRead(latestPressure, latestTemperature)) {
            //     Serial.print(", pressure: "); Serial.print(latestPressure);
            //     Serial.print(", temperature: "); Serial.print(latestTemperature);
            // }
            // Serial.println();
}
}
}

