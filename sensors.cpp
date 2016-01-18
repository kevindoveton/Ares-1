#include "sensors.h"

#define DISPLAY_INTERVAL  300                         // interval between pose displays



bool Sensors :: init()
{

	#ifdef DEBUG
		Serial.println("Sensors :: init()");
	#endif

	Wire.begin();

	// Compass
	if (!compass.init())
    return false;
	compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-3456, -4294, -3141};
  compass.m_max = (LSM303::vector<int16_t>){+3333, +2685, +3502};

	// Gyro
	if (!gyro.init())
    return false;
    
	gyro.enableDefault();
	
	lastDisplay = lastRate = millis();
	sampleCount = 0;

	// Slerp power controls the fusion and can be between 0 and 1
	// 0 means that only gyros are used, 1 means that only accels/compass are used
	// In-between gives the fusion mix.
	
	fusion.setSlerpPower(slerp);

	// use of sensors in the fusion algorithm can be controlled here
	// change any of these to false to disable that sensor
	
	fusion.setGyroEnable(true);
	fusion.setAccelEnable(true);
	fusion.setCompassEnable(true);
  return true;
}

//Not sure how or if this will work...
RTVector3 Sensors :: readSensors() 
{/*
	unsigned long now = millis();
	unsigned long delta;
	float latestPressure;
	float latestTemperature;
	int loopCount = 1;

	//while (1) { // imu->read()
    #if DEBUG
		  Serial.println("readSensors()");
    #endif
		RTVector3 _comp;
		RTVector3 _acel;
		RTVector3 _gyro;

		readGyro(_gyro);
		readCompass(_acel, _comp);

		//if (++loopCount >= 10)
			//continue;

		fusion.newIMUData(_gyro, _acel, _comp, millis()); //gyro, acel, comp, timestamp

		sampleCount++;
		if ((delta = now - lastRate) >= 1000) {
//			Serial.print("Sample rate: "); Serial.print(sampleCount);
			
			sampleCount = 0;
			lastRate = now;
		}

		if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
			lastDisplay = now;
			// RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
			return (RTVector3&)fusion.getFusionPose();
		}*/

   //this runs in 4ms on the MEGA 2560
    G_Dt = (micros() - timer)/1000000.0;
    timer=micros();
    compass.read();
    floatMagX = ((float)compass.m.x - compassXMin) * inverseXRange - 1.0;
    floatMagY = ((float)compass.m.y - compassYMin) * inverseYRange - 1.0;
    floatMagZ = ((float)compass.m.z - compassZMin) * inverseZRange - 1.0;
    floatAccX = (float) compass.a.x;
    floatAccY = (float) compass.a.y;
    floatAccZ = (float) compass.a.z;
    Smoothing(&floatAccX,&smoothAccX);
    Smoothing(&floatAccY,&smoothAccY);
    Smoothing(&floatAccZ,&smoothAccZ);
    accToFilterX = smoothAccX;
    accToFilterY = smoothAccY;
    accToFilterZ = smoothAccZ;
    gyro.read();
    AHRSupdate(&G_Dt);
}

RTVector3 Sensors :: readGyro()
{
	RTVector3 out;
	readGyro(out);
	return out;
}

RTVector3 Sensors :: readAcel()
{
	RTVector3 acel;
	RTVector3 comp;
	readCompass(acel, comp);
	return acel;
}

RTVector3 Sensors :: readCompass()
{
	RTVector3 acel;
	RTVector3 comp;
	readCompass(acel, comp);
	return comp;
}

void Sensors :: readGyro (RTVector3& gyr)
{
	gyro.read();
	gyr.setX(gyro.g.x);
	gyr.setY(gyro.g.y);
	gyr.setZ(gyro.g.z);
}

void Sensors :: readCompass(RTVector3& acel, RTVector3& comp)
{
	compass.read();
	
	acel.setX(compass.a.x);
	acel.setY(compass.a.y);
	acel.setZ(compass.a.z);

	comp.setX(compass.m.x);
	comp.setY(compass.m.y);
	comp.setZ(compass.m.z);
}


