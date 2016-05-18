// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
#include <Arduino.h>
#include <Wire.h>
#include "RTMath.h"
#include <LSM303.h> // Compass
#include <L3G.h> // Gyro

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

class Sensors {
public:
	bool init();

	//Read RTMath.h for the RTVector3 def, basically: (scalar, x, y, z).
	RTVector3 readSensors();
	RTVector3 readGyro();



private:
	int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer,
	//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

	L3G gyro;
	LSM303 compass;
	/*--------- I2C ---------*/
	void I2C_Init();
	void Gyro_Init();
	void Read_Gyro();
	void Accel_Init();
	void Read_Accel();
	void Read_Compass();

	/*------ Vectors ------*/
	float Vector_Dot_Product(float vector1[3],float vector2[3]);
	void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);
	void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);
	void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);

	/*-------- Matrixs --------*/
	void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3]);

	/*-------- General --------*/
	void Normalize(void);
	void Drift_correction(void);
	void Matrix_update(void);
	void Euler_angles(void);
	void Compass_Heading();

	float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

	long timer=0;   //general purpuse timer
	long timer_old;
	long timer24=0; //Second timer used to print values
	int AN[6]; //array that stores the gyro and accelerometer data
	int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

	int gyro_x;
	int gyro_y;
	int gyro_z;
	int accel_x;
	int accel_y;
	int accel_z;
	int magnetom_x;
	int magnetom_y;
	int magnetom_z;
	float c_magnetom_x;
	float c_magnetom_y;
	float c_magnetom_z;
	float MAG_Heading;

	float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
	float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
	float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
	float Omega_P[3]= {0,0,0};//Omega Proportional correction
	float Omega_I[3]= {0,0,0};//Omega Integrator
	float Omega[3]= {0,0,0};

	// Euler angles
	float roll;
	float pitch;
	float yaw;

	float errorRollPitch[3]= {0,0,0};
	float errorYaw[3]= {0,0,0};

	byte gyro_sat=0;

	float DCM_Matrix[3][3]= {
	  {
	    1,0,0  }
	  ,{
	    0,1,0  }
	  ,{
	    0,0,1  }
	};
	float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here

	float Temporary_Matrix[3][3]={
	  {
	    0,0,0  }
	  ,{
	    0,0,0  }
	  ,{
	    0,0,0  }
	};

};
