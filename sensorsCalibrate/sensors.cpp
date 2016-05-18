// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
#include "sensors.h"


// -------------------------------------------------
// -------------  Public Functions  ----------------
// -------------------------------------------------
bool Sensors :: init()
{
	I2C_Init();
	delay(1500);
	Accel_Init(); // this also inits compass
	Gyro_Init();

	delay(20);

	for(int i=0;i<32;i++)    // We take some readings...
	{
		Read_Gyro();
		Read_Accel();

		for(int y=0; y<6; y++)  // Cumulate values
			AN_OFFSET[y] += AN[y];

		delay(20);
	}

	for(int y=0; y<6; y++)
		AN_OFFSET[y] = AN_OFFSET[y]/32;


	AN_OFFSET[5] -= GRAVITY*SENSOR_SIGN[5];

	timer=millis();
	delay(20);

	return true;
}
// Return a Fused RTVector3 of sensors
RTVector3 Sensors :: readSensors()
{
	timer_old = timer;
	timer = millis();
	if (timer>timer_old)
	{
		G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
		if (G_Dt > 0.2)
			G_Dt = 0; // ignore integration times over 200 ms
	}
	else
		G_Dt = 0;

	// *** DCM algorithm
	// Data adquisition
	Read_Gyro();   // This read gyro data
	Read_Accel();     // Read I2C accelerometer

	Read_Compass();    // Read I2C magnetometer
	Compass_Heading(); // Calculate magnetic heading

	// Calculations...
	Matrix_update();
	Normalize();
	Drift_correction();
	Euler_angles();
	Serial.println(ToDeg(pitch));
	return RTVector3(ToDeg(pitch), ToDeg(roll), ToDeg(yaw));
}

RTVector3 Sensors :: readGyro()
{
	Read_Gyro();
	return RTVector3(gyro_x, gyro_y, gyro_z);
}

// -------------------------------------------------
// --------------------  I2C  ----------------------
// -------------------------------------------------

void Sensors :: I2C_Init()
{
  Wire.begin();
}

void Sensors :: Gyro_Init()
{
  gyro.init();
  gyro.enableDefault();
  gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz

}

void Sensors :: Read_Gyro()
{
  gyro.read();

  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;

  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Sensors :: Accel_Init()
{
	compass.init();
	compass.enableDefault();

	switch (compass.getDeviceType())
	{
		case LSM303::device_D:
			compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
			break;

		case LSM303::device_DLHC:
			compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
			break;

		default: // DLM, DLH
			compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
	}

}

// Reads x,y and z accelerometer registers
void Sensors :: Read_Accel()
{
	compass.readAcc();

	AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
	AN[4] = compass.a.y >> 4;
	AN[5] = compass.a.z >> 4;

	accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
	accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
	accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}



void Sensors :: Read_Compass()
{
	compass.readMag();
	magnetom_x = SENSOR_SIGN[6] * compass.m.x;
	magnetom_y = SENSOR_SIGN[7] * compass.m.y;
	magnetom_z = SENSOR_SIGN[8] * compass.m.z;
}


// -------------------------------------------------
// ------------------  Vectors  --------------------
// -------------------------------------------------

//Computes the dot product of two vectors
float Sensors :: Vector_Dot_Product(float vector1[3],float vector2[3])
{
	float op=0;

	for(int c=0; c<3; c++)
		op+=vector1[c]*vector2[c];

	return op;
}

//Computes the cross product of two vectors
void Sensors :: Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
	vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
	vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
	vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar.
void Sensors :: Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
	for(int c=0; c<3; c++)
		vectorOut[c]=vectorIn[c]*scale2;
}

void Sensors :: Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
	for(int c=0; c<3; c++)
		vectorOut[c]=vectorIn1[c]+vectorIn2[c];
}


// -------------------------------------------------
// -----------------  Matrixs  --------------------
// -------------------------------------------------
//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!).
void Sensors :: Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3])
{
	for(int x = 0; x < 3; x++)
	{
		for(int y = 0; y < 3; y++)
		{
			mat[x][y] = 0;
			for(int w = 0; w < 3; w++)
			{
				mat[x][y] += a[x][w] * b[w][y];
			}
		}
	}
}

void Sensors :: Normalize(void)
{
	float error=0;
	float temporary[3][3];
	float renorm=0;

	error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

	Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
	Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

	Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
	Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

	Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

	renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
	Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

	renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
	Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

	renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
	Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Sensors :: Drift_correction(void)
{
	float mag_heading_x;
	float mag_heading_y;
	float errorCourse;
	//Compensation the Roll, Pitch and Yaw drift.
	static float Scaled_Omega_P[3];
	static float Scaled_Omega_I[3];
	float Accel_magnitude;
	float Accel_weight;


	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
	Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //

	Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
	Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);

	Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
	Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);

	//*****YAW***************
	// We make the gyro YAW drift correction based on compass magnetic heading

	mag_heading_x = cos(MAG_Heading);
	mag_heading_y = sin(MAG_Heading);
	errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
	Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
	Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

	Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
	Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
}
*/
/**************************************************/

void Sensors :: Matrix_update(void)
{
	Gyro_Vector[0]=Gyro_Scaled_X(gyro_x); //gyro x roll
	Gyro_Vector[1]=Gyro_Scaled_Y(gyro_y); //gyro y pitch
	Gyro_Vector[2]=Gyro_Scaled_Z(gyro_z); //gyro Z yaw

	Accel_Vector[0]=accel_x;
	Accel_Vector[1]=accel_y;
	Accel_Vector[2]=accel_z;

	Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
	Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

	//Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement

	Update_Matrix[0][0]=0;
	Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
	Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
	Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
	Update_Matrix[1][1]=0;
	Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
	Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
	Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
	Update_Matrix[2][2]=0;

	Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

	for(int x=0; x<3; x++) //Matrix Addition (update)
	{
		for(int y=0; y<3; y++)
			DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
	}
}

void Sensors :: Euler_angles(void)
{
	pitch = -asin(DCM_Matrix[2][0]);
	roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
	yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

void Sensors :: Compass_Heading()
{
	float MAG_X;
	float MAG_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);

	// adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
	c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
	c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
	c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;

	// Tilt compensated Magnetic filed X:
	MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
	// Tilt compensated Magnetic filed Y:
	MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-MAG_Y,MAG_X);
}
