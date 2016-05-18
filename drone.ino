// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
// Ares 1
// 2015-2016 Kevin Doveton // Lewis Daly

#include "receiver.h"
#include "motors.h"
#include "sensors.h"
#include "pid.h"


// Global Variables
Receiver receiver;
Motors motors;
Sensors sensors;
int ch1 = 100, ch2 = 100, ch3 = 100, ch4 = 100, ch5 = 100, ch6 = 100;
PID pids[6];

// Defines
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
// Wrap 180 - Contain results within -180 and +180
// If x < -180, add 360
// If x > 180, subtract 360
// else -180 < x < 180, leave x as is

void setup()
{
	// This must be first
	// Start Serial Monitor
	Serial.begin(9600);
	// Serial.println("started serial");

	// Initiate Receiver
	receiver.init();
	// if (!receiver.init())
		// Serial.println("receiver failed");


	TWBR = ((F_CPU / 400000) - 16) / 2;//set the I2C speed to 400KHz
	sensors.printTimer = millis();
	sensors.timer = micros();

	// Initiate Sensors
	sensors.init();
	// if (!sensors.init())
		// Serial.println("Sensors failed");

	// if (!motors.init())
		// Serial.println("Motors failed");

	pids[PID_PITCH_RATE].kP(0.17);
	// pids[PID_PITCH_RATE].kI(1);
	pids[PID_PITCH_RATE].imax(50);

	pids[PID_ROLL_RATE].kP(0.17);
	// pids[PID_ROLL_RATE].kI(1);
	pids[PID_ROLL_RATE].imax(50);

	pids[PID_YAW_RATE].kP(0.25);
	// pids[PID_YAW_RATE].kI(1);
	pids[PID_YAW_RATE].imax(8.0);

	pids[PID_PITCH_STAB].kP(0.5);
	pids[PID_ROLL_STAB].kP(0.5);
	pids[PID_YAW_STAB].kP(10);


	// read high
	while(!motors.motorsArmed())
	{
		// Arming Sequence
		// Both Sticks in the top right position
		// Both Sticks in the bottom left position
		// Serial.println("!motorsArmed");
		int timer = 0;
		int startTime = millis();

		if (receiver.readThrottle() > 1600)
		{
			while ((timer < 2000) &&
				(startTime > millis()-200) &&
				(receiver.readThrottle() > receiver.maxThrottle - 50) &&
				(receiver.readRoll() > receiver.maxRoll - 50))
			{
				// Serial.println("Max2");
				startTime = millis();
				receiver.maxYaw = max(receiver.readYaw(), receiver.maxYaw);
				receiver.maxPitch = max(receiver.readPitch(), receiver.maxPitch);
				receiver.maxThrottle = max(receiver.readThrottle(), receiver.maxThrottle);
				receiver.maxRoll = max(receiver.readRoll(), receiver.maxRoll);
				timer += millis() - startTime;
			}
		}

		// Wait for user to pull sticks from max to min position
		while ((receiver.readThrottle() > receiver.minThrottle) &&
					(timer > 1950))
		{
			timer = 2000;
		}

		delay(500);
		startTime = millis();

		while ((timer < 4000) &&
			(timer >= 1950) &&
			(startTime > millis() - 200) &&
			(receiver.readThrottle() < receiver.minThrottle + 50) &&
			(receiver.readRoll() < receiver.minRoll + 50))
		{
			// Serial.println("Min");
			startTime = millis();
			receiver.minYaw = min(receiver.readYaw(), receiver.minYaw);
			receiver.minPitch = min(receiver.readPitch(), receiver.minPitch);
			receiver.minThrottle = min(receiver.readThrottle(), receiver.minThrottle);
			receiver.minRoll = min(receiver.readRoll(), receiver.minRoll);
			timer += millis() - startTime;
		}

		// Serial.println(timer);
		// Serial.println(receiver.minThrottle);

		if (timer > 3950)
			motors.armMotor();
		else
			timer = 0;
	}
	// Initiate Motors
	motors.init();
	motors.setSpeeds(receiver.minThrottle,receiver.minThrottle,receiver.minThrottle,receiver.minThrottle);
}

void loop()
{
	static float yaw_target = 0;
	// Get Pulse Width of all Channels
	ch1 = receiver.readYaw();
	ch2 = receiver.readPitch();
	ch3 = receiver.readThrottle();
	ch4 = receiver.readRoll();
	// Serial.println(String(ch1) + " " + String(ch2) + " " + String(ch3) + " " + String(ch4));


	long rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input
	rcthr = ch3;
	rcyaw = map(ch1, receiver.minYaw, receiver.maxYaw, 150, -150);
	rcpit = map(ch2, receiver.minPitch, receiver.maxPitch, 45, -45);
	rcroll = map(ch4, receiver.minRoll, receiver.maxRoll, -45, 45);
	// Serial.println("Thr: " + String(rcthr) + " Yaw: " + String(rcyaw) + " Pit: " + String(rcpit) + " Roll: " + String(rcroll));
	// down + right +
	RTVector3 sensorVector = sensors.readSensors();
	float roll = sensorVector.x();
	float pitch = sensorVector.y();
	float yaw = sensorVector.z();
	// Serial.println("Roll " + String(roll) + "  Pitch" + String(pitch) + "  Yaw " + String(yaw));

	RTVector3 gyroVector = sensors.readGyro();
	float gyroRoll = gyroVector.x();
	float gyroPitch = gyroVector.y();
	float gyroYaw = gyroVector.z(); //convert to deg??
	// Serial.println(String(ceil(gyroRoll)) + " " + String(ceil(gyroPitch)) + " " +String(ceil(gyroYaw)));
	if (motors.motorsArmed())
	// if (0) // Use this to never start the motors
	{
		if (rcthr > receiver.minThrottle + 20) // throttle above 0, motors ARMED
		{
			// Stablise PIDS
			float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250);
			float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
			float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);

			// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
			if(abs(rcyaw ) > 5) {
				yaw_stab_output = rcyaw;
				yaw_target = yaw;   // remember this yaw for when pilot stops
			}

			// Rate PIDS
			long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -500, 500);
			long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);
			long yaw_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);

			// Motor Speeds
			// Positive Roll  ->  Left
			// Positive Pitch   ->  Forward
			float FL = rcthr + roll_output + pitch_output + yaw_output;
			float BL = rcthr + roll_output - pitch_output - yaw_output;
			float FR = rcthr - roll_output + pitch_output - yaw_output;
			float BR = rcthr - roll_output - pitch_output + yaw_output;
			motors.setSpeeds(FL, FR, BR, BL); // Set Motor Speeds

			// Serial Prints
			// Serial.println("FL: " + String(FL) + " FR: " + String(FR) + " BR: " + String(BR) + " BL: " + String(BL));
			// Serial.println(String(FL) + String(FR) + String(BR) + String(BL));
			// Serial.println(String(roll_output) + " " + String(pitch_output) + " " + String(yaw_output));
			// Serial.println(String(roll_stab_output) + " " + String(pitch_stab_output) + " " + String(yaw_stab_output));
		}
		else // throttle at 0, motors ARMED
		{
			motors.setSpeeds(receiver.minThrottle,receiver.minThrottle,receiver.minThrottle,receiver.minThrottle);

			// reset yaw target so we maintain this on takeoff
			yaw_target = yaw;

			// reset PID integrals whilst on the ground
			for (int i=0; i<6; i++)
				pids[i].reset_I();
		}
	}
	else // Throttle at 0, motors DISARMED
	{
		// MOTORS OFF
		motors.setSpeeds(receiver.minThrottle,receiver.minThrottle,receiver.minThrottle,receiver.minThrottle);

			// reset yaw target so we maintain this on takeoff
			yaw_target = yaw;

			for(int i=0; i<6; i++) // reset PID integrals whilst on the ground
				pids[i].reset_I();
	}
}
