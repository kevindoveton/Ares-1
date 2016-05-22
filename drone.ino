// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
// Ares 1
// 2015-2016 Kevin Doveton // Lewis Daly

/* --------  CONFIG  --------*/
#define stabPitchP	0.5
#define stabPitchI	0
#define stabPitchD	0
#define stabRollP	0.5
#define stabRollI	0
#define stabRollD	0
#define stabYawP	0.5
#define stabYawI	0
#define stabYawD	0

#define ratePitchP	0.25
#define ratePitchI	0.2
#define ratePitchD	0
#define rateRollP	0.25
#define rateRollI	0.2
#define rateRollD	0
#define rateYawP	0.25
#define rateYawI	0.2
#define rateYawD	0

#include "receiver.h"
#include "motors.h"
#include "MinIMU9AHRS.h"
// #include "pid.h"
#include "PID_v1.h"
#include <Wire.h>


// Global Variables
Receiver receiver;
Motors motors;
MinIMU9AHRS sensors;
int ch1, ch2, ch3, ch4;

// pids
double pitch_rate_output, roll_rate_output, yaw_rate_output;
double pitch_stab_output, roll_stab_output, yaw_stab_output;
double pitch_rate_input, roll_rate_input, yaw_rate_input;
double pitch_stab_input, roll_stab_input, yaw_stab_input;
double pitch_rate_error, roll_rate_error, yaw_rate_error;
double pitch_stab_error, roll_stab_error, yaw_stab_error;

PID PID_PITCH_RATE(&pitch_rate_input, &pitch_rate_output, &pitch_rate_error, ratePitchP, ratePitchI, ratePitchD, DIRECT);
PID PID_ROLL_RATE(&roll_rate_input, &roll_rate_output, &roll_rate_error, rateRollP, rateRollI, rateRollD, DIRECT);
PID PID_YAW_RATE(&yaw_rate_input, &yaw_rate_output, &yaw_rate_error, rateYawP, rateYawI, rateYawD, DIRECT);
PID PID_PITCH_STAB(&pitch_stab_input, &pitch_stab_output, &pitch_stab_error, stabPitchP, stabPitchI, stabPitchD, REVERSE);
PID PID_ROLL_STAB(&roll_stab_input, &roll_stab_output, &roll_stab_error, stabRollP, stabRollI, stabRollD, REVERSE);
PID PID_YAW_STAB(&yaw_stab_input, &yaw_stab_output, &yaw_stab_error, stabYawP, stabYawI, stabYawD, REVERSE);

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

	// // Initiate Receiver
	receiver.init();

	// PIDS
	// ---------------
	// Set PID modes
	PID_PITCH_RATE.SetMode(AUTOMATIC);
	PID_ROLL_RATE.SetMode(AUTOMATIC);
	PID_YAW_RATE.SetMode(AUTOMATIC);
	PID_PITCH_STAB.SetMode(AUTOMATIC);
	PID_ROLL_STAB.SetMode(AUTOMATIC);
	PID_YAW_STAB.SetMode(AUTOMATIC);
	// Set PID limits
	PID_PITCH_STAB.SetOutputLimits(-250, 250);
	PID_ROLL_STAB.SetOutputLimits(-250, 250);
	PID_YAW_STAB.SetOutputLimits(-360, 360);
	PID_PITCH_RATE.SetOutputLimits(-500, 500);
	PID_ROLL_RATE.SetOutputLimits(-500, 500);
	PID_YAW_RATE.SetOutputLimits(-500, 500);

	PID_PITCH_STAB.SetOutputLimits(-250, 250);
	PID_ROLL_STAB.SetOutputLimits(-250, 250);
	PID_YAW_STAB.SetOutputLimits(-360, 360);
	PID_PITCH_RATE.SetOutputLimits(-500, 500);
	PID_ROLL_RATE.SetOutputLimits(-500, 500);
	PID_YAW_RATE.SetOutputLimits(-500, 500);

	// read high
	while(!motors.motorsArmed())
	{
		// Arming Sequence
		// Both Sticks in the top inwards position
		// Both Sticks in the bottom outwards position
		// Serial.println("!motorsArmed");
		int timer = 0;
		int startTime = millis();

		if (receiver.readThrottle() > 1600)
		{
			while ((timer < 1000) &&
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
					(timer > 950))
		{
			timer = 1000;
		}

		delay(500);
		startTime = millis();

		while ((timer < 2000) &&
			(timer >= 950) &&
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

		if (timer > 1950)
			motors.armMotor();
		else
			timer = 0;
	}
	// Initiate Motors
	motors.init();
	motors.setSpeeds(receiver.minThrottle,receiver.minThrottle,receiver.minThrottle,receiver.minThrottle);

	// Initiate Sensors
	Wire.begin();
	sensors.init();
}

void loop()
{
	static float yaw_target = 0;

	// RC INPUT
	// ---------------------------------

	// Get Pulse Width of all Channels
	ch1 = receiver.readYaw();
	ch2 = receiver.readPitch();
	ch3 = receiver.readThrottle();
	ch4 = receiver.readRoll();
	// Serial.println(String(ch1) + " " + String(ch2) + " " + String(ch3) + " " + String(ch4));


	long rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input
	rcthr = ch3;
	rcyaw = map(ch1, receiver.minYaw, receiver.maxYaw, -150, 150);
	rcpit = map(ch2, receiver.minPitch, receiver.maxPitch, 45, -45);
	rcroll = map(ch4, receiver.minRoll, receiver.maxRoll, -45, 45);
	// Serial.println("Thr: " + String(rcthr) + " Yaw: " + String(rcyaw) + " Pit: " + String(rcpit) + " Roll: " + String(rcroll));


	// SENSORS
	// ---------------------------------

	// down + right +
	sensors.updateReadings();
	EulerAngle sensorVector = sensors.getEuler();
	float roll = ToDeg(sensorVector.roll);
	float pitch = ToDeg(sensorVector.pitch);
	float yaw = ToDeg(sensorVector.yaw);
	// Serial.println(String(roll) + " " + String(pitch) + " " + String(yaw));

	EulerAngle gyroVector = sensors.getGyroEuler();
	float gyroRoll = gyroVector.roll;
	float gyroPitch = gyroVector.pitch;
	float gyroYaw = gyroVector.yaw; //convert to deg??
	// up neg, left neg
	// Serial.println(String(ceil(gyroRoll)) + " " + String(ceil(gyroPitch)) + " " +String(ceil(gyroYaw)));


	if (motors.motorsArmed())
	// if (0) // Use this to never start the motors
	{
		if (rcthr > receiver.minThrottle + 20) // throttle above 0, motors ARMED
		{
			// Stablise PIDS
			pitch_stab_output = rcpit - pitch;
			roll_stab_output = rcroll - roll;
			yaw_stab_output = wrap_180(yaw_target - yaw);

			PID_PITCH_STAB.Compute();
			PID_ROLL_STAB.Compute();
			PID_YAW_STAB.Compute();

			if(abs(rcyaw ) > 5) {
				yaw_stab_output = rcyaw;
				yaw_target = yaw;   // remember this yaw for when pilot stops
			}

			pitch_rate_input = double(pitch_stab_output - gyroPitch);
			roll_rate_input = double(roll_stab_output - gyroRoll);
			yaw_rate_input = double(yaw_stab_output - gyroYaw);

			// pitch_rate_input = (float)gyroPitch - rcpit;
			// roll_rate_input = (float)gyroRoll - rcroll;
			// yaw_rate_input = (float)gyroYaw - rcyaw;

			PID_PITCH_RATE.Compute();
			PID_ROLL_RATE.Compute();
			PID_YAW_RATE.Compute();

			float pitch_output = pitch_rate_output;
			float roll_output = roll_rate_output;
			float yaw_output = yaw_rate_output;

			// Motor Speeds
			// Positive Roll  ->  Right
			// Positive Pitch   ->  Back
			float FL = rcthr - roll_output - pitch_output - yaw_output;
			float BL = rcthr - roll_output + pitch_output + yaw_output;
			float FR = rcthr + roll_output - pitch_output + yaw_output;
			float BR = rcthr + roll_output + pitch_output - yaw_output;
			motors.setSpeeds(FL, FR, BR, BL); // Set Motor Speeds


			// TESTING
			// -----------------------------------------

			// motors.setSpeeds(FL, receiver.minThrottle, BR, receiver.minThrottle); // TEST BLACK
			// motors.setSpeeds(receiver.minThrottle, FR, receiver.minThrottle, BL); // TEST RED
			// Serial Prints
			// Serial.println("FL: " + String(FL) + " FR: " + String(FR) + " BR: " + String(BR) + " BL: " + String(BL));
			// Serial.println("FL: " + String(FL) + " BR: " + String(BR));
			// Serial.println(String(roll_output) + " " + String(pitch_output) + " " + String(yaw_output));
			// Serial.println(String(roll_stab_output) + " " + String(pitch_stab_output) + " " + String(yaw_stab_output));
		}
		else // throttle at 0, motors ARMED
		{
			motors.setSpeeds(receiver.minThrottle,receiver.minThrottle,receiver.minThrottle,receiver.minThrottle);

			// reset yaw target so we maintain this on takeoff
			yaw_target = yaw;

			// reset PID integrals whilst on the ground

		}
	}
	else // Throttle at 0, motors DISARMED
	{
		// MOTORS OFF
		motors.setSpeeds(receiver.minThrottle,receiver.minThrottle,receiver.minThrottle,receiver.minThrottle);

			// reset yaw target so we maintain this on takeoff
			yaw_target = yaw;


	}
}
