// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
// RC Channel Assignments
// 1 Yaw
// 2 Pitch
// 3 Throttle
// 4 Roll
// 5 R 
// 6 L
#define RC_THR_MIN   1116
#define RC_YAW_MIN   1020 
#define RC_YAW_MAX   1828
#define RC_PIT_MIN   1096
#define RC_PIT_MAX   1748
#define RC_ROL_MIN   1044
#define RC_ROL_MAX   1788

#include <Arduino.h>

class Receiver {
	public:
		bool init();
    bool armMotors();
		unsigned long readYaw();
		unsigned long readPitch();
		unsigned long readThrottle();
		unsigned long readRoll();
		unsigned long readRSwitch();
		unsigned long readLSwitch();

		int minYaw = 1500, maxYaw = 1500;
		int minThrottle = 1500, maxThrottle = 1500;
		int minPitch = 1500, maxPitch = 1500;
		int minRoll = 1500, maxRoll = 1500;

	private:
		const int yawPin = 2;
		const int pitchPin = 4;
		const int throttlePin = 7;
		const int rollPin = 8;
		const int rSwitchPin = A0;
		const int lSwitchPin = A1;

		unsigned long readPulse(int pin, int signal, unsigned long timeout);
};

