// RC Channel Assignments
// 1 Yaw
// 2 Pitch
// 3 Throttle
// 4 Roll
// 5 R 
// 6 L

#include <Arduino.h>

class Receiver {
	public:
		bool init();
		void callibrate();

		unsigned long readYaw();
		unsigned long readPitch();
		unsigned long readThrottle();
		unsigned long readRoll();
		unsigned long readRSwitch();
		unsigned long readLSwitch();

		int minYaw, maxYaw;
		int minThrottle, maxThrottle;
		int minPitch, maxPitch;
		int minRoll, maxRoll;

	private:
		const int yawPin = 2;
		const int pitchPin = 4;
		const int throttlePin = 7;
		const int rollPin = 8;
		const int rSwitchPin = A0;
		const int lSwitchPin = A1;

		unsigned long readPulse(int pin, int signal, unsigned long timeout);
};
