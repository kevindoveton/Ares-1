// RC Channel Assignments
// 1 Yaw
// 2 Pitch
// 3 Throttle
// 4 Roll
// 5 R 
// 6 L

class Receiver {
	public:
		void init();
		unsigned long readYaw();
		unsigned long readPitch();
		unsigned long readThrottle();
		unsigned long readRoll();
		unsigned long readRSwitch();
		unsigned long readLSwitch();

	private:
		const int yawPin = 2;
		const int pitchPin = 4;
		const int throttlePin = 5;
		const int rollPin = 6;
		const int rSwitchPin = 7;
		const int lSwitchPin = 8;

		unsigned long readPulse(int pin, int signal, unsigned long timeout);
}