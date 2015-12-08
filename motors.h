#include <Servo.h>

class Motors {
	public:
		void init();

	private:
		const int esc1Pin = 3;
		const int esc2Pin = 9;
		const int esc3Pin = 10;
		const int esc4Pin = 11;	

		Servo esc1;
		Servo esc2;
		Servo esc3;
		Servo esc4;
}