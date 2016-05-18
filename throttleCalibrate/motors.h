// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
#include <Servo.h>
#include <Arduino.h>

class Motors {
	public:
		bool init();
		void setSpeeds(float FL, float FR, float BR, float BL);
    bool motorsArmed();
    void armMotor();
    void disarmMotor();

	private:
		const int esc1Pin = 3;
		const int esc2Pin = 5;
		const int esc3Pin = 6;
		const int esc4Pin = 9;

		Servo esc1;
		Servo esc2;
		Servo esc3;
		Servo esc4;

    bool motorArm = false;
    

//   void setAllSpeed(int);
};

