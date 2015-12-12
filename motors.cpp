#include "motors.h"

void Motors :: init()
{
	esc1.attach(esc1Pin);
	esc2.attach(esc2Pin);
	esc3.attach(esc3Pin);
	esc4.attach(esc4Pin);

	#ifdef DEBUG
		Serial.println("Motors :: init()");
	#endif
}

void Motors :: setAllSpeed(float speed)
{
	speed = speed/100 * 180;
	esc1.write(speed);
	esc2.write(speed);
	esc3.write(speed);
	esc4.write(speed);
}

void Motors :: setSpeeds(float FL, float FR, float BR, float BL)
{
  esc1.write(FL);
  esc2.write(FR);
  esc3.write(BR);
  esc4.write(BL); 
}

