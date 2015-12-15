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

void Motors :: setSpeeds(float FL, float FR, float BR, float BL)
{
  esc1.write(FL);
  esc2.write(FR);
  esc3.write(BR);
  esc4.write(BL); 
}

void Motors :: setAllSpeeds(float speed)
{
  setSpeeds(speed, speed, speed, speed);
}

