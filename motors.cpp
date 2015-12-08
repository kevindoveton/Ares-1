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