#include "receiver.h"

void Receiver :: init()
{
	pinMode(yawPin, INPUT);
	pinMode(pitchPin, INPUT);
	pinMode(throttlePin, INPUT);
	pinMode(rollPin, INPUT);
	pinMode(rSwitchPin, INPUT);
	pinMode(lSwitchPin, INPUT);

	#ifdef DEBUG
		Serial.println("Receiver :: init()");
		Serial.println("Pins: \n Yaw Pin: " + string(yawPin) + ", Pitch Pin: " + string(pitchPin) + ", Throttle Pin: " + string(throttlePin) + ", Roll Pin: " + string(rollPin) + ", Right Switch Pin: " + string(rSwitchPin) + ", Left Switch Pin: " + string(lSwitchPin));
	#endif
}