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


unsigned long Receiver :: readYaw()
{
	return readPulse(yawPin, HIGH, 25000)
}


unsigned long Receiver :: readPitch()
{
	return readPulse(pitchPin, HIGH, 25000)
}


unsigned long Receiver :: readThrottle()
{
	return readPulse(throttlePin, HIGH, 25000)
}


unsigned long Receiver :: readRoll()
{
	return readPulse(rollPin, HIGH, 25000)
}


unsigned long Receiver :: readRSwitch()
{
	return readPulse(rSwitchPin, HIGH, 25000)
}


unsigned long Receiver :: readLSwitch()
{
	return readPulse(lSwitchPin, HIGH, 25000)
}


unsigned long Receiver :: readPulse(int pin, int signal, unsigned long timeout)
{
	unsigned long current, killTime, ptime;
	current = micros();
	killTime = current + timeout;

	while(digitalRead(pin) == signal)
	{
		delayMicroseconds(4); 
		current = micros();

		if(current >= killTime){
 			return 0UL;
		}

	}

	while(digitalRead(pin) != signal)
	{
		delayMicroseconds(4);
		current = micros(); 
		
		if(current >= killTime){
			return 0UL;
		}

	}
	ptime = micros();
	while(digitalRead(pin) == signal){
		delayMicroseconds(4); 
	}
	return micros() - ptime;
}
