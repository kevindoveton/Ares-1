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

void Receiver :: callibrate()
{
	#if DEBUG
		Serial.println("Started callibration");
	#endif

	unsigned long start = millis();

	while (millis() - start <= 5000)
	{
		if (int currentYaw = readYaw() < minYaw)
		{
			minYaw = currentYaw;
		}
		else if (currentYaw > maxYaw)
		{	
			maxYaw = currentYaw;
		}

		if (int currentPitch = readPitch() < minPitch)
		{
			minPitch = currentPitch;
		}
		else if (currentPitch > maxPitch)
		{	
			maxPitch = currentPitch;
		}

		if (int currentThrottle = readThrottle() < minThrottle)
		{	
			minThrottle = currentThrottle;
		}
		else if (currentThrottle > maxThrottle)
		{	
			maxThrottle = currentThrottle;
		}

		if (int currentRoll = readRoll() < minRoll)
		{	
			minRoll = currentRoll;
		}
		else if (currentRoll > maxRoll)
		{	
			maxRoll = currentRoll;
		}

	}
}

unsigned long Receiver :: readYaw()
{
	return readPulse(yawPin, HIGH, 5000);
}


unsigned long Receiver :: readPitch()
{
	return readPulse(pitchPin, HIGH, 5000);
}


unsigned long Receiver :: readThrottle()
{
	return readPulse(throttlePin, HIGH, 5000);
}


unsigned long Receiver :: readRoll()
{
	return readPulse(rollPin, HIGH, 5000);
}


unsigned long Receiver :: readRSwitch()
{
	return readPulse(rSwitchPin, HIGH, 5000);
}


unsigned long Receiver :: readLSwitch()
{
	return readPulse(lSwitchPin, HIGH, 5000);
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
