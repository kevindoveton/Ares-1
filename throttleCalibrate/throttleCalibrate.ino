// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
// Ares 1
// 2015-2016 Kevin Doveton // Lewis Daly

#include "receiver.h"
#include "motors.h"




// Global Variables
Receiver receiver;
Motors motors;

int ch1 = 100, ch2 = 100, ch3 = 100, ch4 = 100, ch5 = 100, ch6 = 100;

void setup() 
{ 
	// This must be first
	// Start Serial Monitor
	Serial.begin(9600);
	// Serial.println("started serial");
	
	// Initiate Receiver
	receiver.init();
	// if (!receiver.init())
		// Serial.println("receiver failed");

	// Initiate Motors
	motors.init();
	// if (!motors.init())
		// Serial.println("Motors failed");

	
}
 
void loop() 
{
	
	ch3 = receiver.readThrottle();
	motors.setSpeeds(ch3, ch3, ch3, ch3); // Set Motor Speeds

}
