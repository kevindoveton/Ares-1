// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
// Ares 1
// 2015-2016 Kevin Doveton // Lewis Daly

#include "sensors.h"


// Global Variables
Sensors sensors;

void setup()
{
	// Start Serial Monitor
	Serial.begin(9600);

	// Initiate Sensors
	sensors.init();
}

void loop()
{
	sensors.readSensors();
}

// !,AN:-23,3,-17,-1,-11,257,0.18,-1.07,1.87
