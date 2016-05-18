// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
// Ares 1
// 2015-2016 Kevin Doveton // Lewis Daly

#include "sensors.h"


// Global Variables
Sensors sensors;


void setup()
{
	// This must be first
	// Start Serial Monitor
	Serial.begin(9600);

	// Initiate Sensors
	sensors.init();

}

void loop()
{
	RTVector3 sen = sensors.readSensors();
	Serial.println("Fused   \t" + String(sen.x()) + " " + String(sen.y()) + " " + String(sen.z()));

	// delay(500);
}
