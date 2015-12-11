// Ares 1
// 2015 Kevin Doveton // Lewis Daly

#include "receiver.h"
#include "motors.h"
#include "sensors.h"


//#define DEBUG

Receiver receiver;
Motors motors;
Sensors sensors;

int ch1 = 100, ch2 = 100, ch3 = 100, ch4 = 100, ch5 = 100, ch6 = 100;

void setup() 
{
	receiver.init();
  sensors.init();
	motors.init();


	// Start Serial Monitor
	#ifdef DEBUG
		Serial.begin(9600);
	#endif

}

void loop() 
{

	// Get Pulse Width of all Channels
	// TODO: fix. This is a bit dodgy
//	 ch2 = getCurrentReading(20, ch2);
//	 ch1 = getCurrentReading(19, ch1);
//	 ch4 = getCurrentReading(22, ch4);
//	 ch3 = getCurrentReading(21, ch3);
//	 ch5 = getCurrentReading(5, ch5);
//	 ch6 = getCurrentReading(6, ch6);
	ch1 = receiver.readYaw();
	ch2 = receiver.readPitch();
	ch3 = receiver.readThrottle();
	ch4 = receiver.readRoll();

	long rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input
	rcthr = ch3;
	rcyaw = map(ch1, 1200, 2000, -150, 150);
	rcpit = map(ch2, 1200, 2000, -45, 45);
	rcroll = map(ch4, 1200, 2000, -45, 45);

//  float roll,pitch,yaw;  
//  RTVector3 sensorVector = sensors.readSensors(); //Not sure how to convert to euler...
//  roll = sensorVector.x();


	//seems to be 1300 - 1700
	#ifdef DEBUG
		//Serial.println("Yaw-chl 1:\tPitch-ch2:\tThrott-ch3:\tRoll-ch4:\t");
		String outputString = String(rcthr, DEC) + "\t\t" + String(rcyaw, DEC) + "\t\t" + String(rcpit, DEC) + "\t\t" + String(rcroll, DEC);
		Serial.print(outputString);
		Serial.print("\n");
	#endif


	//Test the esc's
	//int throttle = map(ch3, 1300, 2000, 0, 179);
	//esc1.write(throttle);

	// Delay to Make it Readable
	//  delay(500);
}

//int getCurrentReading(int channelNumber, int lastValue) 
//{
//	int currentReading = pulseIn(channelNumber, HIGH, 25000);
//	if (currentReading != 0) {
//		return currentReading;
//	}
//	return lastValue; 
//}

