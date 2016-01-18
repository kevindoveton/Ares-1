// Ares 1
// 2015 Kevin Doveton // Lewis Daly

#include "receiver.h"
#include "motors.h"
#include "sensors.h"
#include "pid.h"

//#define DEBUG

Receiver receiver;
Motors motors;
Sensors sensors;

int ch1 = 100, ch2 = 100, ch3 = 100, ch4 = 100, ch5 = 100, ch6 = 100;

PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

const float pi = 3.1415927;
const float radConst = 57.2957795131;
void setup() 
{ 
	// This must be first
	// Start Serial Monitor
	#ifdef DEBUG
		Serial.begin(9600);
		Serial.println("started serial");
	#endif
  Serial.begin(9600);
    Serial.println("started serial");
  
	if (!receiver.init())
    Serial.println("receiver failed");

  if (!sensors.init())
    Serial.println("Sensors failed");
	
	if (!motors.init())
    Serial.println("Motors failed");

  motors.setAllSpeeds(0);
  //receiver.callibrate();

	pids[PID_PITCH_RATE].kP(0.7);
	//  pids[PID_PITCH_RATE].kI(1);
	pids[PID_PITCH_RATE].imax(50);

	pids[PID_ROLL_RATE].kP(0.7);
	//  pids[PID_ROLL_RATE].kI(1);
	pids[PID_ROLL_RATE].imax(50);

	pids[PID_YAW_RATE].kP(2.5);
	//  pids[PID_YAW_RATE].kI(1);
	pids[PID_YAW_RATE].imax(50);

	pids[PID_PITCH_STAB].kP(4.5);
	pids[PID_ROLL_STAB].kP(4.5);
	pids[PID_YAW_STAB].kP(10);
}

int rev = 0;
 
void loop() 
{
	static float yaw_target = 0;  

	// Get Pulse Width of all Channels
	ch1 = receiver.readYaw();
	ch2 = receiver.readPitch();
	ch3 = receiver.readThrottle();
	ch4 = receiver.readRoll();


  
	long rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input
	rcthr = ch3;
	rcyaw = map(ch1, 1200, 2000, -150, 150);
	rcpit = map(ch2, 1200, 2000, -45, 45);
	rcroll = map(ch4, 1200, 2000, -45, 45);

 

	float roll,pitch,yaw;  
	RTVector3 sensorVector = sensors.readSensors(); //Not sure how to convert to euler...
	//TODO: convert to degrees?
	roll = sensorVector.x();
	pitch = sensorVector.y();
	yaw = sensorVector.z();
//  motors.setAllSpeeds(rcthr);
//  Serial.println("Pitch " + String(pitch) + "  Roll" + String(roll) + "  Yaw " + String(yaw));
	

	RTVector3 gyroVector = sensors.readGyro();
	float gyroPitch = gyroVector.y(), gyroRoll = gyroVector.x(), gyroYaw = gyroVector.z(); //convert to deg?
 

 
  //Serial.println("Pitch " + String(gyroPitch) + "Roll" + String(roll) + "Yaw " + String(yaw) + "Throttle" + String(rcthr));
   


	if(rcthr > 1300) 
	{   // **MINIMUM THROTTLE TO DO CORRECTIONS MAKE THIS 20pts ABOVE YOUR MIN THR STICK ** /


    // our new stab pids
    float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
    float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
    
    // rate pids from earlier
    long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);  
    long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
    long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500); 
      
    
    
    if(abs(rcyaw) > 5) {  // if pilot commanding yaw
      yaw_stab_output = rcyaw;  // feed to rate controller (overwriting stab controller output)
      yaw_target = yaw;         // update yaw target
    }
    
    	

		motors.setSpeeds(rcthr - roll_output - pitch_output - yaw_output, 
						rcthr + roll_output - pitch_output + yaw_output, 
						rcthr + roll_output + pitch_output - yaw_output, 
						rcthr - roll_output + pitch_output + yaw_output);
    Serial.println(String(rcthr - roll_output - pitch_output - yaw_output) + " " + String(rcthr + roll_output - pitch_output + yaw_output) + " " + String(rcthr + roll_output + pitch_output - yaw_output) + " " + String(rcthr - roll_output + pitch_output + yaw_output));
//    Serial.println(String(roll_output) + " " + String(pitch_output) + " " + String(yaw_output));
	} 
	else 
	{ 
	  // MOTORS OFF
		motors.setAllSpeeds(0); //not sure if 1000 is correct.

		// reset yaw target so we maintain this on takeoff
		yaw_target = yaw;
		
		for(int i=0; i<6; i++) // reset PID integrals whilst on the ground
			pids[i].reset_I();
	}
	


	//seems to be 1300 - 1700
	#ifdef DEBUG
		Serial.println("Yaw-chl 1:\tPitch-ch2:\tThrott-ch3:\tRoll-ch4:\t");
		String outputString = String(rcthr, DEC) + "\t\t" + String(rcyaw, DEC) + "\t\t" + String(rcpit, DEC) + "\t\t" + String(rcroll, DEC);
		Serial.print(outputString);
		Serial.print("\n");
	#endif
}
