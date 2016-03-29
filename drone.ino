// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
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

#define RC_THR_MIN   1116
#define RC_YAW_MIN   1020 
#define RC_YAW_MAX   1828
#define RC_PIT_MIN   1096
#define RC_PIT_MAX   1748
#define RC_ROL_MIN   1044
#define RC_ROL_MAX   1788


#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

void setup() 
{ 
	// This must be first
	// Start Serial Monitor

//  Serial.begin(9600);
//  Serial.println("started serial");
  
	if (!receiver.init())
    Serial.println("receiver failed");


  TWBR = ((F_CPU / 400000) - 16) / 2;//set the I2C speed to 400KHz
  sensors.printTimer = millis();
  sensors.timer = micros();
  
  if (!sensors.init())
    Serial.println("Sensors failed");
	
	if (!motors.init())
    Serial.println("Motors failed");

  motors.setSpeeds(0,0,0,0);
  
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

  
    // read high
//  while()
  {
    receiver.maxYaw = max(receiver.readYaw(), receiver.maxYaw);
    receiver.maxPitch = max(receiver.readPitch(), receiver.maxPitch);
    receiver.maxThrottle = max(receiver.readThrottle(), receiver.maxThrottle);
    receiver.maxRoll = max(receiver.readRoll(), receiver.maxRoll);
  }
//  while()
  {
    receiver.minYaw = min(receiver.readYaw(), receiver.minYaw);
    receiver.minPitch = min(receiver.readPitch(), receiver.minPitch);
    receiver.minThrottle = min(receiver.readThrottle(), receiver.minThrottle);
    receiver.minRoll = min(receiver.readRoll(), receiver.minRoll);
  }
  motors.armMotor();
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
  //  Serial.println(String(ch1) + " " + String(ch2) + " " + String(ch3) + " " + String(ch4));
  
    
    long rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input
    rcthr = ch3;
    rcyaw = map(ch1, receiver.minYaw, receiver.maxYaw, -150, 150);
    rcpit = map(ch2, receiver.minPitch, receiver.maxPitch, 45, -45);
    rcroll = map(ch4, receiver.minRoll, receiver.maxRoll, -45, 45);
  //  Serial.println("Thr: " + String(rcthr) + " Yaw: " + String(rcyaw) + " Pit: " + String(rcpit) + " Roll: " + String(rcroll));
  
    float roll,pitch,yaw;  
    RTVector3 sensorVector = sensors.readSensors();
   
    roll = sensorVector.y();
    pitch = sensorVector.x();
    yaw = sensorVector.z();
  //  Serial.println("Pitch " + String(roll) + "  Roll" + String(pitch) + "  Yaw " + String(yaw));
    
    RTVector3 gyroVector = sensors.readGyro();
    float gyroPitch = gyroVector.x(), gyroRoll = gyroVector.y(), gyroYaw = gyroVector.z(); //convert to deg??
  //  Serial.println(String(ceil(gyroRoll)) + " " + String(ceil(gyroPitch)) + " " +String(ceil(gyroYaw)));
  if (motors.motorsArmed())
  {  
  	if(rcthr > receiver.minThrottle+50) 
  	{   // **MINIMUM THROTTLE TO DO CORRECTIONS MAKE THIS 20pts ABOVE YOUR MIN THR STICK ** /
        // Minimum is about 1204
  
  
      // Stablise PIDS
      float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
      float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
      float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
  //    Serial.println(String(roll_stab_output) + " " + String(pitch_stab_output) + " " + String(yaw_stab_output));
      
      // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
      if(abs(rcyaw ) > 5) {
        yaw_stab_output = rcyaw;
        yaw_target = yaw;   // remember this yaw for when pilot stops
      }
      
      // rate PIDS
      long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);  
      long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
      long yaw_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  
      
      	
      float FL = rcthr + roll_output + pitch_output - yaw_output;
      float BL = rcthr + roll_output - pitch_output + yaw_output;    
      float FR = rcthr - roll_output + pitch_output + yaw_output;
      float BR = rcthr - roll_output - pitch_output - yaw_output;
  		motors.setSpeeds(FL, FR, BR, BL);
  //    Serial.println("FL: " + String(FL) + " FR: " + String(FR) + " BR: " + String(BR) + " BL: " + String(BL));
  //    Serial.println(String(roll_output) + " " + String(pitch_output) + " " + String(yaw_output));
      
  	} 
  	else 
  	{
  	  // MOTORS OFF
  		motors.setSpeeds(1100,1100,1100,1100); //not sure if 1000 is correct.
  
  		// reset yaw target so we maintain this on takeoff
  		yaw_target = yaw;
  		
  		for(int i=0; i<6; i++) // reset PID integrals whilst on the ground
  			pids[i].reset_I();
  	}
  }
  else
  {
     // MOTORS OFF
      motors.setSpeeds(0,0,0,0); //not sure if 1000 is correct.
  
      // reset yaw target so we maintain this on takeoff
      yaw_target = yaw;
      
      for(int i=0; i<6; i++) // reset PID integrals whilst on the ground
        pids[i].reset_I();
  }
}
