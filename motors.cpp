#include "motors.h"

bool Motors :: init()
{
	esc1.attach(esc2Pin); // FR
	esc2.attach(esc4Pin); // BL
	esc3.attach(esc1Pin); // FL
	esc4.attach(esc3Pin); // BR



  return true;
}

void Motors :: setSpeeds(float FL, float FR, float BR, float BL)
{
  esc1.write(FL);
  esc2.write(FR);
  esc3.write(BR);
  esc4.write(BL); 
}

//void Motors :: setAllSpeeds(float speed)
//{
//  setSpeeds(speed, speed, speed, speed);
//}


