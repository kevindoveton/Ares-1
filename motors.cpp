// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
#include "motors.h"

bool Motors :: init()
{
	esc1.attach(esc1Pin); 
	esc2.attach(esc2Pin); 
	esc3.attach(esc3Pin); 
	esc4.attach(esc4Pin); 
  return true;
}

void Motors :: setSpeeds(float FL, float FR, float BR, float BL)
{
  esc1.write(FL);
  esc2.write(FR);
  esc3.write(BR);
  esc4.write(BL); 
}

bool Motors :: motorsArmed() 
{
   return motorArm;
}

void Motors :: armMotor()
{
  motorArm = true;
}

void Motors :: disarmMotor()
{
  motorArm = true;
}

