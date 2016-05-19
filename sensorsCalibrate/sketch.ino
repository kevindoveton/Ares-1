
#include "MinIMU9AHRS.h"
#include <Wire.h>

// Instantiate the AHRS
MinIMU9AHRS ahrs;

void setup()
{
  Serial.begin(9600);
  Serial.println("SETUP START");
  Wire.begin();
  // Set up the AHRS with the I2C pins.
  ahrs.init();
  Serial.println("SETUP COMPLETE");
};

void loop()
{
  ahrs.updateReadings();

  EulerAngle euler = ahrs.getEuler();

  Serial.print(ToDeg(euler.roll));
  Serial.print(",");
  Serial.print(ToDeg(euler.pitch));
  Serial.print(",");
  Serial.print(ToDeg(euler.yaw));
  Serial.println();

};
