#include <Quad_L3D4200D.h>
#include <Wire.h>

class L3D4200D gyro;

double gyroHeading = 0;

unsigned long clockOld;

void setup(){
  Wire.begin();
  Serial.begin(38400);
  
  gyro.init();
  gyro.calibrate();
}

void loop(){
  gyro.update();
  
  int clockNew = millis();
  int cycle = clockNew - clockOld;
  clockOld = clockNew;
  gyroHeading = gyroHeading + (1.0)*gyro.x*((cycle*1.0)/1000.0);
  
  Serial.println(gyroHeading);
  
  delay(20);
  
}


