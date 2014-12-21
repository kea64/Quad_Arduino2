#include <Quad_L3D4200D.h>
#include <Wire.h>

class L3D4200D gyro;

double gyroHeadingX = 0;
double gyroHeadingY = 0;

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
  gyroHeadingX = gyroHeadingX + (1.0)*gyro.x*((cycle*1.0)/1000.0);
  gyroHeadingY = gyroHeadingY + (1.0)*gyro.y*((cycle*1.0)/1000.0);
  
  Serial.println("X: ");
  Serial.println(gyroHeadingX);
  Serial.println("Y: ");
  Serial.println(gyroHeadingY);
  
  delay(20);
  
}


