#include <Wire.h>
#include <ADXL345.h>

ADXL345 adxl;

int aX,aY,aZ;
float xAng,yAng;
const float pi = 3.14159;

void setup() {
  Serial.begin(115200);
  adxl.powerOn();
  
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(45);
}

void loop() {
   
  getAcc();
  Serial.print("X:");
  Serial.println(aX);
  Serial.print("Y:");
  Serial.println(aY);
  Serial.print("Z:");
  Serial.println(aZ);
  Serial.print("xAng: ");
  Serial.println(xAng);
  Serial.print("yAng: ");
  Serial.println(yAng);
  delay(500);
}

void getAcc(){
  adxl.readAccel(&aX, &aY, &aZ);
  if ((aX > 0 && aZ > 0)||(aX < 0 && aZ > 0)){
    xAng = (atan((1.0*aX)/(1.0*aZ)))*(180.0/pi);
  }
  else if (aX > 0 && aZ < 0){
    xAng = (atan((1.0*aX)/(1.0*aZ)))*(180.0/pi)+180;
  }
  else if (aX < 0 && aZ < 0){
    xAng = (atan((1.0*aX)/(1.0*aZ)))*(180.0/pi)-180;
  }
  
  
  if ((aY > 0 && aZ > 0)||(aY < 0 && aZ > 0)){
    yAng = (atan((1.0*aY)/(1.0*aZ)))*(180.0/pi);
  }
  else if (aY > 0 && aZ < 0){
    yAng = (atan((1.0*aY)/(1.0*aZ)))*(180.0/pi)+180;
  }
  else if (aY < 0 && aZ < 0){
    yAng = (atan((1.0*aY)/(1.0*aZ)))*(180.0/pi)-180;
  }
  
}
