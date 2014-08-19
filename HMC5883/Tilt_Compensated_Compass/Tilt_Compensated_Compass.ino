#include <Wire.h>
#include <ADXL345.h>
#include <Math.h>

#define address 0x1E
#define accConv 0.0039

ADXL345 adxl;

int mX,mY,mZ,magHeading,aX,aY,aZ;
float pitch,roll,heading,CMx,CMy,accX,accY,accZ;
const float pi = 3.14159;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  adxl.powerOn();
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(45);
  
  initMag();
}

void loop() {
  getAcc();
  getMag();
  calcTiltHeading();
  Serial.print("Mag: ");
  Serial.println(magHeading);
  Serial.print("Tilt: ");
  Serial.println(heading);
  Serial.print("Pitch: ");
  Serial.println(pitch*(180/PI));
  Serial.print("Roll: ");
  Serial.println(roll*(180/PI));
  delay(100);

}

void initMag(){
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void getMag(){
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    mX = Wire.read()<<8; //X msb
    mX |= Wire.read(); //X lsb
    mZ = Wire.read()<<8; //Z msb
    mZ |= Wire.read(); //Z lsb
    mY = Wire.read()<<8; //Y msb
    mY |= Wire.read(); //Y lsb
     
  }
  
}

void getAcc(){
  adxl.readAccel(&aX, &aY, &aZ);
  
  accX = aX * accConv;
  accY = aY * accConv;
  accZ = aZ * accConv;
  pitch = atan2(-accX,accZ);
  roll = atan2(accY,accZ);
}

void calcTiltHeading(){
  
  CMx = mX * cos(pitch) + mZ * sin(pitch);
  CMy = mX * sin(roll) * sin(pitch) + mY * cos(roll) - mZ * sin(roll) * cos(pitch);
  heading = atan2(CMy,CMx);
  if (heading < 0){heading += 2*PI;}
  if (heading > 2*PI) {heading -= 2*PI;}
  heading = heading * (180/PI);
}
