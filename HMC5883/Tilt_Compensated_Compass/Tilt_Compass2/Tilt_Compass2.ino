#include <Wire.h>
#include <ADXL345.h>
#include <Comp6DOF_n0m1.h>
#include <math.h>
#include <HMC5883L.h>

#define address 0x1E

ADXL345 adxl;
Comp6DOF_n0m1 sixDOF;
HMC5883L compass;

int mX,mY,mZ,magHeading,aX,aY,aZ;
float roll,pitch,heading,CMx,CMy;
const float pi = 3.14159;

int error = 0;

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
  calcHeading();
  
  Serial.print("Mag: ");
  Serial.println(magHeading);
  Serial.print("Roll: ");
  Serial.println(sixDOF.roll()/100);
  Serial.print("Pitch: ");
  Serial.println(sixDOF.pitch()/100);
  Serial.print("Yaw: ");
  Serial.println(sixDOF.yaw()/100);
  Serial.print("Tilt: ");
  Serial.println(heading);
  delay(50);
}

void initMag(){
  compass = HMC5883L();
  error = compass.SetScale(1.3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
}

void getMag(){
  MagnetometerRaw raw = compass.ReadRawAxis();
}

void getAcc(){
  adxl.readAccel(&aX, &aY, &aZ);
  pitch = atan2(aX,aZ);
  roll = atan2(aY,aZ);
}

void calcHeading(){
  magHeading = atan2(mY,mX)*(180/pi)+180;
  MagnetometerRaw raw = compass.ReadRawAxis();
  sixDOF.compCompass(raw.XAxis, raw.YAxis, raw.ZAxis, aX, aY, aZ, false); 
  heading = degrees(atan2(sixDOF.yAxisComp(),sixDOF.xAxisComp()));
  
}
