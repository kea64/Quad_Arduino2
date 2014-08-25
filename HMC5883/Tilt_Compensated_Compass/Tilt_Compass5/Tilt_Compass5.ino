#include <Wire.h>
#include <ADXL345.h>
#include <compass.h>

ADXL345 adxl;
int magHeading,aX,aY,aZ;
float pitch,roll,heading,CMx,CMy,accX,accY,accZ,yaw;

#define accConv 0.0039
#define ACC_SCALAR 0.93

void setup(){
  Serial.begin(38400);
  Wire.begin();
  
  adxl.powerOn();
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(45);
  
  compass_x_offset = -99.62;
  compass_y_offset = -58.63;
  compass_z_offset = 0;
  compass_x_gainError = 0.93;
  compass_y_gainError = 0.98;
  compass_z_gainError = 0.93;
  
  compass_init(2);
  //compass_offset_calibration(3);
  
}

void loop(){
  getAcc();
  getMag();
  calcTiltHeading();
  Serial.print("Yaw: ");
  Serial.println(yaw);
  Serial.print("Roll: ");
  Serial.println(degrees(roll));
  Serial.print("Pitch: ");
  Serial.println(degrees(pitch));
  delay(250);
}

void getAcc(){
  adxl.readAccel(&aX, &aY, &aZ);
  
  accX = aX * accConv;
  accY = aY * accConv;
  accZ = aZ * accConv;
  pitch = atan2(-accX,accZ) * ACC_SCALAR;
  roll = atan2(accY,accZ) * ACC_SCALAR;
}

void getMag(){
  compass_scalled_reading();
}

void calcTiltHeading(){
  
  CMx = compass_x_scalled * cos(pitch) + compass_z_scalled * sin(pitch);
  CMy = compass_x_scalled * sin(roll) * sin(pitch) + compass_y_scalled * cos(roll) - compass_z_scalled * sin(roll) * cos(pitch);
  yaw = atan2(CMy,CMx);
  if (yaw < 0){yaw += 2*PI;}
  if (yaw > 2*PI) {yaw -= 2*PI;}
  yaw = yaw * (180/PI);
}
