#include <Wire.h>
#include <ADXL345.h>

ADXL345 adxl;

#define address 0x1E
#define ITG3200_Address 0x68
#define alphaGyro .25
#define alphaYaw 0.8

int mX,mY,mZ,magHeading,aX,aY,aZ;
const float pi = 3.14159;
float GyroX,GyroY,GyroZ,GyroTemp,filterZ,filterX,filterY,gyroHeading,cycle,yaw,gyroHeadingTemp,magHeadingTemp,xAng,yAng,gyroXAng,gyroYAng;

unsigned long clockNew,clockOld;

int g_offx = 0;
int g_offy = 0;
int g_offz = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  adxl.powerOn();
  
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(45);
  
  initMag();
  delay(100);
  initGyro();
  delay(100);
  gyroCalibrate();
  cycle = 0.0;
  clockOld = 0.0;
  getMag();
  getAcc();
  gyroHeading = magHeading;
  gyroXAng = xAng;
  gyroYAng = yAng;
  delay(1000);
  
}

void loop() {
  getAcc();
  getMag();
  getGyro();
  calcGyroHeading();
  calcYaw();
  Serial.print("Mag: ");
  Serial.println(magHeading);
  Serial.print("Gyro: ");
  Serial.println(gyroHeading);
  Serial.print("Yaw: ");
  Serial.println(yaw);
  Serial.print("GyroX: ");
  Serial.println(gyroXAng);
  Serial.print("GyroY: ");
  Serial.println(gyroYAng);
  delay(10);
}

void initMag(){
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
}

void getMag(){
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    mX = Wire.read()<<8; //X msb
    mX |= Wire.read(); //X lsb
    mZ = Wire.read()<<8; //Z msb
    mZ |= Wire.read(); //Z lsb
    mY = Wire.read()<<8; //Y msb
    mY |= Wire.read(); //Y lsb
    magHeading = atan2(mY,mX)*(180/pi)+180;
  }
}

void calcGyroHeading(){
  filterX = (1-alphaGyro)*filterX+alphaGyro*(GyroX/14.35);
  filterY = (1-alphaGyro)*filterY+alphaGyro*(GyroY/14.35);
  filterZ = (1-alphaGyro)*filterZ+alphaGyro*(GyroZ/14.35);
  clockNew = millis();
  cycle = clockNew - clockOld;
  clockOld = clockNew;
  gyroXAng = gyroXAng + (-1)*filterX*((cycle*1.0)/1000.0);
  gyroYAng = gyroYAng + (-1)*filterY*((cycle*1.0)/1000.0);
  gyroHeading = gyroHeading + (-1)*filterZ*((cycle*1.0)/1000.0);
  
  delay(10);
  
  if (gyroHeading > -1000 && gyroHeading < 1000){
    while (gyroHeading>360){
      gyroHeading = gyroHeading - 360;
    }
    while (gyroHeading<0){
      gyroHeading = gyroHeading + 360;
    }
  }
}

void initGyro() {
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x3E);  
   Wire.write(0x00);   
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x15);  
   Wire.write(0x07);   
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x16);  
   Wire.write(0x1E);   // +/- 2000 dgrs/sec, 1KHz, 1E, 19
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x17);  
   Wire.write(0x00);   
   Wire.endTransmission(); 
   
}

void gyroCalibrate(){

 int tmpx = 0;
 int tmpy = 0;
 int tmpz = 0; 

 g_offx = 0;
 g_offy = 0;
 g_offz = 0;
 
 for (char i = 0;i<10;i++)
    {
    delay(10);  
    getGyro();
    tmpx += GyroX;
    tmpy += GyroY;
    tmpz += GyroZ; 
    }  
 g_offx = tmpx/10;
 g_offy = tmpy/10;
 g_offz = tmpz/10;
 
}

void getGyro() {
  Wire.beginTransmission(ITG3200_Address); 
  Wire.write(0x1B);       
  Wire.endTransmission(); 
  
  Wire.beginTransmission(ITG3200_Address); 
  Wire.requestFrom(ITG3200_Address, 8);    // request 8 bytes from ITG3200
  
  int i = 0;
  byte buff[8];
  while(Wire.available())    
  { 
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission(); 
    
  GyroX = ((buff[4] << 8) | buff[5]) - g_offx;
  GyroY = ((buff[2] << 8) | buff[3]) - g_offy;
  GyroZ = ((buff[6] << 8) | buff[7]) - g_offz;
  GyroTemp = (buff[0] << 8) | buff[1]; // temperature 
 
}

void calcYaw(){
  gyroHeadingTemp = gyroHeading;
  magHeadingTemp = magHeading; 
  if (abs(magHeading-gyroHeading) > 200){ // Corrects an Error when compass is close to North
    
    if (gyroHeading < magHeading){
      magHeadingTemp = magHeading - 360;
    }
    if (gyroHeading > magHeading){
      gyroHeadingTemp = gyroHeading - 360;
    }
  }
  yaw = alphaYaw*gyroHeadingTemp+(1-alphaYaw)*magHeadingTemp;
  if (yaw < 0.0){
    yaw = yaw + 360;
  }
}

void getAcc(){
  adxl.readAccel(&aX, &aY, &aZ);
  
  xAng = atan2(aX,aZ)*(180/pi);
  yAng = atan2(aY,aZ)*(180/pi);
}
