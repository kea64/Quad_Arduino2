#include <Wire.h>

float GyroX,GyroY,GyroZ,GyroTemp,filterZ,gyroHeading,cycle;

unsigned long clockNew,clockOld;

int g_offx = 0;
int g_offy = 0;
int g_offz = 0;

#define ITG3200_Address 0x68
#define alphaGyro .25

void setup() {
 Wire.begin(); 
  Serial.begin(115200);
  delay(100);
  initGyro();
  delay(100);
  gyroCalibrate();
  delay(1000);
  cycle = 0.0;
  clockOld = 0.0;
  gyroHeading = 0.0;
}

void loop() {
  getGyro();
  calcGyroHeading();
  Serial.println(gyroHeading);
  
}

void calcGyroHeading(){
  
  filterZ = (1-alphaGyro)*filterZ+alphaGyro*(GyroZ/14.35);
  clockNew = millis();
  cycle = clockNew - clockOld;
  clockOld = clockNew;
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
