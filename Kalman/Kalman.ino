#include <Wire.h>
#include <ADXL345.h>

float GyroX,GyroY,GyroZ,GyroTemp,biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ,xAng,yAng;
float timeStep = 0.02;
float pitchGyro = 0;
float pitchAccel = 0;
float pitchPrediction = 0;
float rollGyro = 0;
float rollAccel = 0;
float rollPrediction = 0;
float gyroVar = 0.1;
float deltaGyroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1;
float Pvv = 0.1;
float Pxv = 0.1;
float kx,kv;
unsigned long timer;

const float pi = 3.14159;
int aX,aY,aZ;
int g_offx = 0;
int g_offy = 0;
int g_offz = 0;
int totalGyroXValues = 0;
int totalGyroYValues = 0;
int totalGyroZValues = 0;
int totalAccelXValues = 0;
int totalAccelYValues = 0;
int totalAccelZValues = 0;
int i;

ADXL345 adxl;
#define ITG3200_Address 0x68

void setup() {
  Wire.begin(); 
  Serial.begin(115200);
  delay(100);
  initGyro();
  delay(100);
  GyroCalibrate();
  GyroCalibrate2();
  delay(100);
  adxl.powerOn();
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(45);
}

void loop() {
  kalman();
  Serial.print(pitchGyro);
  Serial.print("\t");
  Serial.print(pitchAccel);
  Serial.print("\t");
  Serial.print(pitchPrediction);
  Serial.print("\t"); 
  Serial.print(rollGyro);
  Serial.print("\t");
  Serial.print(rollAccel);
  Serial.print("\t");
  Serial.print(rollPrediction);
  Serial.print("\n");

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

void GyroCalibrate(){

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

void GyroCalibrate2(){
  for (i = 0; i < 50; i += 1) {
    getGyro();
    getAcc();
    totalGyroXValues += GyroX;
    totalGyroYValues += GyroY;
    totalGyroZValues += GyroZ;
    totalAccelXValues += aX;
    totalAccelYValues += aY;
    totalAccelZValues += aZ;
    delay(20);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  biasAccelX = totalAccelXValues / 50;
  biasAccelY = totalAccelYValues / 50;
  biasAccelZ = (totalAccelZValues / 50) - 256;
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

void getAcc(){
  adxl.readAccel(&aX, &aY, &aZ);
  xAng = atan2(aX,aZ)*(180/pi);
  yAng = atan2(aY,aZ)*(180/pi);
  
}

void kalman(){
  timer = millis();
  getGyro();
  getAcc();
  
  pitchAccel = atan2((aY-biasAccelY)/256,(aZ-biasAccelZ)/256)*(180.0/pi);
  pitchGyro = pitchGyro + ((GyroX - biasGyroX)/14.375)*timeStep;
  pitchPrediction = pitchPrediction + ((GyroX - biasGyroX)/14.375)*timeStep;
  
  rollAccel = atan2((aX - biasAccelX) / 256, (aZ - biasAccelZ) / 256) * 180.0 / pi;
  rollGyro = rollGyro - ((GyroY - biasGyroY) / 14.375) * timeStep; 
  rollPrediction = rollPrediction - ((GyroY - biasGyroY) / 14.375) * timeStep;
  
  Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
  Pxv += timeStep * Pvv;
  Pxx += timeStep * gyroVar;
  Pvv += timeStep * deltaGyroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  
  pitchPrediction += (pitchAccel - pitchPrediction) * kx;
  rollPrediction += (rollAccel - rollPrediction) * kx;
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;
  
  timer = millis() - timer;
  timer = (timeStep * 1000) - timer; 
  delay(timer);
}
