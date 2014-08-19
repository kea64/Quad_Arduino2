#include <Wire.h>
#include <ADXL345.h>

float GyroX,GyroY,GyroZ,GyroTemp,biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ,biasMagHeading,xAng,yAng,cycle,pitchGyro,rollGyro,yawGyro,pitchPrediction,rollPrediction,yawPrediction;
float timeStep = 0.02;
float pitchAccel = 0;
float rollAccel = 0;
float yawMag = 0;
float gyroVar = 0.1;
float deltaGyroVar = 0.1;
float accelVar = 5;
float magVar = 5;
float Pxx = 0.1;
float Pvv = 0.1;
float Pxv = 0.1;
float kx,kv,kmx,kmv;
unsigned long clockNew,clockOld;

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
int totalMagValues = 0;
int i,mX,mY,mZ,magHeading;

ADXL345 adxl;
#define ITG3200_Address 0x68
#define address 0x1E
#define cycleDelay 20

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
  initMag();
  clockOld = 0.0;
  determineAngles();
  
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
  Serial.print("\t");
  Serial.print(yawMag);
  Serial.print("\t");
  Serial.print(yawGyro);
  Serial.print("\t");
  Serial.print(yawPrediction);
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
    //totalAccelXValues += aX;
    //totalAccelYValues += aY;
    //totalAccelZValues += aZ;
    delay(20);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  //biasAccelX = totalAccelXValues / 50;
  //biasAccelY = totalAccelYValues / 50;
  //biasAccelZ = (totalAccelZValues / 50) - 256;
  biasAccelX = 1;
  biasAccelY = 1;
  biasAccelZ = 1;
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
  clockNew = millis();
  cycle = (((clockNew - clockOld)*1.0)/1000.0);
  getGyro();
  getAcc();
  getMag();
  
  pitchAccel = atan2((aY-biasAccelY)/256,(aZ-biasAccelZ)/256)*(180.0/pi);
  pitchGyro = pitchGyro + ((GyroX - biasGyroX)/14.375)*cycle;
  pitchPrediction = pitchPrediction + ((GyroX - biasGyroX)/14.375)*cycle;
  
  rollAccel = atan2((aX - biasAccelX) / 256, (aZ - biasAccelZ) / 256) * 180.0 / pi;
  rollGyro = rollGyro - ((GyroY - biasGyroY) / 14.375) * cycle; 
  rollPrediction = rollPrediction - ((GyroY - biasGyroY) / 14.375) * cycle;
  
  yawGyro = yawGyro - ((GyroZ - biasGyroZ) / 14.375) * cycle;
  yawPrediction = yawPrediction - ((GyroZ - biasGyroZ) / 14.375) * cycle;
  
  Pxx += cycle * (2 * Pxv + timeStep * Pvv);
  Pxv += cycle * Pvv;
  Pxx += cycle * gyroVar;
  Pvv += cycle * deltaGyroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  kmx = Pxx * (1 / (Pxx + magVar));
  kmv = Pxv * (1 / (Pxx + magVar));
  
  pitchPrediction += (pitchAccel - pitchPrediction) * kx;
  rollPrediction += (rollAccel - rollPrediction) * kx;
  yawPrediction += (yawMag - yawPrediction) * kmx;
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;
  
  clockOld = millis();
  delay(cycleDelay);
}

void determineAngles(){
  getAcc();
  getMag();
  pitchGyro = yAng;
  pitchPrediction = yAng;
  rollGyro = xAng;
  rollPrediction = xAng;
  yawGyro = yawMag;
}

void initMag(){
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
    yawMag = atan2(mY,mX)*(180/pi)+180; 
    if (yawMag > 180){
      yawMag = yawMag - 360;
    }
    yawMag += 90;
    if (yawMag > 180 && yawMag < 270){
     yawMag = yawMag - 360; 
    }
  }
}
