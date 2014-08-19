#include <Wire.h>
#include <ADXL345.h>

float GyroX,GyroY,GyroZ,GyroTemp,xAng,yAng,cycle,gyroXAng,gyroYAng,gyroZAng,biasGyroX,biasGyroY,biasGyroZ,biasAccelX,biasAccelY,biasAccelZ,pitch,roll;

int aX,aY,aZ,i;
int g_offx = 0;
int g_offy = 0;
int g_offz = 0;
int totalGyroXValues = 0;
int totalGyroYValues = 0;
int totalGyroZValues = 0;
int totalAccelXValues = 0;
int totalAccelYValues = 0;
int totalAccelZValues = 0;
const float pi = 3.14159;

unsigned long clockNew,clockOld;

#define address 0x1E
#define ITG3200_Address 0x68
#define alpha 0.8
ADXL345 adxl;

void setup() {
  Wire.begin(); 
  Serial.begin(115200);
  adxl.powerOn();
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(45);
  delay(100);
  initGyro();
  delay(100);
  GyroCalibrate();
  delay(100);
  sensorCalibrate();
  getAcc();
  pitch = yAng;
  roll = xAng;

}

void loop() {
  getGyro();
  calcGyroAng();
  calcCompli();
  Serial.print("GyroX: ");
  Serial.println(gyroXAng);
  Serial.print("GyroY: ");
  Serial.println(gyroYAng);
  Serial.print("GyroZ: ");
  Serial.println(gyroZAng);
  Serial.print("Pitch: ");
  Serial.println(pitch);
  Serial.print("Roll: ");
  Serial.println(roll);
  delay(20);
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

void calcGyroAng(){
  clockNew = millis();
  cycle = clockNew-clockOld;
  gyroXAng = gyroXAng + ((GyroX-biasGyroX)/14.375)*((cycle*1.0)/1000.0);
  gyroYAng = gyroYAng + ((GyroY-biasGyroY)/14.375)*((cycle*1.0)/1000.0);
  gyroZAng = gyroZAng + ((GyroZ-biasGyroZ)/14.375)*((cycle*1.0)/1000.0);
  clockOld = millis();
  delay(10);
}

void sensorCalibrate() {
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

void calcCompli(){
  pitch = alpha*gyroXAng + (1-alpha)*yAng;
  roll = alpha*gyroYAng + (1-alpha)*xAng;
}
