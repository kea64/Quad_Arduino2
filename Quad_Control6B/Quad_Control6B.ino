#include <Servo.h>
#include <Wire.h>
#include <ADXL345.h>
#include <SFE_BMP180.h>
#include <PinChangeInt.h>
#include <TinyGPS++.h>
#include <compass.h>

#define ITG3200_Address 0x68
#define address 0x1E
#define COMPLI_DELAY 5
#define BARO_DELAY 20
#define TEMP_DELAY 1000
#define COMM_DELAY 250
#define ELEV_DELAY 10
#define LAND_DELAY 20000
#define accConv 0.0039
#define ACC_SCALAR 0.93
#define FREE_FALL_THRESHOLD 8
#define FREE_FALL_DURATION 20
#define ESC_SCALING 1
#define ESC_MIN 90
#define ESC_MAX 180
#define ROLL_OFFSET 0 //Positive Lowers Aft 4
#define PITCH_OFFSET 0 //Positive Lowers Aft 2
#define YAW_OFFSET 0
#define altAlpha 0.9
#define compliAlpha 0.98

#define BARO_MODE 3
#define MAX_ALTITUDE 15.0
#define IRPin 0
#define IRAlpha 0.8
#define MAX_YAW 20.0
#define MAX_ROLL 100.0
#define MAX_PITCH 100.0
#define MAX_THROTTLE 1790
#define MIN_THROTTLE 1350

//#define kpy 0.2
//#define kiy 15
//#define kdy 0.1

#define kpy 0.5
#define kiy 0.2
#define kdy 0

//#define kpt 5.5
//#define kit 0.2
//#define kdt 5

#define kpt 11.5
#define kit 1.5
#define kdt 15

#define kpp 250000
#define kip 100000

#define kpr 250000
#define kir 100000

#define aileronPin A3
#define elevatorPin A2
#define throttlePin A1
#define rudderPin A0
#define channel1 4
#define channel2 5
#define channel3 6
#define channel4 7
#define channel5 8
#define channel6 9

#define RC_ENABLE 1
#define GPS_SATELLITE_MINIMUM 5

volatile int channel1Cycle;
volatile int channel2Cycle;
volatile int channel3Cycle;
volatile int channel4Cycle;
volatile int channel5Cycle;
volatile int channel6Cycle;

int RC_CONTROL_MODE = 0;

boolean STRAFE_MODE_ENABLE, FORWARD_MODE_ENABLE;

unsigned long channel1Start,channel2Start,channel3Start,channel4Start,channel5Start,channel6Start;

ADXL345 adxl;
TinyGPSPlus gps;
SFE_BMP180 pressure;

Servo Throttle;
Servo Rudder;
Servo Elevator;
Servo Aileron;

//-------------------------------SETUP-----------------------------------//

void setup(){  
   Wire.begin(); 
   Serial.begin(38400);
   
   //Peripheral Setup
   initGyro();
   initAcc();
   initMag();
   initServo();
   pressure.begin();
   
}

//----------------------------MAIN/SETUP----------------------------------//


void loop(){
  double gyroX, gyroY, gyroZ, accX, accY, accZ, gyroOffsetX, gyroOffsetY, gyroOffsetZ, pitch, roll, cycle;
  
  unsigned long compliClockNew, compliClockOld, baroClockOld, tempClockOld, commClockOld, elevClockOld, landClockOld;

  int aX, aY, aZ;
  int tempVar = 1;
  
  gyroCalibrate(gyroX, gyroY, gyroZ, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  
  //Servo Read Initialize for 6 Channels
  if (RC_ENABLE == 1){
    pinMode(channel1,INPUT);digitalWrite(channel1,HIGH);PCintPort::attachInterrupt(channel1,&channel1Update,CHANGE);
    pinMode(channel2,INPUT);digitalWrite(channel2,HIGH);PCintPort::attachInterrupt(channel2,&channel2Update,CHANGE);
    pinMode(channel3,INPUT);digitalWrite(channel3,HIGH);PCintPort::attachInterrupt(channel3,&channel3Update,CHANGE);
    pinMode(channel4,INPUT);digitalWrite(channel4,HIGH);PCintPort::attachInterrupt(channel4,&channel4Update,CHANGE);
    //pinMode(channel5,INPUT);digitalWrite(channel5,HIGH);PCintPort::attachInterrupt(channel5,&channel5Update,CHANGE);
    //pinMode(channel6,INPUT);digitalWrite(channel6,HIGH);PCintPort::attachInterrupt(channel6,&channel6Update,CHANGE);
  }
  
  pinMode(13,OUTPUT); //GPS Lock Indicator
  digitalWrite(13,LOW);
  
  //Initialize State Control Timers
  compliClockOld = millis();
  baroClockOld = millis();
  tempClockOld = millis();
  commClockOld = millis();
  elevClockOld = millis();
  landClockOld = millis();
  
//_______________________________________________________________________// 
//---------------------------Actual Loop---------------------------------//
//_______________________________________________________________________//

  while(1==1){
    
    updateGyro(gyroX, gyroY, gyroZ, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    updateAcc(accX, accY, accZ, aX, aY, aZ);
    
  }
}

//----------------------------Functions----------------------------------//

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

void initAcc(){
  adxl.powerOn();
  adxl.setFreeFallThreshold(FREE_FALL_THRESHOLD); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(FREE_FALL_DURATION); //(20 - 70) recommended - 5ms per increment
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
}

void initMag(){
  compass_x_offset = -90;
  compass_y_offset = -85;
  compass_z_offset = 225;
  compass_x_gainError = 0.94;
  compass_y_gainError = 0.99;
  compass_z_gainError = 0.94;
  
  compass_init(2);
}

void gyroCalibrate(double &gyroX, double &gyroY, double &gyroZ, double &gyroOffsetX, double &gyroOffsetY, double &gyroOffsetZ){
 int tmpx = 0;
 int tmpy = 0;
 int tmpz = 0; 

 for (char i = 0;i<10;i++)
    {
    delay(10);  
    updateGyro(gyroX, gyroY, gyroZ, 0, 0, 0);
    tmpx += gyroX;
    tmpy += gyroY;
    tmpz += gyroZ; 
    }  
 gyroOffsetX = tmpx/10;
 gyroOffsetY = tmpy/10;
 gyroOffsetZ = tmpz/10;
 
}

void initAngles(double &accX, double &accY, double &accZ, int &aX, int &aY, int &aZ, double &roll, double &pitch){
  updateAcc(accX, accY, accZ, aX, aY, aZ); //Obtains Initial Angles; Quad must be motionless
  roll = atan2(accY,accZ)*(180/PI) + ROLL_OFFSET; //Accounts for Angle Differences
  pitch = atan2(-accX,accZ)*(180/PI) + PITCH_OFFSET; 
}

void initServo(){
  Aileron.attach(aileronPin);
  Elevator.attach(elevatorPin);
  Throttle.attach(throttlePin);
  Rudder.attach(rudderPin);
}

void updateGyro(double &gyroX, double &gyroY, double &gyroZ, int gyroOffsetX, int gyroOffsetY, int gyroOffsetZ) {
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
    
  gyroX = ((buff[4] << 8) | buff[5]) - gyroOffsetX;
  gyroY = ((buff[2] << 8) | buff[3]) - gyroOffsetY;
  gyroZ = ((buff[6] << 8) | buff[7]) - gyroOffsetZ;
  
}

void updateAcc(double &accX, double &accY, double &accZ, int &aX, int &aY, int &aZ){
  adxl.readAccel(&aX, &aY, &aZ);
  accX = aX * accConv;
  accY = aY * accConv;
  accZ = aZ * accConv;
}

void updateMag(){
  compass_scalled_reading();
}

/*
void compli(double &roll, double &pitch, double &compliClockNew, double &compliClockOld, double &cycle){
  //Complimentary Filter to Mix Gyro and Accelerometer Data
  compliClockNew = millis(); //Cycle Timing Code
  cycle = (((compliClockNew - compliClockOld)*1.0)/1000.0);
  getGyro();
  getAcc();
  
  double pitchAccel = degrees(atan2(-accX,accZ))*ACC_SCALAR + PITCH_OFFSET;
  pitch = compliAlpha * (pitch + ((GyroX)/14.375) * cycle) + (1 - compliAlpha) * pitchAccel;
  
  double rollAccel = degrees(atan2(accY,accZ))*ACC_SCALAR + ROLL_OFFSET;
  roll = compliAlpha * (roll + ((GyroY)/14.375) * cycle) + (1 - compliAlpha) * rollAccel;
  
  compliClockOld = millis();
}
*/

void channel1Update(){
  if (RC_CONTROL_MODE == 0 || RC_CONTROL_MODE == 1 || (STRAFE_MODE_ENABLE == 0 && RC_CONTROL_MODE == 2)){
    if (digitalRead(channel1) == 1){
      channel1Start = micros();
    } else {
      channel1Cycle = micros() - channel1Start;
      Aileron.writeMicroseconds(channel1Cycle);
    }
  } else if (RC_CONTROL_MODE == 2 && STRAFE_MODE_ENABLE == 1){
    //Special Code Here?
  }
  *tempVar++;
}

void channel2Update(){
  if (RC_CONTROL_MODE == 0 || RC_CONTROL_MODE == 1 || (FORWARD_MODE_ENABLE == 0 && RC_CONTROL_MODE == 2)){
    if (digitalRead(channel2) == 1){
      channel2Start = micros();
    } else {
      channel2Cycle = micros() - channel2Start;
      Elevator.writeMicroseconds(channel2Cycle);
    }
  } else if (RC_CONTROL_MODE == 2 && FORWARD_MODE_ENABLE == 1){
    //Special Code Here?
  }
}

void channel3Update(){
  
    //NOTE: May need rephrasing for maximizing other processes. Still not sure if necessary to run indefinitely  

    if (digitalRead(channel3) == 1){
      channel3Start = micros();
    } else {
      channel3Cycle = micros() - channel3Start;
    }
    
    if (RC_CONTROL_MODE == 0){
      Throttle.writeMicroseconds(channel3Cycle);
    } else if (RC_CONTROL_MODE == 2){
      //Extra mode Here
    }
}

void channel4Update(){
  if (RC_CONTROL_MODE == 0){
    if (digitalRead(channel4) == 1){
      channel4Start = micros();
    } else {
      channel4Cycle = micros() - channel4Start;
      Rudder.writeMicroseconds(channel4Cycle);
    }
  }
}

/*
void channel5Update(){
  
    if (digitalRead(channel5) == 1){
      channel5Start = micros();
    } else {
      channel5Cycle = micros() - channel5Start;
      if (channel5Cycle < 1300){
        if (RC_CONTROL_MODE != 1){
          targetAlt = alt;
          targetIntAlt = alt;
          ITermT = channel3Cycle;
          aileronInitial = channel1Cycle;
          elevatorInitial = channel2Cycle;
        }
        RC_CONTROL_MODE = 1;
        
      } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700) {
        RC_CONTROL_MODE = 0; //For Safety Purposes
        
      } else if (channel5Cycle > 1700) {
        if (RC_CONTROL_MODE != 2){
          targetAlt = alt;
          targetIntAlt = alt;
          ITermT = channel3Cycle;
          targetHeading = yaw;
          aileronInitial = channel1Cycle;
          elevatorInitial = channel2Cycle;
          targetLatitude = gps.location.lat();
          targetLongitude = gps.location.lng();
        }
        RC_CONTROL_MODE = 2;
      }
        
    }
}

void channel6Update(){
  if (RC_CONTROL_MODE == 2 || RC_CONTROL_MODE == 1){
    if (digitalRead(channel6) == 1){
      channel6Start = micros();
    } else {
      channel6Cycle = micros() - channel6Start;
      channel6Var = 2000.0 * (channel6Cycle - 1000);
      if (channel6Var < 0.0) { channel6Var = 0.0;}
    }
    
  }
}
*/
