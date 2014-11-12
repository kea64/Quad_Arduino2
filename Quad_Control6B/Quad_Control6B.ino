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
#define GPSAlpha 1.000000

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
#define kip 0

#define kpr 250000
#define kir 0

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

unsigned long channel1Start,channel2Start,channel3Start,channel4Start,channel5Start,channel6Start;

double channel6Var;

static const double waypoint[] = {39.957016, -75.188874, 3.0,    //Waypoints
                                  39.956952, -75.188233, 3.0,
                                  39.957141, -75.188185, 3.0,
                                  39.957068, -75.188523, 3.0
                                                            };

static const int numWaypoint = ((sizeof(waypoint)/sizeof(const double))/3)-1;                                                           
     
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
   
   pinMode(13,OUTPUT); //GPS Lock Indicator
   digitalWrite(13,LOW);
  
   //Servo Read Initialize for 6 Channels
  if (RC_ENABLE == 1){
    pinMode(channel1,INPUT);digitalWrite(channel1,HIGH);PCintPort::attachInterrupt(channel1,&channel1Interrupt,CHANGE);
    pinMode(channel2,INPUT);digitalWrite(channel2,HIGH);PCintPort::attachInterrupt(channel2,&channel2Interrupt,CHANGE);
    pinMode(channel3,INPUT);digitalWrite(channel3,HIGH);PCintPort::attachInterrupt(channel3,&channel3Interrupt,CHANGE);
    pinMode(channel4,INPUT);digitalWrite(channel4,HIGH);PCintPort::attachInterrupt(channel4,&channel4Interrupt,CHANGE);
    pinMode(channel5,INPUT);digitalWrite(channel5,HIGH);PCintPort::attachInterrupt(channel5,&channel5Interrupt,CHANGE);
    pinMode(channel6,INPUT);digitalWrite(channel6,HIGH);PCintPort::attachInterrupt(channel6,&channel6Interrupt,CHANGE);
  }
   
}

//----------------------------MAIN/SETUP----------------------------------//


void loop(){
  double gyroX, gyroY, gyroZ, accX, accY, accZ, gyroOffsetX, gyroOffsetY, gyroOffsetZ, pitch, roll, cycle, yaw, latitude, longitude, targetLatitude, targetLongitude;
  double ITermT, ITermP, ITermR;
  double baseline, T, P, p0, a;
  double alt = 0.0;
  double targetAlt = 1.0;
  double targetIntAlt = 1.0;
  
  unsigned long compliClockNew, compliClockOld, baroClockOld, tempClockOld, commClockOld, elevClockOld, landClockOld;

  int aX, aY, aZ;
  int targetHeading;
  int elevatorInitial, aileronInitial;
  int RC_CONTROL_MODE = 0;
  int waypointCounter = 0;
  
  bool AUTO_ROLL = 0;
  bool AUTO_PITCH = 0;
  bool AUTO_YAW = 0;
  bool AUTO_THROTTLE = 0;
  
  calibrateGyro(gyroX, gyroY, gyroZ, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  
  pressure.begin();
  updateTemp(T);
  updateBaro(P, T);
  baseline = P;
  
  //Reset State Control Timers
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
    
    checkBaro(P, T, baseline, alt, baroClockOld);
    
    checkTemp(T, tempClockOld);
    
    processInterrupts(RC_CONTROL_MODE, AUTO_ROLL, AUTO_PITCH, AUTO_THROTTLE, AUTO_YAW, yaw, latitude, longitude, alt, waypointCounter, targetAlt, targetIntAlt, ITermT, ITermR, ITermP, targetHeading, aileronInitial, elevatorInitial, targetLatitude, targetLongitude);
    
    checkDistance(waypointCounter, targetAlt, alt);
  }
}

//----------------------------Functions----------------------------------//
//_______________________________________________________________________//

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

void calibrateGyro(double &gyroX, double &gyroY, double &gyroZ, double &gyroOffsetX, double &gyroOffsetY, double &gyroOffsetZ){
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

void updateBaro(double &P, double &T){
  char status = pressure.startPressure(BARO_MODE);
  if (status != 0){
    delay(status);
    status = pressure.getPressure(P,T);
  }      
}

void updateTemp(double &T){
  char status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
  }
}

void updateGPS(double &latitude, double &longitude){
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      
      if (gps.location.lat() != 0.0){
        if (millis() < 2000){
          latitude = gps.location.lat();
          longitude = gps.location.lng();
        } else {
          latitude = gps.location.lat() * GPSAlpha + (1 - GPSAlpha) * latitude;
          longitude = gps.location.lng() * GPSAlpha + (1 - GPSAlpha) * longitude;
        }
      }
      
      if (gps.satellites.value() >= GPS_SATELLITE_MINIMUM){ //GPS Lock Indicator
        digitalWrite(13,HIGH);
      } else {
        digitalWrite(13,LOW);
      }
    }
  }
}

void compli(double &gyroX, double &gyroY, double &gyroZ, double gyroOffsetX, double gyroOffsetY, double gyroOffsetZ, double &accX, double &accY, double &accZ, int &aX, int &aY, int &aZ, double &roll, double &pitch, unsigned long compliClockOld){
  //Complimentary Filter to Mix Gyro and Accelerometer Data
  double cycle = (((millis() - compliClockOld)*1.0)/1000.0);
  updateGyro(gyroX, gyroY, gyroZ, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  updateAcc(accX, accY, accZ, aX, aY, aZ);
  
  double pitchAccel = atan2(-accX,accZ)*(180.0/PI)*ACC_SCALAR + PITCH_OFFSET;
  pitch = compliAlpha * (pitch + ((gyroX)/14.375) * cycle) + (1 - compliAlpha) * pitchAccel;
  
  double rollAccel = atan2(accY,accZ)*(180.0/PI)*ACC_SCALAR + ROLL_OFFSET;
  roll = compliAlpha * (roll + ((gyroY)/14.375) * cycle) + (1 - compliAlpha) * rollAccel;
  
  compliClockOld = millis();
}

void calcYaw(double roll, double pitch, double &yaw){
  updateMag();
  double CMx = compass_x_scalled * cos(radians(pitch-PITCH_OFFSET)) + compass_z_scalled * sin(radians(pitch-PITCH_OFFSET)); //Adjusts mX reading
  double CMy = compass_x_scalled * sin(radians(roll-ROLL_OFFSET)) * sin(radians(pitch-PITCH_OFFSET)) + compass_y_scalled * cos(radians(roll-ROLL_OFFSET)) - compass_z_scalled * sin(radians(roll-ROLL_OFFSET)) * cos(radians(pitch-PITCH_OFFSET)); //Adjusts mY Reading
  yaw = atan2(CMy,CMx) - radians(YAW_OFFSET);
  if (yaw < 0){yaw += 2*PI;}
  if (yaw > 2*PI) {yaw -= 2*PI;}
  yaw = yaw * (180/PI);
  if (yaw <= 360 && yaw > 180) {yaw -= 360;}
}

void calcAlt(double P, double baseline, double &alt){
  double a = pressure.altitude(P,baseline);
  
  alt = altAlpha * alt + (1-altAlpha) * a; //Heavy Barometer Filtering
}

void checkDistance(int &waypointCounter, double targetAlt, double alt){
  //Monitors Distance to Waypoints and updates them when the quad arrives
  int distanceToWaypoint = int(TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),waypoint[waypointCounter],waypoint[waypointCounter + 1]));
  if(distanceToWaypoint <= 3){
    waypointCounter += 3;
    if ((waypointCounter/3) > numWaypoint && (abs(targetAlt - alt)) < 2){
     waypointCounter = 0; 
     //Land Code Here Perhaps
    }
  }
}

void channel1Interrupt(){
  if (digitalRead(channel1) == 1){
      channel1Start = micros();
  } else {
      channel1Cycle = micros() - channel1Start;
  }
}

void channel2Interrupt(){
  if (digitalRead(channel2) == 1){
      channel2Start = micros();
  } else {
      channel2Cycle = micros() - channel2Start;
  }
}

void channel3Interrupt(){
  if (digitalRead(channel3) == 1){
      channel3Start = micros();
  } else {
      channel3Cycle = micros() - channel3Start;
  }
}

void channel4Interrupt(){
  if (digitalRead(channel4) == 1){
      channel4Start = micros();
  } else {
      channel4Cycle = micros() - channel4Start;
  }
}

void channel5Interrupt(){
  if (digitalRead(channel5) == 1){
      channel5Start = micros();
  } else {
      channel5Cycle = micros() - channel5Start;
  }
}

void channel6Interrupt(){
  if (digitalRead(channel6) == 1){
      channel6Start = micros();
  } else {
      channel6Cycle = micros() - channel6Start;
  }
}

void processInterrupts(int &RC_CONTROL_MODE, bool &AUTO_ROLL, bool &AUTO_PITCH, bool &AUTO_THROTTLE, bool &AUTO_YAW, double yaw, double latitude, double longitude, double alt, int waypointCounter, double &targetAlt, double &targetIntAlt, double &ITermT, double &ITermR, double &ITermP, int &targetHeading, int &aileronInitial, int &elevatorInitial, double &targetLatitude, double &targetLongitude){
   if (channel5Cycle < 1300){
     if (RC_CONTROL_MODE != 1){
       targetAlt = waypoint[waypointCounter + 2];
       targetIntAlt = alt;
       ITermT = channel3Cycle;
       aileronInitial = channel1Cycle;
       elevatorInitial = channel2Cycle;
       targetHeading = int(TinyGPSPlus::courseTo(latitude,longitude,waypoint[waypointCounter],waypoint[waypointCounter + 1]));
       if (targetHeading > 180){targetHeading -= 360;}
     }
     
     RC_CONTROL_MODE = 1;
     
   } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700){
     RC_CONTROL_MODE = 0;
     
   } else if (channel5Cycle > 1700){
     if (RC_CONTROL_MODE != 2){
          targetAlt = alt;
          targetIntAlt = alt;
          ITermT = channel3Cycle;
          targetHeading = yaw;
          aileronInitial = channel1Cycle;
          elevatorInitial = channel2Cycle;
          ITermR = channel1Cycle;
          ITermP = channel2Cycle;
          latitude = gps.location.lat();
          longitude = gps.location.lng();
          targetLatitude = latitude;
          targetLongitude = longitude;
      }
      
      RC_CONTROL_MODE = 2;
      
      
   }
   
   //------------------Mode Select------------------//
   switchModes(RC_CONTROL_MODE, AUTO_ROLL, AUTO_PITCH, AUTO_THROTTLE, AUTO_YAW);
   
   //------------------Channel6Var------------------//
   channel6Var = 2000.0 * (channel6Var - 1000);
   if (channel6Var < 0.0) {channel6Var = 0.0;}
   
   //-------------------Channel 1-------------------//
   if (!AUTO_ROLL){
     Aileron.writeMicroseconds(channel1Cycle);
   }
   
   //-------------------Channel 2-------------------//
   if (!AUTO_PITCH){
     Elevator.writeMicroseconds(channel2Cycle);
   }
   
   //-------------------Channel 3-------------------//
   if (!AUTO_THROTTLE){
     Throttle.writeMicroseconds(channel3Cycle);
   }
   
   //-------------------Channel 4-------------------//
   if (!AUTO_YAW){
     Rudder.writeMicroseconds(channel4Cycle);
   }
   
 }
 
 void switchModes(int RC_CONTROL_MODE, bool &AUTO_ROLL, bool &AUTO_PITCH, bool &AUTO_THROTTLE, bool &AUTO_YAW){
   switch (RC_CONTROL_MODE){
     case 0:
       AUTO_ROLL = 0;
       AUTO_PITCH = 0;
       AUTO_THROTTLE = 0;
       AUTO_YAW = 0;
       break;
       
     case 1:
       AUTO_ROLL = 0;
       AUTO_PITCH = 0;
       AUTO_THROTTLE = 1;
       AUTO_YAW = 1;
       break;
       
      case 2:
       AUTO_ROLL = 1;
       AUTO_PITCH = 1;
       AUTO_THROTTLE = 1;
       AUTO_YAW = 1;
       break;
   }
 }
 
 void checkBaro(double P, double T, double baseline, double alt, unsigned long baroClockOld){
   if ((millis() - baroClockOld) >= BARO_DELAY){
    updateBaro(P, T);
    calcAlt(P, baseline, alt);
    baroClockOld = millis();
  }
 }
 
 void checkTemp(double T, unsigned long tempClockOld){
   if ((millis() - tempClockOld) >= TEMP_DELAY){
    updateTemp(T);
    tempClockOld = millis();
  }
 }
 
 void checkCompli(){
   //Main Sensor Reading and Motor Control
  if ((millis() - compliClockOld) >= COMPLI_DELAY){
    compli(); //Complimentary Filter
    calcYaw(); //Tilt Compensated Compass Code
    //GPS Navigation Mode
    if (RC_CONTROL_MODE == 2 || RC_ENABLE != 1){
        yawUpdate(); // Yaw Control for navigation
        //If GPS Locked, Enable Translation Mode
        if (gps.satellites.value() > GPS_SATELLITE_MINIMUM){
            FORWARD_MODE_ENABLE = 1;
            STRAFE_MODE_ENABLE = 1;
            translationUpdate(); //Roll + Pitch Control for navigation   
        } else {
          FORWARD_MODE_ENABLE = 0;
          STRAFE_MODE_ENABLE = 0;
        }
    }
    //Following statement may be deleted after tuning translation
    if (RC_CONTROL_MODE == 1){
      yawUpdate();
      FORWARD_MODE_ENABLE = 0;
      STRAFE_MODE_ENABLE = 0;
    }
  }
 }
