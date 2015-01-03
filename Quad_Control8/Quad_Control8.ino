#define NO_PORTC_PINCHANGES
#define NO_PORTB_PINCHANGES

#include <Servo.h>
#include <Wire.h>
#include <PinChangeInt.h>
#include <TinyGPS++.h>
#include <H2_HMC5883L.h>
#include <H2_ADXL345.h>
#include <H2_ITG3200.h>
#include <H2_BMP180.h>
#include <H2_Registers.h>
#include <H2_TiltComp.h>


#define xMagError 0.94
#define yMagError 0.99
#define zMagError 0.94
#define xMagOffset -90
#define yMagOffset -85
#define zMagOffset 225
#define ROLL_OFFSET 0 //Positive Lowers Aft 4
#define PITCH_OFFSET 0 //Positive Lowers Aft 2
#define YAW_OFFSET 0
#define ROLL_SENSITIVITY 0.5
#define PITCH_SENSITIVITY 0.5
#define YAW_SENSITIVITY 0.25 //Controls the degree at which CH4 affects yaw
#define ROLL_MAXIMUM 1900
#define ROLL_MINIMUM 1100
#define PITCH_MAXIMUM 1900
#define PITCH_MINIMUM 1100
#define THROTTLE_MAXIMUM 1790
#define THROTTLE_MINIMUM 1350
#define YAW_RATE_MAXIMUM 1625
#define YAW_RATE_MINIMUM 1375
#define GPS_ROLL_MAXIMUM 100
#define GPS_PITCH_MAXIMUM 100
#define ACC_SCALAR 0.93
#define ARM_ENGAGE_THRESHOLD 1700
#define ARM_DISENGAGE_THRESHOLD 1100
#define ARM_THROTTLE_THRESHOLD 1100
#define GPS_SATELLITE_MINIMUM 5

#define kpr 0
#define kir 0
#define kdr 0

#define kpp 0
#define kip 0
#define kdp 0

#define kpt 0
#define kit 0
#define kdt 0

#define kpy 0
#define kiy 0
#define kdy 0

#define gpr 0
#define gir 0
#define gdr 0

#define gpp 0
#define gip 0
#define gdp 0

#define altAlpha 0.9
#define compliAlpha 0.98

#define BARO_MODE 3
#define COMPLI_DELAY 5
#define BARO_DELAY 20
#define TEMP_DELAY 1000
#define COMM_DELAY 250
#define ELEV_DELAY 10
#define LAND_DELAY 20000
#define PID_DELAY 5

#define aileronPin A3
#define elevatorPin A2
#define throttlePin A1
#define rudderPin A0
#define channel1 2
#define channel2 3
#define channel3 4
#define channel4 5
#define channel5 6
#define channel6 7

static const double waypoint[] = {39.957016, -75.188874, 3.0,    //Waypoints
                                  39.956952, -75.188233, 3.0,
                                  39.957141, -75.188185, 3.0,
                                  39.957068, -75.188523, 3.0
                                                            };

static const int numWaypoint = ((sizeof(waypoint)/sizeof(const double))/3)-1; 

volatile int channel1Cycle;
volatile int channel2Cycle;
volatile int channel3Cycle;
volatile int channel4Cycle;
volatile int channel5Cycle;
volatile int channel6Cycle;

unsigned long channel1Start,channel2Start,channel3Start,channel4Start,channel5Start,channel6Start;

double channel6Var;

Servo Throttle;
Servo Rudder;
Servo Elevator;
Servo Aileron;


//HMC5883L mag;     
//ADXL345 accel;
//ITG3200 gyro;
//BMP180 baro;
//ORIENTATION_REGISTER orient;
//TARGET_REGISTER target;
//MODE_REGISTER mode;
//PID_REGISTER PID;
TinyGPSPlus gps;

//-------------------------------SETUP-----------------------------------//

void setup(){
  Wire.begin();
  Serial.begin(38400);
  
  pinMode(channel1,INPUT);digitalWrite(channel1,HIGH);PCintPort::attachInterrupt(channel1,&channel1Interrupt,CHANGE);
  pinMode(channel2,INPUT);digitalWrite(channel2,HIGH);PCintPort::attachInterrupt(channel2,&channel2Interrupt,CHANGE);
  pinMode(channel3,INPUT);digitalWrite(channel3,HIGH);PCintPort::attachInterrupt(channel3,&channel3Interrupt,CHANGE);
  pinMode(channel4,INPUT);digitalWrite(channel4,HIGH);PCintPort::attachInterrupt(channel4,&channel4Interrupt,CHANGE);
  pinMode(channel5,INPUT);digitalWrite(channel5,HIGH);PCintPort::attachInterrupt(channel5,&channel5Interrupt,CHANGE);
  pinMode(channel6,INPUT);digitalWrite(channel6,HIGH);PCintPort::attachInterrupt(channel6,&channel6Interrupt,CHANGE);
  
  Aileron.attach(aileronPin);
  Elevator.attach(elevatorPin);
  Throttle.attach(throttlePin);
  Rudder.attach(rudderPin);
  
  pinMode(13,OUTPUT); //Satellite Lock Indicator
  digitalWrite(13,LOW);
   
//  gyro.init();
//  accel.init();
//  mag.init(xMagError, yMagError, zMagError, xMagOffset, yMagOffset, zMagOffset);
//  baro.begin(BARO_MODE, altAlpha);
//  mode.init(0);
  
}

//----------------------------MAIN/SETUP----------------------------------//

void loop(){
  HMC5883L mag;     
  ADXL345 accel;
  ITG3200 gyro;
  BMP180 baro;
  ORIENTATION_REGISTER orient;
  TARGET_REGISTER target;
  MODE_REGISTER mode;
  PID_REGISTER PID;
  
  unsigned long compliClockNew, compliClockOld, baroClockOld, tempClockOld, commClockOld, elevClockOld, landClockOld, PIDClockOld;
  
  gyro.init();
  accel.init();
  mag.init(xMagError, yMagError, zMagError, xMagOffset, yMagOffset, zMagOffset);
  baro.begin(BARO_MODE, altAlpha);
  mode.init(0);
  
  gyro.calibrate();
  
  orient.initAngles(accel, ROLL_OFFSET, PITCH_OFFSET);
  
  target.roll = ROLL_OFFSET;
  target.pitch = PITCH_OFFSET;
  target.throttle = 0.0;
  target.yaw = orient.yaw;
  
  //Reset State Control Timers
  compliClockOld = millis();
  baroClockOld = millis();
  tempClockOld = millis();
  commClockOld = millis();
  elevClockOld = millis();
  landClockOld = millis();
  PIDClockOld = millis();
  
//_______________________________________________________________________// 
//---------------------------Actual Loop---------------------------------//
//_______________________________________________________________________//

  while(1==1){
    checkCompli(gyro, accel, mag, orient, compliClockOld);
    
    checkBaro(baro, baroClockOld, orient);
    
    checkTemp(baro, tempClockOld);
    
    updateGPS(mode, orient, target);
    
    processInterrupts(mode, orient, target, PID);
    
    updatePID(mode, orient, target, PID, PIDClockOld);
    
    transmitCommands(mode, PID);
    
    transmitData(orient, baro, commClockOld);
    
  }
  
}

void checkCompli(class ITG3200 &gyro, class ADXL345 &acc, class HMC5883L &mag, class ORIENTATION_REGISTER &orient, unsigned long &compliClockOld){
   //Main Sensor Reading and Motor Control
  if ((millis() - compliClockOld) >= COMPLI_DELAY){
    compli(gyro, acc, orient, compliClockOld); //Complimentary Filter
    calcYaw(mag, orient, ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET); //Tilt Compensated Compass Code
    
    compliClockOld = millis();
  }
}
 
 void compli(class ITG3200 &gyro, class ADXL345 &accel, class ORIENTATION_REGISTER &orient, unsigned long &compliClockOld){
  //Complimentary Filter to Mix Gyro and Accelerometer Data
  gyro.update();
  accel.update();
  
  double cycle = (((millis() - compliClockOld)*1.0)/1000.0);
  
  double pitchAccel = atan2(-accel.x,accel.z)*(180.0/PI)*ACC_SCALAR + PITCH_OFFSET;
  orient.pitch = compliAlpha * (orient.pitch + (gyro.y) * cycle) + (1 - compliAlpha) * pitchAccel;
  
  double rollAccel = atan2(accel.y,accel.z)*(180.0/PI)*ACC_SCALAR + ROLL_OFFSET;
  orient.roll = compliAlpha * (orient.roll + (gyro.x) * cycle) + (1 - compliAlpha) * rollAccel;
  
}

void checkBaro(class BMP180 &baro, unsigned long &baroClockOld, class ORIENTATION_REGISTER &orient){
   if ((millis() - baroClockOld) >= BARO_DELAY){
    baro.updatePressure();
    baro.calculateAltitude();
    baroClockOld = millis();
  }
 }
 
 void checkTemp(class BMP180 &baro, unsigned long &tempClockOld){
   if ((millis() - tempClockOld) >= TEMP_DELAY){
    baro.updateTemperature();
    tempClockOld = millis();
  }
 }
 
 void transmitData(class ORIENTATION_REGISTER &orient, class BMP180 baro, unsigned long &commClockOld){
   if ((millis() - commClockOld) >= COMM_DELAY){
     Serial.print("Roll: ");
     Serial.println(orient.roll);
     Serial.print("Pitch: ");
     Serial.println(orient.pitch);
     Serial.print("Yaw: ");
     Serial.println(orient.yaw);
     Serial.print("Alt: ");
     Serial.println(baro.alt);
   
     commClockOld = millis();
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

void processInterrupts(class MODE_REGISTER &mode, class ORIENTATION_REGISTER &orient, class TARGET_REGISTER &target, class PID_REGISTER &PID){
   if (channel5Cycle < 1300){
     if (mode.RC_CONTROL_MODE != 1){
       target.alt = waypoint[target.waypointCounter + 2];
       target.intAlt = orient.alt;
       PID.throttle.iTerm = channel3Cycle;
       PID.roll.initial = channel1Cycle;
       PID.pitch.initial = channel2Cycle;
       target.yaw = int(TinyGPSPlus::courseTo(orient.latitude,orient.longitude,waypoint[target.waypointCounter],waypoint[target.waypointCounter + 1]));
       if (target.yaw > 180){target.yaw -= 360;}
     }
     
     mode.RC_CONTROL_MODE = 1;
     
   } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700){
     mode.RC_CONTROL_MODE = 0;
     
   } else if (channel5Cycle > 1700){
     if (mode.RC_CONTROL_MODE != 2){
          target.alt = orient.alt;
          target.intAlt  = orient.alt;
          PID.throttle.iTerm = channel3Cycle;
          target.yaw = orient.yaw;
          PID.roll.initial = channel1Cycle;
          PID.pitch.initial = channel2Cycle;
          PID.roll.iTerm = channel1Cycle;
          PID.pitch.iTerm = channel2Cycle;
          orient.latitude = gps.location.lat();
          orient.longitude = gps.location.lng();
          target.latitude = orient.latitude;
          target.longitude = orient.longitude; 
      }
      
      mode.RC_CONTROL_MODE = 2;
   }
   
   //------------------Mode Select------------------//
   mode.switchModes();
   
   updateTarget(target);
}

void updateTarget(class TARGET_REGISTER &target){
   //------------------Channel6Var------------------//
   channel6Var = 2000.0 * (channel6Var - 1000);
   if (channel6Var < 0.0) {channel6Var = 0.0;}
   
}

void updatePID(class MODE_REGISTER &mode, class ORIENTATION_REGISTER &orient, class TARGET_REGISTER &target, class PID_REGISTER &PID, unsigned long &PIDClockOld){
  if ((millis() - PIDClockOld) >= PID_DELAY){
  
    double PIDCycle = (((millis() - PIDClockOld)*1.0)/1000.0);
    
    if (!mode.CH1_ENABLE){
      PID.roll.control = calcPID(PID.roll, orient, target, 1, 0, 0, PID.roll.initial + GPS_ROLL_MAXIMUM , PID.roll.initial - GPS_ROLL_MAXIMUM, gpr, gir, gdr, PIDCycle);
    }
    if (!mode.CH2_ENABLE){
      PID.pitch.control = calcPID(PID.pitch, orient, target, 2, 0, 0, PID.pitch.initial + GPS_PITCH_MAXIMUM , PID.pitch.initial - GPS_PITCH_MAXIMUM, gpp, gip, gdp, PIDCycle);
    }
    if (!mode.CH3_ENABLE){
      target.incAlt();
      PID.throttle.control = calcPID(PID.throttle, orient, target, 0, target.intAlt, orient.alt, THROTTLE_MAXIMUM, THROTTLE_MINIMUM, kpt, kit, kdt, PIDCycle);
    }
    if (!mode.CH4_ENABLE){
      PID.yaw.control = calcPID(PID.yaw, orient, target, 0, target.yaw, orient.yaw, YAW_RATE_MAXIMUM, YAW_RATE_MINIMUM, kpy, kiy, kdy, PIDCycle);
    }
    
    PIDClockOld = millis();
  }
}

int calcPID(class PID_SINGLE &motion, class ORIENTATION_REGISTER orient, class TARGET_REGISTER target, int type, double targetBasic, double currPos, const double maximum, const double minimum, const double kp, const double ki, const double kd, double cycle){
  //Type Contol Parameter
  //0 - Basic PID
  //1 - GPS Roll PID
  //2 - GPS Pitch PID
  
  double error;
  
  switch (type){
   case 0:
     {
     error = targetBasic - currPos;
     }
     break;
     
   case 1:
     {
     double errorLongitude = target.longitude - orient.longitude;
     double errorLatitude = target.latitude - orient.latitude;
     
     error = (errorLongitude * cos(radians(orient.yaw)) + errorLatitude * sin(radians(orient.yaw)));
     }
     break;
     
   case 2:
     {
     double errorLongitude = target.longitude - orient.longitude;
     double errorLatitude = target.latitude - orient.latitude;
     
     error = -(errorLatitude * cos(radians(orient.yaw)) + errorLongitude * sin(radians(orient.yaw)));
     }
     break;
  };
  
  //int error = target - currPos;
  motion.iTerm += (ki * 0.001 * cycle * error);
  if (motion.iTerm > maximum){motion.iTerm = maximum;}
  else if (motion.iTerm < minimum){motion.iTerm = minimum;}
  double control = kp * error + motion.iTerm - ((kd * (currPos - motion.lastPos))/cycle);
  if (control > maximum) {control = maximum;}
  if (control < minimum) {control = minimum;}
  motion.lastPos = currPos;
  int controlOut = int(control);
  
  return(controlOut);
}

void transmitCommands(class MODE_REGISTER mode, class PID_REGISTER PID){
  //NEEDS PID Structs/Classes!
  if (mode.CH1_ENABLE){
    Aileron.writeMicroseconds(channel1Cycle);
  } else {
    Aileron.writeMicroseconds(PID.GPSRoll.control);
  }
  
  if (mode.CH2_ENABLE){
    Elevator.writeMicroseconds(channel2Cycle);
  } else {
    Elevator.writeMicroseconds(PID.GPSPitch.control);
  }
  
  if (mode.CH3_ENABLE){
    Throttle.writeMicroseconds(channel3Cycle);
  } else {
    Throttle.writeMicroseconds(PID.throttle.control);
  }
  
  if (mode.CH4_ENABLE){
    Rudder.writeMicroseconds(channel4Cycle);
  } else {
    Rudder.writeMicroseconds(PID.yaw.control);
  }
}

void updateGPS(class MODE_REGISTER mode, class ORIENTATION_REGISTER &orient, class TARGET_REGISTER &target){
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      orient.latitude = gps.location.lat();
      orient.longitude = gps.location.lng();
      
      if (mode.GPS_NAV_ENABLE){
        target.alt = waypoint[target.waypointCounter + 2];
        target.yaw = int(TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),waypoint[target.waypointCounter],waypoint[target.waypointCounter + 1]));
        if (target.yaw > 180){target.yaw -= 360;}
      }
      
      distanceCheck(orient, target);
      
      if (gps.satellites.value() >= GPS_SATELLITE_MINIMUM){ //GPS Lock Indicator
        digitalWrite(13,HIGH);
      } else {
        digitalWrite(13,LOW);
      }
    }
  }
}

void distanceCheck(class ORIENTATION_REGISTER orient, class TARGET_REGISTER &target){
  //Monitors Distance to Waypoints and updates them when the quad arrives
  int distanceToWaypoint = int(TinyGPSPlus::distanceBetween(orient.latitude,orient.longitude,waypoint[target.waypointCounter],waypoint[target.waypointCounter + 1]));
  if(distanceToWaypoint <= 3){
    target.waypointCounter += 3;
    if ((target.waypointCounter/3) > numWaypoint && (abs(target.alt - orient.alt)) < 2){
     target.waypointCounter = 0; 
     //Land Code Here Perhaps
    }
  }
}
