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
#include <H2_Channel.h>


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
#define ROLL_MAXIMUM 45
#define ROLL_MINIMUM -45
#define PITCH_MAXIMUM 45
#define PITCH_MINIMUM -45
#define THROTTLE_MAXIMUM 90
#define THROTTLE_MINIMUM -90
#define YAW_MAXIMUM 45
#define YAW_MINIMUM -45
#define GPS_ROLL_MAXIMUM 20
#define GPS_PITCH_MAXIMUM 20
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
  MODE2_REGISTER mode;
  PID_REGISTER PID;
  CHANNEL_REGISTER CHANNEL;
  
  unsigned long compliClockNew, compliClockOld, baroClockOld, tempClockOld, commClockOld, elevClockOld, landClockOld, PIDClockOld;
  
  gyro.init();
  accel.init();
  mag.init(xMagError, yMagError, zMagError, xMagOffset, yMagOffset, zMagOffset);
  baro.begin(BARO_MODE, altAlpha);
  mode.switchModes(0);
  
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
    //Sensor Updates
    checkCompli(gyro, accel, mag, orient, compliClockOld);
    checkBaro(baro, baroClockOld, orient);
    checkTemp(baro, tempClockOld);
    updateGPS(mode, orient, target);
    
    //Interpretation and Processing
    processGPS(mode, orient, target);
    
    //Motor Output
    updatePID(mode, orient, target, CHANNEL, PIDClockOld);
    
    //Status Feedback
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

void processInterrupts2(class CHANNEL_REGISTER &CHANNEL, class MODE2_REGISTER &mode, class TARGET_REGISTER &target){
  bool newMode = 0;
  
  if (channel5Cycle < 1300){
    if (mode.RC_CONTROL_MODE != 1){
      mode.RC_CONTROL_MODE = 1;
      initializeAuto(mode, target);
    }
    mode.switchModes(1);
    
  } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700){
    if (mode.RC_CONTROL_MODE != 0){
      mode.RC_CONTROL_MODE = 0;
    }
    mode.switchModes(0);
    
  } else if (channel5Cycle > 1700){
    if (mode.RC_CONTROL_MODE != 2){
      mode.RC_CONTROL_MODE = 2;
    }
    mode.switchModes(2);
    
  }
  
  
}

void motorController(){
  
 //Enter Special Code Here to Take Non-Auxiliary Code and Output Tailored Controls
 
}

void updatePID(class MODE2_REGISTER &mode, class ORIENTATION_REGISTER &orient, class TARGET_REGISTER &target, class CHANNEL_REGISTER &CHANNEL, unsigned long &PIDClockOld){
  if ((millis() - PIDClockOld) >= PID_DELAY){
    
    double PIDCycle = (((millis() - PIDClockOld)*1.0)/1000.0);
    
    // Manual PID Target Set
    int targetManualRoll;
    int targetManualPitch;
    int targetManualThrottle;
    int targetManualYaw;
    
    int targetAutoRoll;
    int targetAutoPitch;
    int targetAutoThrottle;
    int targetAutoYaw;
    
    if (mode.CH1_AUTO_EN){
      double errorGPSRoll = (target.longitude * cos(radians(orient.yaw)) + target.latitude * sin(radians(orient.yaw)));
      calcPID(CHANNEL.AUTO_ROLL, orient, target, errorGPSRoll, orient.roll, CHANNEL.ROLL.initial + GPS_ROLL_MAXIMUM , CHANNEL.ROLL.initial - GPS_ROLL_MAXIMUM, gpr, gir, gdr, PIDCycle);
      targetManualRoll = CHANNEL.AUTO_ROLL.control;
    } else {
      targetManualRoll = CHANNEL.servo2HalfDegrees(channel1Cycle);
    }
    
    if (mode.CH2_AUTO_EN){
      double errorGPSPitch = -(target.latitude * cos(radians(orient.yaw)) + target.longitude * sin(radians(orient.yaw)));
      calcPID(CHANNEL.AUTO_PITCH, orient, target, errorGPSPitch, orient.pitch, CHANNEL.PITCH.initial + GPS_PITCH_MAXIMUM , CHANNEL.PITCH.initial - GPS_PITCH_MAXIMUM, gpp, gip, gdp, PIDCycle);
      targetManualPitch = CHANNEL.AUTO_PITCH.control;
    } else {
      targetManualPitch = CHANNEL.servo2HalfDegrees(channel2Cycle);
    }
    
    if (!mode.AUXI_EN){
      //KRIS- DOUBLE CHECK THIS BEFORE RUNNING
      double errorRoll = targetManualRoll - orient.roll;
      double errorPitch = targetManualPitch - orient.pitch;
      double errorThrottle = targetManualThrottle - orient.throttle;
      double errorYaw = targetManualYaw - orient.yaw;
      
      calcPID(CHANNEL.ROLL, orient, target, errorRoll, orient.roll, ROLL_MAXIMUM , ROLL_MINIMUM, kpr, kir, kdr, PIDCycle);
      calcPID(CHANNEL.PITCH, orient, target, errorPitch, orient.pitch, PITCH_MAXIMUM , PITCH_MINIMUM, kpp, kip, kdp, PIDCycle);
      calcPID(CHANNEL.THROTTLE, orient, target, errorThrottle, orient.throttle, THROTTLE_MAXIMUM, THROTTLE_MINIMUM, kpt, kit, kdt, PIDCycle);
      calcPID(CHANNEL.YAW, orient, target, errorYaw, orient.yaw, YAW_MAXIMUM, YAW_MINIMUM, kpy, kiy, kdy, PIDCycle);
    }
    
    PIDClockOld = millis();
  } 
}

void calcPID(class CHANNEL &motion, class ORIENTATION_REGISTER orient, class TARGET_REGISTER target, double error, double currPos, const double maximum, const double minimum, const double kp, const double ki, const double kd, double cycle){
  
  //int error = target - currPos;
  motion.iTerm += (ki * 0.001 * cycle * error);
  if (motion.iTerm > maximum){motion.iTerm = maximum;}
  else if (motion.iTerm < minimum){motion.iTerm = minimum;}
  double control = kp * error + motion.iTerm - ((kd * (currPos - motion.lastPos))/cycle);
  if (control > maximum) {control = maximum;}
  if (control < minimum) {control = minimum;}
  motion.lastPos = currPos;
  motion.control = int(control);
  
}

void updateGPS(class MODE2_REGISTER mode, class ORIENTATION_REGISTER &orient, class TARGET_REGISTER &target){
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      orient.latitude = gps.location.lat();
      orient.longitude = gps.location.lng();
      
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

void processGPS(class MODE2_REGISTER mode, class ORIENTATION_REGISTER &orient, class TARGET_REGISTER &target){ 
   if (mode.CH1_AUTO_EN || mode.CH2_AUTO_EN){
     if (mode.CH1_HOLD_EN || mode.CH2_HOLD_EN){
       target.latitude = target.holdLatitude - orient.latitude;
       target.longitude = target.holdLongitude - orient.longitude;
     } else {
       target.latitude = waypoint[target.waypointCounter] - orient.latitude;
       target.longitude = waypoint[target.waypointCounter + 1] - orient.longitude;
     }
   } 
  
   if (mode.CH3_AUTO_EN){
     if (mode.CH3_HOLD_EN){
       target.alt = target.holdAlt;
     } else {
       target.alt = waypoint[target.waypointCounter + 2];
     }
   }
   
   if (mode.CH4_AUTO_EN){
     if (mode.CH4_HOLD_EN){
       target.yaw = target.holdYaw;
     } else {
       target.yaw = int(TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),waypoint[target.waypointCounter],waypoint[target.waypointCounter + 1]));
       if (target.yaw > 180){target.yaw -= 360;}
     }
   }
   
   distanceCheck(orient, target);
}

void initializeAuto(class MODE2_REGISTER mode, class TARGET_REGISTER &target){
 
}
