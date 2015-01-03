//#define NO_PORTC_PINCHANGES
//#define NO_PORTB_PINCHANGES

#include <Servo.h>
#include <Wire.h>
#include <PinChangeInt.h>
#include <TinyGPS++.h>
#include <H2_HMC5883L.h>
#include <H2_ADXL345.h>
#include <H2_ITG3200.h>
#include <H2_BMP180.h>
#include <H2_Registers.h>
//#include <H2_Filters.h>
#include <H2_TiltComp.h>

#define address 0x1E
#define xMagError 0.94
#define yMagError 0.99
#define zMagError 0.94
#define xMagOffset -90
#define yMagOffset -85
#define zMagOffset 225
#define COMPLI_DELAY 5
#define BARO_DELAY 20
#define TEMP_DELAY 1000
#define COMM_DELAY 250
#define ELEV_DELAY 10
#define LAND_DELAY 20000
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
#define channel1 2
#define channel2 3
#define channel3 4
#define channel4 5
#define channel5 6
#define channel6 7

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
  
HMC5883L mag;     
ADXL345 accel;
ITG3200 gyro;
BMP180 baro;
TinyGPSPlus gps;
MODE_REGISTER mode;
ORIENTATION_REGISTER orient;

Servo Throttle;
Servo Rudder;
Servo Elevator;
Servo Aileron;

struct targetRegisterStruct{
  double alt;
  double intAlt;
  double latitude;
  double longitude;
  int heading;
  int waypointCounter;
};

struct PIDStruct{
  double iTerm;
  double lastPos;
  double control;
  double maximum;
  double minimum;
  
  unsigned long clockOld;
};



//-------------------------------SETUP-----------------------------------//

void setup(){  
   Wire.begin(); 
   Serial.begin(38400);

   //Peripheral Setup
   gyro.init();
   accel.init();
   mag.init(xMagError, yMagError, zMagError, xMagOffset, yMagOffset, zMagOffset);
   baro.begin(BARO_MODE, altAlpha);
   mode.init(0);
   
   Aileron.attach(aileronPin);
   Elevator.attach(elevatorPin);
   Throttle.attach(throttlePin);
   Rudder.attach(rudderPin);
   
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
  targetRegisterStruct targetRegister = {1.0,1.0,0,0,0,0};
  PIDStruct roll;
  PIDStruct pitch;
  PIDStruct yaw;
  PIDStruct throttle;
  
  double cycle;
  double ITermT, ITermP, ITermR;
  
  unsigned long compliClockNew, compliClockOld, baroClockOld, tempClockOld, commClockOld, elevClockOld, landClockOld;

  int elevatorInitial, aileronInitial;
 
  gyro.calibrate();
  
  orient.initAngles(accel, ROLL_OFFSET, PITCH_OFFSET);
 
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
    checkCompli(gyro, accel, orient, compliClockOld);
    
    checkBaro(baro, baroClockOld, orient);
    
    checkTemp(baro, tempClockOld);
    
    processInterrupts(mode, orient, targetRegister, ITermT, ITermR, ITermP, aileronInitial, elevatorInitial);
    
    checkDistance(targetRegister, orient);
    
    transmitData(orient, gyro, baro, commClockOld);
    
    updateGPS(orient.latitude,orient.longitude);
    
    calcPID(roll,0,0,0,0,0);
    calcPID(pitch,0,0,0,0,0);
    calcPID(yaw,0,0,0,0,0);
    calcPID(throttle,0,0,0,0,0);
  }
}

//----------------------------Functions----------------------------------//
//_______________________________________________________________________//


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

void compli(class ITG3200 &gyro, class ADXL345 &accel, class ORIENTATION_REGISTER &orient, unsigned long &compliClockOld){
  //Complimentary Filter to Mix Gyro and Accelerometer Data
  gyro.update();
  accel.update();
  
  double cycle = (((millis() - compliClockOld)*1.0)/1000.0);
  
  double pitchAccel = atan2(-accel.x,accel.z)*(180.0/PI)*ACC_SCALAR + PITCH_OFFSET;
  orient.pitch = compliAlpha * (orient.pitch + ((gyro.x)/14.375) * cycle) + (1 - compliAlpha) * pitchAccel;
  
  double rollAccel = atan2(accel.y,accel.z)*(180.0/PI)*ACC_SCALAR + ROLL_OFFSET;
  orient.roll = compliAlpha * (orient.roll + ((gyro.y)/14.375) * cycle) + (1 - compliAlpha) * rollAccel;

}

int calcPID(struct PIDStruct &motion, double target, double currPos, const double kp, const double ki, const double kd){
  
  int error = target - currPos;
  motion.iTerm += (ki * 0.001 * int(millis() - motion.clockOld) * error);
  if (motion.iTerm > motion.maximum){motion.iTerm = motion.maximum;}
  else if (motion.iTerm < motion.minimum){motion.iTerm = motion.minimum;}
  motion.control = kp * error + motion.iTerm - ((kd * (currPos - motion.lastPos))/(0.001 * (millis() - motion.clockOld)));
  if (motion.control > motion.maximum) {motion.control = motion.maximum;}
  if (motion.control < motion.minimum) {motion.control = motion.minimum;}
  motion.lastPos = currPos;
  int controlOut = int(motion.control);
  
  motion.clockOld = millis();
  
  return(controlOut);
}

void checkDistance(struct targetRegisterStruct &targetRegister, class ORIENTATION_REGISTER &orient){
  //Monitors Distance to Waypoints and updates them when the quad arrives
  int distanceToWaypoint = int(TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),waypoint[targetRegister.waypointCounter],waypoint[targetRegister.waypointCounter + 1]));
  if(distanceToWaypoint <= 3){
    targetRegister.waypointCounter += 3;
    if ((targetRegister.waypointCounter/3) > numWaypoint && (abs(targetRegister.alt - orient.alt)) < 2){
     targetRegister.waypointCounter = 0; 
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

void processInterrupts(class MODE_REGISTER &mode, class ORIENTATION_REGISTER &orient, struct targetRegisterStruct &targetRegister, double &ITermT, double &ITermR, double &ITermP, int &aileronInitial, int &elevatorInitial){
   if (channel5Cycle < 1300){
     if (mode.RC_CONTROL_MODE != 1){
       targetRegister.alt = waypoint[targetRegister.waypointCounter + 2];
       targetRegister.intAlt = orient.alt;
       ITermT = channel3Cycle;
       aileronInitial = channel1Cycle;
       elevatorInitial = channel2Cycle;
       targetRegister.heading = int(TinyGPSPlus::courseTo(orient.latitude,orient.longitude,waypoint[targetRegister.waypointCounter],waypoint[targetRegister.waypointCounter + 1]));
       if (targetRegister.heading > 180){targetRegister.heading -= 360;}
     }
     
     mode.RC_CONTROL_MODE = 1;
     
   } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700){
     mode.RC_CONTROL_MODE = 0;
     
   } else if (channel5Cycle > 1700){
     if (mode.RC_CONTROL_MODE != 2){
          targetRegister.alt = orient.alt;
          targetRegister.intAlt  = orient.alt;
          ITermT = channel3Cycle;
          targetRegister.heading = orient.yaw;
          aileronInitial = channel1Cycle;
          elevatorInitial = channel2Cycle;
          ITermR = channel1Cycle;
          ITermP = channel2Cycle;
          orient.latitude = gps.location.lat();
          orient.longitude = gps.location.lng();
          targetRegister.latitude = orient.latitude;
          targetRegister.longitude = orient.longitude;
      }
      
      mode.RC_CONTROL_MODE = 2;
   }
   
   //------------------Mode Select------------------//
   mode.switchModes();
   
   //------------------Channel6Var------------------//
   channel6Var = 2000.0 * (channel6Var - 1000);
   if (channel6Var < 0.0) {channel6Var = 0.0;}
   
   //-------------------Channel 1-------------------//
   if (mode.CH1_ENABLE){
     Aileron.writeMicroseconds(channel1Cycle);
   }
   
   //-------------------Channel 2-------------------//
   if (mode.CH2_ENABLE){
     Elevator.writeMicroseconds(channel2Cycle);
   }
   
   //-------------------Channel 3-------------------//
   if (mode.CH3_ENABLE){
     Throttle.writeMicroseconds(channel3Cycle);
   }
   
   //-------------------Channel 4-------------------//
   if (mode.CH4_ENABLE){
     Rudder.writeMicroseconds(channel4Cycle);
   }
   
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
 
 void checkCompli(class ITG3200 &gyro, class ADXL345 &acc, class ORIENTATION_REGISTER &orient, unsigned long &compliClockOld){
   //Main Sensor Reading and Motor Control
  if ((millis() - compliClockOld) >= COMPLI_DELAY){
    compli(gyro, accel, orient, compliClockOld); //Complimentary Filter
    calcYaw(mag, orient, ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET); //Tilt Compensated Compass Code
    //GPS Navigation Mode
//    if (RC_CONTROL_MODE == 2 || RC_ENABLE != 1){
//        yawUpdate(); // Yaw Control for navigation
//        //If GPS Locked, Enable Translation Mode
//        if (gps.satellites.value() > GPS_SATELLITE_MINIMUM){
//            FORWARD_MODE_ENABLE = 1;
//            STRAFE_MODE_ENABLE = 1;
//            translationUpdate(); //Roll + Pitch Control for navigation   
//        } else {
//          FORWARD_MODE_ENABLE = 0;
//          STRAFE_MODE_ENABLE = 0;
//        }
//    }
//    //Following statement may be deleted after tuning translation
//    if (RC_CONTROL_MODE == 1){
//      yawUpdate();
//      FORWARD_MODE_ENABLE = 0;
//      STRAFE_MODE_ENABLE = 0;
//    }
    compliClockOld = millis();
  }
 }
 
 void transmitData(class ORIENTATION_REGISTER &orient, class ITG3200 gyro, class BMP180 baro, unsigned long &commClockOld){
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
 
