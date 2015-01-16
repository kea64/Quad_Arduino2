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
#include <H2_TiltComp.h>
#include <H2_Orient.h>
#include <H2_Target.h>
#include <H2_PID.h>


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
#define ROLL_MAXIMUM 250
#define PITCH_MAXIMUM 250
#define THROTTLE_MAXIMUM 1900
#define THROTTLE_MINIMUM 1100
#define THROTTLE_CUTOFF 1150
#define YAW_MAXIMUM 200
#define YAW_RATE 0.1
#define GPS_ROLL_MAXIMUM 150
#define GPS_PITCH_MAXIMUM 150
#define ACC_SCALAR 0.93
#define ARM_ENGAGE_THRESHOLD 1700
#define ARM_DISENGAGE_THRESHOLD 1100
#define ARM_THROTTLE_THRESHOLD 1100
#define GPS_SATELLITE_MINIMUM 5
#define SERVO_MAXIMUM 2000
#define SERVO_MIDPOINT 1500
#define SERVO_MINIMUM 1000

#define GPS_EN 1
#define AUXILIARY_EN 1
#define QUAD_EN 1 //Choose only 1 Frame. Defaults to Quad.
#define TRI_EN 0

#define kpr 0
#define kir 0
#define kdr 0

#define kpp 0
#define kip 0
#define kdp 0

#define kpy 0
#define kiy 0
#define kdy 0

#define kpt 0
#define kit 0
#define kdt 0

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
#define CONTROL_DELAY 5

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

Servo output1;
Servo output2;
Servo output3;
Servo output4;

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
  
  output1.attach(aileronPin);
  output2.attach(elevatorPin);
  output3.attach(throttlePin);
  output4.attach(rudderPin);
  
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
  ORIENT_STRUCT orient;
  TARGET_STRUCT target;
  PID_REGISTER channels;
  
  int RC_CONTROL_MODE = 0;
  
  unsigned long compliClockNew, compliClockOld, baroClockOld, tempClockOld, commClockOld, elevClockOld, landClockOld, controlClockOld;
  
  gyro.init();
  accel.init();
  mag.init(xMagError, yMagError, zMagError, xMagOffset, yMagOffset, zMagOffset);
  baro.begin(BARO_MODE, altAlpha);
  
  channels.rollPID.updateDefaults(kpr, kir, kdr, SERVO_MIDPOINT + ROLL_MAXIMUM, SERVO_MIDPOINT - ROLL_MAXIMUM, ROLL_OFFSET);
  channels.pitchPID.updateDefaults(kpp, kip, kdp, SERVO_MIDPOINT + PITCH_MAXIMUM, SERVO_MIDPOINT - PITCH_MAXIMUM, PITCH_OFFSET);
  channels.yawPID.updateDefaults(kpy, kiy, kdy, SERVO_MIDPOINT + YAW_MAXIMUM, SERVO_MIDPOINT - YAW_MAXIMUM, YAW_OFFSET);
  
  channels.autoRollPID.updateDefaults(gpr, gir, gdr, SERVO_MIDPOINT + GPS_ROLL_MAXIMUM, SERVO_MIDPOINT - GPS_ROLL_MAXIMUM, 0);
  channels.autoPitchPID.updateDefaults(gpp, gip, gdp, SERVO_MIDPOINT + GPS_PITCH_MAXIMUM, SERVO_MIDPOINT - GPS_PITCH_MAXIMUM, 0);
  channels.autoThrottlePID.updateDefaults(kpt, kit, kdt, THROTTLE_MAXIMUM, -THROTTLE_MAXIMUM, 0);
  
  gyro.calibrate();
  
  initAngles(orient, accel);
  
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
  controlClockOld = millis();
  
//_______________________________________________________________________// 
//---------------------------Actual Loop---------------------------------//
//_______________________________________________________________________//

  while(1==1){
    //Sensor Updates
    checkCompli(gyro, accel, mag, orient, compliClockOld);
    checkBaro(baro, baroClockOld, orient);
    checkTemp(baro, tempClockOld);
    
    if (GPS_EN){
      updateGPS(orient, target);
    }
    
    updateMode(channels, target, orient, RC_CONTROL_MODE);
    updateController(channels, target, orient, RC_CONTROL_MODE, controlClockOld);
    
    //Status Feedback
    transmitData(orient, baro, commClockOld); 
  } 
}

void initAngles(struct ORIENT_STRUCT &orient, class ADXL345 &acc){
  acc.update(); //Obtains Initial Angles; Quad must be motionless
  orient.roll = atan2(acc.y, acc.z)*(180 / PI) + ROLL_OFFSET; //Accounts for Angle Differences
  orient.pitch = atan2(-acc.x, acc.z)*(180 / PI) + PITCH_OFFSET;
}

void checkCompli(class ITG3200 &gyro, class ADXL345 &acc, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
   //Main Sensor Reading and Motor Control
  if ((millis() - compliClockOld) >= COMPLI_DELAY){
    compli(gyro, acc, orient, compliClockOld); //Complimentary Filter
    calcYaw(mag, orient, ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET); //Tilt Compensated Compass Code
    
    compliClockOld = millis();
  }
}
 
void compli(class ITG3200 &gyro, class ADXL345 &accel, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
  //Complimentary Filter to Mix Gyro and Accelerometer Data
  gyro.update();
  accel.update();
  
  double cycle = (((millis() - compliClockOld)*1.0)/1000.0);
  
  double pitchAccel = atan2(-accel.x,accel.z)*(180.0/PI)*ACC_SCALAR + PITCH_OFFSET;
  orient.pitch = compliAlpha * (orient.pitch + (gyro.y) * cycle) + (1 - compliAlpha) * pitchAccel;
  
  double rollAccel = atan2(accel.y,accel.z)*(180.0/PI)*ACC_SCALAR + ROLL_OFFSET;
  orient.roll = compliAlpha * (orient.roll + (gyro.x) * cycle) + (1 - compliAlpha) * rollAccel;
}

void checkBaro(class BMP180 &baro, unsigned long &baroClockOld, struct ORIENT_STRUCT &orient){
   if ((millis() - baroClockOld) >= BARO_DELAY){
    baro.updatePressure();
    baro.calculateAltitude();
    baroClockOld = millis();
  }
}

void incAlt(struct TARGET_STRUCT &target){
	//Elevation Change Smoothing -- Delays Target Altitude to Maximize I Gain
	if (target.intAlt < target.alt){ target.intAlt += 0.03; }
	if (target.intAlt > target.alt){ target.intAlt -= 0.03; }
}

void checkTemp(class BMP180 &baro, unsigned long &tempClockOld){
   if ((millis() - tempClockOld) >= TEMP_DELAY){
    baro.updateTemperature();
    tempClockOld = millis();
  }
}

void updateGPS(struct ORIENT_STRUCT &orient, struct TARGET_STRUCT &target){
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      orient.latitude = gps.location.lat();
      orient.longitude = gps.location.lng();
      
      if (gps.satellites.value() >= GPS_SATELLITE_MINIMUM){ //GPS Lock Indicator
        digitalWrite(13,HIGH);
        orient.GPS_LOCK = 1;
      } else {
        digitalWrite(13,LOW);
        orient.GPS_LOCK = 0;
      }
    }
  }
}

void transmitData(struct ORIENT_STRUCT &orient, class BMP180 baro, unsigned long &commClockOld){
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

void updateController(struct PID_REGISTER &channels, struct TARGET_STRUCT target, struct ORIENT_STRUCT orient, int RC_CONTROL_MODE, unsigned long &controlClockOld){
   if ((millis() - controlClockOld) >= CONTROL_DELAY){
     double cycle = millis() - controlClockOld;
     double output1, output2, output3, output4;
     
     double rollSensor = newMap(orient.roll, -90, 90, 1000, 2000);
     double pitchSensor = newMap(orient.pitch, -90, 90, 1000, 2000);
     double yawSensor = newMap(orient.pitch, -180, 180, 1000, 2000);
     
     switch (RC_CONTROL_MODE){
       case 0:
           if (AUXILIARY_EN){
             output1 = channel1Cycle;
             output2 = channel2Cycle;
             output3 = channel3Cycle;
             output4 = channel4Cycle;
           } else {
             //channels.rollPID.calc(error, sensor, cycle);
             
             channels.rollPID.calc(channel1Cycle - rollSensor, rollSensor, cycle);
             channels.pitchPID.calc(channel1Cycle - pitchSensor, pitchSensor, cycle);
             channels.yawPID.calc(target.yawRate - yawSensor, yawSensor, cycle);
             
             output1 = channels.rollPID.getControl();
             output2 = channels.pitchPID.getControl();
             output3 = channel3Cycle;
             output4 = channels.yawPID.getControl();
           }
           break;
       case 1:
           
           break;
       case 2:
       
           break;
     }
     
     processMotors();
     
     controlClockOld = millis(); 
   }
}

void processMotors(){
  if (QUAD_EN){
    
  } else if (TRI_EN){
    
  }
  
}

void updateMode(struct PID_REGISTER &channels, struct TARGET_STRUCT &target, struct ORIENT_STRUCT &orient, int &RC_CONTROL_MODE){
  if (channel5Cycle < 1300){
    if (RC_CONTROL_MODE != 1){
      target.alt = waypoint[target.waypointCounter + 2];
      target.intAlt = orient.alt;
      channels.autoThrottlePID.setIntegral(channel3Cycle);
      channels.autoRollPID.updateBounds(channel1Cycle + GPS_ROLL_MAXIMUM, channel1Cycle - GPS_ROLL_MAXIMUM);
      channels.autoPitchPID.updateBounds(channel2Cycle + GPS_PITCH_MAXIMUM, channel2Cycle - GPS_PITCH_MAXIMUM);
      channels.autoRollPID.setIntegral(channel1Cycle);
      channels.autoPitchPID.setIntegral(channel2Cycle);
    }
    
    if (GPS_EN && orient.GPS_LOCK){
      target.yaw = int(TinyGPSPlus::courseTo(orient.latitude,orient.longitude,waypoint[target.waypointCounter],waypoint[target.waypointCounter + 1]));
    } else {
      target.yaw = orient.yaw;
    }
    
    RC_CONTROL_MODE = 1;
    
  } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700){
    if (RC_CONTROL_MODE != 0){
      target.yawRate = orient.yaw;
    }
    
    target.yawRate += (channel4Cycle - SERVO_MIDPOINT) * YAW_RATE;
    if (target.yawRate > SERVO_MAXIMUM){
      target.yawRate = SERVO_MAXIMUM;
    } else if (target.yawRate < SERVO_MINIMUM){
      target.yawRate = SERVO_MINIMUM;
    }
    
    RC_CONTROL_MODE = 0;
    
  } else if (channel5Cycle > 1700){
    if (RC_CONTROL_MODE != 2){
      target.alt = orient.alt;
      target.intAlt = orient.alt;
      channels.autoThrottlePID.setIntegral(channel3Cycle);
      channels.autoRollPID.updateBounds(channel1Cycle + GPS_ROLL_MAXIMUM, channel1Cycle - GPS_ROLL_MAXIMUM);
      channels.autoPitchPID.updateBounds(channel2Cycle + GPS_PITCH_MAXIMUM, channel2Cycle - GPS_PITCH_MAXIMUM);
      channels.autoRollPID.setIntegral(channel1Cycle);
      channels.autoPitchPID.setIntegral(channel2Cycle);
      target.latitude = orient.latitude;
      target.longitude = orient.longitude;
      target.yaw = orient.yaw;
    }
    
    RC_CONTROL_MODE = 2;
    
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

int newMap(int inValue, int inLow, int inHigh, int outLow, int outHigh){
  return ((inValue - inLow) * (1.0 * (outHigh - outLow)) / (1.0 * (inHigh - inLow)) + outLow);
}
