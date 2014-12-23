#define NO_PORTC_PINCHANGES
#define NO_PORTB_PINCHANGES

#include <Servo.h>
#include <Wire.h>
#include <PinChangeInt.h>
#include <TinyGPS++.h>
#include <H2_HMC5883L.h>
#include <H2_ADXL345.h>
#include <H2_L3D4200D.h>
#include <H2_BMP180.h>
#include <H2_Registers.h>
//#include <H2_Filters.h>
#include <H2_TiltComp.h>


#define xMagError 0.96
#define yMagError 1.01
#define zMagError 0.95
#define xMagOffset -23
#define yMagOffset -102
#define zMagOffset 0
#define ROLL_OFFSET 0 //Positive Lowers Aft 4
#define PITCH_OFFSET 0 //Positive Lowers Aft 2
#define YAW_OFFSET 0
#define ROLL_SENSITIVITY 0.5
#define PITCH_SENSITIVITY 0.5
#define YAW_SENSITIVITY 0.25 //Controls the degree at which CH4 affects yaw
#define ACC_SCALAR 0.93
#define ARM_ENGAGE_THRESHOLD 1700
#define ARM_DISENGAGE_THRESHOLD 1100
#define ARM_THROTTLE_THRESHOLD 1100

#define altAlpha 0.9
#define compliAlpha 0.98

#define BARO_MODE 3
#define COMPLI_DELAY 5
#define BARO_DELAY 20
#define TEMP_DELAY 1000
#define COMM_DELAY 250
#define ELEV_DELAY 10
#define LAND_DELAY 20000

#define arm1Pin A3
#define arm2Pin A2
#define arm3Pin A0
#define throttlePin A1
#define channel1 2
#define channel2 3
#define channel3 4
#define channel4 5
#define channel5 6
#define channel6 7

volatile int channel1Cycle;
volatile int channel2Cycle;
volatile int channel3Cycle;
volatile int channel4Cycle;
volatile int channel5Cycle;
volatile int channel6Cycle;

unsigned long channel1Start,channel2Start,channel3Start,channel4Start,channel5Start,channel6Start;

double channel6Var;

Servo Arm1;
Servo Arm2;
Servo Arm3;
Servo Throttle;


HMC5883L mag;     
ADXL345 accel;
L3D4200D gyro;
BMP180 baro;
ORIENTATION_REGISTER orient;
TARGET_REGISTER target;
MODE_REGISTER mode;

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
  
  Arm1.attach(arm1Pin);
  Arm2.attach(arm2Pin);
  Arm3.attach(arm3Pin);
  Throttle.attach(throttlePin);
  
  pinMode(13,OUTPUT); //Arming Indicator
  digitalWrite(13,LOW);
   
  gyro.init();
  accel.init();
  mag.init(xMagError, yMagError, zMagError, xMagOffset, yMagOffset, zMagOffset);
  baro.begin(BARO_MODE, altAlpha);
  mode.init(0);
  
}

//----------------------------MAIN/SETUP----------------------------------//

void loop(){
  int elevatorInitial, aileronInitial;
  
  double cycle;
  double ITermT, ITermP, ITermR;
  
  unsigned long compliClockNew, compliClockOld, baroClockOld, tempClockOld, commClockOld, elevClockOld, landClockOld;
  
  bool MOTOR_ENABLE = 0;
  
  gyro.calibrate();
  
  orient.initAngles(accel, ROLL_OFFSET, PITCH_OFFSET);
  
  target.roll = ROLL_OFFSET;
  target.pitch = PITCH_OFFSET;
  target.yaw = orient.yaw;
  
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
    checkCompli(gyro, accel, mag, orient, compliClockOld);
    
    checkBaro(baro, baroClockOld, orient);
    
    checkTemp(baro, tempClockOld);
    
    processInterrupts(mode, orient, target);
    
    transmitData(orient, baro, commClockOld);
    
    checkArming(MOTOR_ENABLE);
    
    if (MOTOR_ENABLE == 1){
      //Add Motor Control Code Here
      
    } else {
      Throttle.write(0);
    }
   
  }
  
}

void checkCompli(class L3D4200D &gyro, class ADXL345 &acc, class HMC5883L &mag, class ORIENTATION_REGISTER &orient, unsigned long &compliClockOld){
   //Main Sensor Reading and Motor Control
  if ((millis() - compliClockOld) >= COMPLI_DELAY){
    compli(gyro, accel, orient, compliClockOld); //Complimentary Filter
    calcYaw(mag, orient, ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET); //Tilt Compensated Compass Code
    
    compliClockOld = millis();
  }
}
 
 void compli(class L3D4200D &gyro, class ADXL345 &accel, class ORIENTATION_REGISTER &orient, unsigned long &compliClockOld){
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

void checkArming(bool &MOTOR_ENABLE){
  if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle >= ARM_ENGAGE_THRESHOLD){
      MOTOR_ENABLE = 1;
      digitalWrite(13,HIGH);
      
  } else if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle <= ARM_DISENGAGE_THRESHOLD){
      MOTOR_ENABLE = 0;
      digitalWrite(13,LOW);
  }
  //Perhaps Add PID DISABLE Code Here
}

void processInterrupts(class MODE_REGISTER &mode, class ORIENTATION_REGISTER &orient, class TARGET_REGISTER &targetRegister){
   if (channel5Cycle < 1300){
     if (mode.RC_CONTROL_MODE != 1){
       //NEW MODE 1 CODE HERE
     }
     
     mode.RC_CONTROL_MODE = 1;
     
   } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700){
     mode.RC_CONTROL_MODE = 0;
     
   } else if (channel5Cycle > 1700){
     if (mode.RC_CONTROL_MODE != 2){
        //NEW MODE 2 CODE HERE  
      }
      
      mode.RC_CONTROL_MODE = 2;
   }
   
   //------------------Mode Select------------------//
   mode.switchModes();
   
   updateTarget(target);
}

void updateTarget(class TARGET_REGISTER &target){
   //--------------------Channel1-------------------//
   if (mode.CH1_ENABLE){
     target.roll = (channel1Cycle-1500) * 0.18 * ROLL_SENSITIVITY + ROLL_OFFSET;
   }
   
   //--------------------Channel2-------------------//
   if (mode.CH2_ENABLE){
     target.pitch = (channel2Cycle-1500) * 0.18 * PITCH_SENSITIVITY + PITCH_OFFSET;
   }
   
   //--------------------Channel4-------------------//
   if (mode.CH4_ENABLE){
     target.yaw = (channel4Cycle-1500) * 0.18 * YAW_SENSITIVITY + target.yaw;
     if (target.yaw > 180.0){
       target.yaw -= 360;
     }
     if (target.yaw < -180.0){
       target.yaw += 360;
     }
   }
   
   //------------------Channel6Var------------------//
   channel6Var = 2000.0 * (channel6Var - 1000);
   if (channel6Var < 0.0) {channel6Var = 0.0;}
   
}

