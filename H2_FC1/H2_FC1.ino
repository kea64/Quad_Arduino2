#define NO_PORTC_PINCHANGES
#define NO_PORTB_PINCHANGES

#include <Servo.h>
#include <Wire.h>
#include <PinChangeInt.h>
#include <TinyGPS++.h>
#include <H2_HMC5883L.h>
#include <H2_ADXL345.h>
#include <H2_ITG3200.h>
#include <H2_L3D4200D.h>
#include <H2_BMP180.h>
#include <H2_TiltComp.h>
#include <H2_Orient.h>
#include <H2_Target.h>
#include <H2_PID.h>
#include <H2_Output.h>

#define xMagError 0.96
#define yMagError 1.01
#define zMagError 0.95
#define xMagOffset -23
#define yMagOffset -102
#define zMagOffset 0
#define ROLL_OFFSET 0 
#define PITCH_OFFSET 0 
#define YAW_OFFSET 90
#define ROLL_SENSITIVITY 0.5
#define PITCH_SENSITIVITY 0.5
#define YAW_SENSITIVITY 0.25 //Controls the degree at which CH4 affects yaw
#define ROLL_RATE_MAXIMUM 350
#define PITCH_RATE_MAXIMUM 350
#define YAW_RATE_MAXIMUM 400
#define ROLL_STAB_MAXIMUM 400
#define PITCH_STAB_MAXIMUM 400
#define YAW_STAB_MAXIMUM 300
#define THROTTLE_MAXIMUM 1864
#define THROTTLE_MINIMUM 1188
#define AUTO_THROTTLE_MAXIMUM 1700
#define AUTO_THROTTLE_MINIMUM 1300
#define THROTTLE_CUTOFF 1250
#define GPS_ROLL_MAXIMUM 20
#define GPS_PITCH_MAXIMUM 20
#define ACC_SCALAR 0.93
#define ARM_ENGAGE_THRESHOLD 1190
#define ARM_DISENGAGE_THRESHOLD 1840
#define ARM_THROTTLE_THRESHOLD 1210
#define GPS_SATELLITE_MINIMUM 5
#define SERVO_MAXIMUM 2000
#define SERVO_MIDPOINT 1500
#define SERVO_MINIMUM 1000
#define TAIL_SERVO_MAX_DEGREE 70
#define TAIL_SERVO_MIN_DEGREE 120
#define TAIL_SERVO_MAX 1920
#define TAIL_SERVO_MIN 1168
#define TAIL_SERVO_OFFSET 5
#define DIV_BY_MILL 0.001
#define INITIAL_ARM_DELAY 3000

#define DEBUG_EN 1
#define GPS_EN 0
#define AUXILIARY_EN 0
#define ACRO_EN 0
#define QUAD_EN 0 //Choose only 1 Frame. Defaults to Quad.
#define TRI_EN 1

#define KPRS 0.55
#define KIRS 0
#define KDRS 0
#define KMRS 25

#define KPRR 0.6
#define KIRR 1
#define KDRR 0
#define KMRR 80

#define KPPS 0.55
#define KIPS 0
#define KDPS 0
#define KMPS 25

#define KPPR 0.6
#define KIPR 1
#define KDPR 0
#define KMPR 80

#define KPYS 0
#define KIYS 0
#define KDYS 0
#define KMYS 25

#define KPYR 3
#define KIYR 0
#define KDYR 0
#define KMYR 25

#define KPT 0
#define KIT 0
#define KDT 0
#define KMT 25

#define GPR 0
#define GIR 0
#define GDR 0
#define GMR 25

#define GPP 0
#define GIP 0
#define GDP 0
#define GMP 25

#define altAlpha 0.9
#define compliAlpha 0.97
#define GYROALPHA 0.85
#define ACC_ALPHA 0.25

#define BARO_MODE 3

#define COMPLI_DELAY 200
#define BARO_DELAY 50
#define TEMP_DELAY 2000
#define COMM_DELAY 250
#define CONTROL_DELAY 200
#define MODE_DELAY 20

//#define COMPLI_DELAY 1
//#define BARO_DELAY 50
//#define TEMP_DELAY 2000
//#define COMM_DELAY 250
//#define CONTROL_DELAY 1
//#define MODE_DELAY 20

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

int count = 0;

unsigned long channel1Start,channel2Start,channel3Start,channel4Start,channel5Start,channel6Start;

double channel6Var = 0;

Servo output1;
Servo output2;
Servo output3;
Servo output4;

TinyGPSPlus gps;

//-------------------------------SETUP-----------------------------------//

void setup(){
  Wire.begin();
  Serial.begin(115200);
  
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
  
  pinMode(13,OUTPUT); //Satellite Lock Indicator and Arming Indicator
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
  L3D4200D gyro;
  BMP180 baro;
  ORIENT_STRUCT orient;
  TARGET_STRUCT target;
  OUTPUT_STRUCT output;
  PID_REGISTER channels;
  
  output.roll = 0;
  output.pitch = 0;
  output.throttle = 0;
  output.yaw = 0;
  
  //channel1Cycle = 1520;
  //channel2Cycle = 1500;
  //channel3Cycle = 1500;
  //channel4Cycle = 1500;
  //channel5Cycle = 1500;
  
  byte RC_CONTROL_MODE = 0;
  
  bool MOTOR_EN = 0;
  
  unsigned long compliClockOld, baroClockOld, tempClockOld, commClockOld, controlClockOld, modeClockOld;
  
  gyro.init(GYROALPHA);
  accel.init(ACC_ALPHA);
  mag.init(xMagError, yMagError, zMagError, xMagOffset, yMagOffset, zMagOffset);
  baro.begin(BARO_MODE, altAlpha);
  
  channels.rsPID.updateDefaults(KPRS, KIRS, KDRS, 0, ROLL_STAB_MAXIMUM, -ROLL_STAB_MAXIMUM, KMRS);
  channels.psPID.updateDefaults(KPPS, KIPS, KDPS, 0, PITCH_STAB_MAXIMUM, -PITCH_STAB_MAXIMUM, KMPS);
  channels.ysPID.updateDefaults(KPYS, KIYS, KDYS, 0, YAW_STAB_MAXIMUM, -YAW_STAB_MAXIMUM, KMYS);
  
  channels.rrPID.updateDefaults(KPRR, KIRR, KDRR, 0, ROLL_RATE_MAXIMUM, -ROLL_RATE_MAXIMUM, KMRR);
  channels.prPID.updateDefaults(KPPR, KIPR, KDPR, 0, PITCH_RATE_MAXIMUM, -PITCH_RATE_MAXIMUM, KMPR);
  channels.yrPID.updateDefaults(KPYR, KIYR, KDYR, 0, YAW_RATE_MAXIMUM, -YAW_RATE_MAXIMUM, KMYR);
  
  channels.atPID.updateDefaults(KPT, KIT, KDT, 0, THROTTLE_MAXIMUM, THROTTLE_MINIMUM, KMT);
  
  if (GPS_EN){
    channels.arPID.updateDefaults(GPR, GIR, GDR, 0, GPS_ROLL_MAXIMUM, -GPS_ROLL_MAXIMUM, GMR);
    channels.apPID.updateDefaults(GPP, GIP, GDP, 0, GPS_PITCH_MAXIMUM, -GPS_PITCH_MAXIMUM, GMP);
  }
  
  delay(500);
  
  gyro.calibrate();
  
  initAngles(orient, accel);
  
  target.roll = ROLL_OFFSET;
  target.pitch = PITCH_OFFSET;
  target.throttle = 0.0;
  target.yaw = orient.yaw;
  
  //Reset State Control Timers
  compliClockOld = micros();
  baroClockOld = millis();
  tempClockOld = millis();
  commClockOld = millis();
  controlClockOld = micros();
  modeClockOld = millis();
  
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
    
    updateMode(channels, target, orient, RC_CONTROL_MODE, modeClockOld);
    
    checkArming(MOTOR_EN);
    
    if (MOTOR_EN || AUXILIARY_EN){
      updateController(channels, target, orient, output, RC_CONTROL_MODE, controlClockOld);
    } else {
      output1.writeMicroseconds(1188);
      output2.writeMicroseconds(1188);
      output3.writeMicroseconds(1188);
      output4.write(90);
    }
    
    //Status Feedback
    transmitData(orient, baro, output, commClockOld); 
    
  } 
}

void initAngles(struct ORIENT_STRUCT &orient, class ADXL345 &acc){
  acc.update(); //Obtains Initial Angles; Quad must be motionless
  orient.roll = atan2(acc.y, acc.z)*(180 / PI) + ROLL_OFFSET; //Accounts for Angle Differences
  orient.pitch = atan2(-acc.x, acc.z)*(180 / PI) + PITCH_OFFSET;
}

void checkCompli(class L3D4200D &gyro, class ADXL345 &acc, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
   //Main Sensor Reading and Motor Control
  if ((micros() - compliClockOld) >= COMPLI_DELAY){
    compli(gyro, acc, orient, compliClockOld); //Complimentary Filter
    calcYaw(mag, orient, ROLL_OFFSET, PITCH_OFFSET, YAW_OFFSET); //Tilt Compensated Compass Code
    
    compliClockOld = micros();
  }
}
 
void compli(class L3D4200D &gyro, class ADXL345 &accel, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
  //Complimentary Filter to Mix Gyro and Accelerometer Data
  gyro.update();
  accel.update();
  
  orient.rollGyro = gyro.x;
  orient.pitchGyro = gyro.y;
  orient.yawGyro = gyro.z;
  
  double cycle = (micros() - compliClockOld) * 0.000001;
  
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

void transmitData(struct ORIENT_STRUCT &orient, BMP180 baro, struct OUTPUT_STRUCT output, unsigned long &commClockOld){
   if ((millis() - commClockOld) >= COMM_DELAY){
     
     if (DEBUG_EN){
       //Serial.print(orient.roll);
       //Serial.print(" ");
       //Serial.print(gyro.x);
       //Serial.print(" ");
       //Serial.println(atan2(accel.y,accel.z)*(180.0/PI)*ACC_SCALAR);
       
       Serial.print("Roll: ");
       Serial.println(orient.roll);
       Serial.print("Pitch: ");
       Serial.println(orient.pitch);
       Serial.print("Yaw: ");
       Serial.println(orient.yaw);
       Serial.print("Alt: ");
       Serial.println(baro.alt);
     
       Serial.print("Roll G: ");
       Serial.println(orient.rollGyro);
       Serial.print("Pitch G: ");
       Serial.println(orient.pitchGyro);
       
       Serial.print("Yaw G: ");
       Serial.println(orient.yawGyro);
       Serial.print("Output R: ");
       Serial.println(output.roll);
       Serial.print("Output P: ");
       Serial.println(output.pitch);
       Serial.print("Output T: ");
       Serial.println(output.throttle);
       Serial.print("Output Y: ");
       Serial.println(output.yaw);
       Serial.print("Ch1: ");
       Serial.println(channel1Cycle);
       Serial.print("Ch2: ");
       Serial.println(channel2Cycle);
       Serial.print("Ch3: ");
       Serial.println(channel3Cycle);
       Serial.print("Ch4: ");
       Serial.println(channel4Cycle);
       Serial.print("CH6VAR ");
       Serial.println(channel6Var);
       Serial.println(count);
       
     }
     count = 0;
     commClockOld = millis();
   }
}

void updateController(struct PID_REGISTER &channels, struct TARGET_STRUCT target, struct ORIENT_STRUCT orient, struct OUTPUT_STRUCT &output, byte RC_CONTROL_MODE, unsigned long &controlClockOld){
   if ((micros() - controlClockOld) >= CONTROL_DELAY){
     double cycle = (micros() - controlClockOld) * 0.000001;
     count += 1;
     
     channel6Var = newMap(channel6Cycle, 1000, 2000, 0, 2);
     channels.rsPID.updateGain(channel6Var, 0, 0);
     channels.psPID.updateGain(channel6Var, 0, 0);
     
     double rollChannel = newMap(channel1Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, -45, 45);
     double pitchChannel = newMap(channel2Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, 45, -45);
     double yawChannel = newMap(channel4Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, -90, 90);
     
     double errorLongitude;
     double errorLatitude;
     double errorRoll;
     double errorPitch;
     
     if (GPS_EN){
       errorLongitude = target.longitude - orient.longitude;
       errorLatitude = target.latitude - orient.latitude;
       errorRoll = (errorLongitude * cos(radians(orient.yaw)) + errorLatitude * sin(radians(orient.yaw)));
       errorPitch = -(errorLatitude * cos(radians(orient.yaw)) + errorLongitude * sin(radians(orient.yaw)));
       
       channels.arPID.calc(errorRoll, 0, cycle);
       channels.apPID.calc(errorPitch, 0, cycle);
     }
     
     switch (RC_CONTROL_MODE){
       case 0:
           if (AUXILIARY_EN){
             output.roll = channel1Cycle;
             output.pitch = channel2Cycle;
             output.throttle = channel3Cycle;
             output.yaw = channel4Cycle;
           } else {
             //channels.rollPID.calc(error, sensor, cycle);
             if (ACRO_EN && channel3Cycle >= THROTTLE_CUTOFF){
               channels.rrPID.calc(rollChannel - orient.rollGyro, orient.rollGyro, cycle);
               channels.prPID.calc(pitchChannel - orient.pitchGyro, orient.pitchGyro, cycle);
             } else if(channel3Cycle >= THROTTLE_CUTOFF) {
               channels.rsPID.calc(rollChannel - orient.roll, orient.roll, cycle);
               channels.psPID.calc(pitchChannel - orient.pitch, orient.pitch, cycle);
               
               count = channels.psPID.getControl();
               
               channels.rrPID.calc(channels.rsPID.getControl() - orient.rollGyro, orient.rollGyro, cycle);
               channels.prPID.calc(channels.psPID.getControl() - orient.pitchGyro, orient.pitchGyro, cycle);
             }
             
               
             channels.yrPID.calc(yawChannel - orient.yawGyro, orient.yawGyro, cycle);
             
             output.roll = channels.rrPID.getControl();
             output.pitch = channels.prPID.getControl();
             output.throttle = channel3Cycle;
             output.yaw = channels.yrPID.getControl();
           }
           break;
       case 1:
           channels.rsPID.calc(channels.arPID.getControl() - orient.roll, orient.roll, cycle);
           channels.psPID.calc(channels.apPID.getControl() - orient.pitch, orient.pitch, cycle);
           channels.ysPID.calc(target.yaw - orient.yaw, orient.yaw, cycle);
             
           channels.rrPID.calc(channels.rsPID.getControl() - orient.rollGyro, orient.rollGyro, cycle);
           channels.prPID.calc(channels.psPID.getControl() - orient.pitchGyro, orient.pitchGyro, cycle);
           channels.yrPID.calc(channels.ysPID.getControl() - orient.yawGyro, orient.yawGyro, cycle);
           
           channels.atPID.calc(target.alt - orient.alt, orient.alt, cycle);
           
           if (AUXILIARY_EN){
             //ADD QUAD GPS CODE HERE- NEED TO PASS GPSROLL TO MAP TO OUTPUT
           } else {
             output.roll = channels.rrPID.getControl();
             output.pitch = channels.prPID.getControl();
             output.throttle = channels.atPID.getControl();
             output.yaw = channels.yrPID.getControl();
           }
           break;
       case 2:
           channels.rsPID.calc(channels.arPID.getControl() - orient.roll, orient.roll, cycle);
           channels.psPID.calc(channels.apPID.getControl() - orient.pitch, orient.pitch, cycle);
           channels.ysPID.calc(target.yaw - orient.yaw, orient.yaw, cycle);
             
           channels.rrPID.calc(channels.rsPID.getControl() - orient.rollGyro, orient.rollGyro, cycle);
           channels.prPID.calc(channels.psPID.getControl() - orient.pitchGyro, orient.pitchGyro, cycle);
           channels.yrPID.calc(channels.ysPID.getControl() - orient.yawGyro, orient.yawGyro, cycle);
           
           channels.atPID.calc(target.alt - orient.alt, orient.alt, cycle);
           
           if (AUXILIARY_EN){
             //ADD QUAD GPS CODE HERE- NEED TO PASS GPSROLL TO MAP TO OUTPUT
           } else {
             output.roll = channels.rrPID.getControl();
             output.pitch = channels.prPID.getControl();
             output.throttle = channels.atPID.getControl();
             output.yaw = channels.yrPID.getControl();
           }
           break;
     }
     
     processMotors(output);
     
     controlClockOld = micros(); 
   }
}

void processMotors(struct OUTPUT_STRUCT output){
  
    if (QUAD_EN){
    
    } else if (TRI_EN){
      
        int op1 = output.throttle + output.roll - 0.8 * output.pitch;
        int op2 = output.throttle - output.roll - 0.8 * output.pitch;
        int op3 = output.throttle + output.pitch;
        int op4 = SERVO_MIDPOINT + output.yaw;
      
        double tailConv = newMap(op4, TAIL_SERVO_MIN, TAIL_SERVO_MAX, TAIL_SERVO_MIN_DEGREE, TAIL_SERVO_MAX_DEGREE) - TAIL_SERVO_OFFSET;
        
        op3 = ((op3 - SERVO_MINIMUM) / sin(radians(tailConv))) + SERVO_MINIMUM;
        
        withinBounds(op1, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
        withinBounds(op2, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
        withinBounds(op3, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
        withinBounds(op4, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
        
        if (output.throttle > THROTTLE_CUTOFF){
          output1.writeMicroseconds(op1);
          output2.writeMicroseconds(op2);
          output3.writeMicroseconds(op3);
          output4.writeMicroseconds(op4);
        } else {
          output1.writeMicroseconds(1188);
          output2.writeMicroseconds(1188);
          output3.writeMicroseconds(1188);
          output4.writeMicroseconds(SERVO_MIDPOINT);
        }
    }
}

void updateMode(struct PID_REGISTER &channels, struct TARGET_STRUCT &target, struct ORIENT_STRUCT &orient, byte &RC_CONTROL_MODE, unsigned long &modeClockOld){
  if (millis() - modeClockOld > MODE_DELAY){
    double rollChannel = newMap(channel1Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, -90, 90);
    double pitchChannel = newMap(channel2Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, -90, 90);
     
    if (channel5Cycle < 1300){
      if (RC_CONTROL_MODE != 1){
        target.alt = waypoint[target.waypointCounter + 2];
        channels.atPID.setIntegral(channel3Cycle);
        //channels.autoRollPID.updateBounds(channel1Cycle + GPS_ROLL_MAXIMUM, channel1Cycle - GPS_ROLL_MAXIMUM);
        //channels.autoPitchPID.updateBounds(channel2Cycle + GPS_PITCH_MAXIMUM, channel2Cycle - GPS_PITCH_MAXIMUM);
        channels.arPID.setIntegral(rollChannel);
        channels.apPID.setIntegral(pitchChannel);
      }
    
      if (GPS_EN && orient.GPS_LOCK){
        target.yaw = int(TinyGPSPlus::courseTo(orient.latitude,orient.longitude,waypoint[target.waypointCounter],waypoint[target.waypointCounter + 1]));
        target.latitude = waypoint[target.waypointCounter];
        target.longitude = waypoint[target.waypointCounter + 1];
      } else {
        target.yaw = orient.yaw;
      }
    
      RC_CONTROL_MODE = 1;
    
    } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700){
      if (RC_CONTROL_MODE != 0){
      
      }
    
      RC_CONTROL_MODE = 0;
    
    } else if (channel5Cycle > 1700){
      if (RC_CONTROL_MODE != 2){
        target.alt = orient.alt;
        channels.atPID.setIntegral(channel3Cycle);
        //channels.autoRollPID.updateBounds(channel1Cycle + GPS_ROLL_MAXIMUM, channel1Cycle - GPS_ROLL_MAXIMUM);
        //channels.autoPitchPID.updateBounds(channel2Cycle + GPS_PITCH_MAXIMUM, channel2Cycle - GPS_PITCH_MAXIMUM);
        channels.arPID.setIntegral(rollChannel);
        channels.apPID.setIntegral(pitchChannel);
        target.latitude = orient.latitude;
        target.longitude = orient.longitude;
        target.yaw = orient.yaw;
      }
    
      RC_CONTROL_MODE = 2;
    
    }
    
    modeClockOld = millis();
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

double newMap(int inValue, int inLow, int inHigh, int outLow, int outHigh){
  return ((inValue - inLow) * (1.0 * (outHigh - outLow)) / (1.0 * (inHigh - inLow)) + outLow);
}

void withinBounds(int &value, int upper, int lower){
  if (value > upper){value = upper;}
  if (value < lower){value = lower;}
}

void checkArming(bool &MOTOR_EN){
  if (millis() > INITIAL_ARM_DELAY){
    if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle <= ARM_ENGAGE_THRESHOLD && channel3Cycle > 100 && channel4Cycle > 100){
      MOTOR_EN = 1;
      digitalWrite(13,HIGH);
      
    } else if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle >= ARM_DISENGAGE_THRESHOLD){
      MOTOR_EN = 0;
      digitalWrite(13,LOW);
    }
  }
  //Perhaps Add PID DISABLE Code Here
}
