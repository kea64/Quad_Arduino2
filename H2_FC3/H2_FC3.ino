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
#include <H2_Drone_Values.h>
#include <H2_Support.h>
#include <H2_Controller.h>
#include <H2_Mode.h>
#include <H2_Filters.h>
#include <H2_Check_Timing.h>

double waypoint[] = {39.957016, -75.188874, 3.0,    //Waypoints
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

unsigned long channel1Start, channel2Start,channel3Start,channel4Start,channel5Start,channel6Start;

double channel6Var = 0;

Servo output1;
Servo output2;
Servo output3;
Servo output4;

TinyGPSPlus gps;

//HMC5883L mag;
//ADXL345 accel;
//L3D4200D gyro;
//ITG3200 gyro;
//BMP180 baro;
//ORIENT_STRUCT orient;
//TARGET_STRUCT target;
//OUTPUT_STRUCT output;
//PID_REGISTER channels2;

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
  
  output1.attach(channel1Pin);
  output2.attach(channel2Pin);
  output3.attach(channel3Pin);
  output4.attach(channel4Pin);
  
  pinMode(13,OUTPUT); //Satellite Lock Indicator and Arming Indicator
  digitalWrite(13,LOW);
  
}

//----------------------------MAIN/SETUP----------------------------------//

void loop(){
  
  HMC5883L mag;     
  ADXL345 accel;
  L3D4200D gyro;
  //ITG3200 gyro;
  BMP180 baro;
  ORIENT_STRUCT orient;
  TARGET_STRUCT target;
  OUTPUT_STRUCT output;
  PID_REGISTER channels;
  
  output.roll = 0;
  output.pitch = 0;
  output.throttle = 0;
  output.yaw = 0;
  
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
  compliClockOld = millis();
  baroClockOld = millis();
  tempClockOld = millis();
  commClockOld = millis();
  controlClockOld = millis();
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
      output1.writeMicroseconds(THROTTLE_MINIMUM);
      output2.writeMicroseconds(THROTTLE_MINIMUM);
      output3.writeMicroseconds(THROTTLE_MINIMUM);
      output4.writeMicroseconds(THROTTLE_MINIMUM);
    }
    
    //Status Feedback
    transmitData(orient, accel, gyro, baro, output, commClockOld); 
    
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

void transmitData(struct ORIENT_STRUCT &orient, class ADXL345 &acc, class L3D4200D &gyro,  BMP180 baro, struct OUTPUT_STRUCT output, unsigned long &commClockOld){
   if ((millis() - commClockOld) >= COMM_DELAY){
     
     if (DEBUG_EN){
       
       /*
       Serial.print("AccX: ");
       Serial.print(acc.x);
       Serial.print(" AccY: ");
       Serial.print(acc.y);
       Serial.print(" AccZ: ");
       Serial.print(acc.z);
       
       Serial.print(" GyroX: ");
       Serial.print(gyro.x);
       Serial.print(" GyroY: ");
       Serial.print(gyro.y);
       Serial.print(" GyroZ: ");
       Serial.print(gyro.z);
       
       Serial.print(" Baro: ");
       Serial.println(baro.alt);
       
       */
       
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
     
       /*
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
       
       */
       
       
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

