//#define NO_PORTC_PINCHANGES
//#define NO_PORTB_PINCHANGES

#include <Servo.h>
#include <Wire.h>
#include <PinChangeInt.h>
#include <TinyGPS++.h>
#include <EEPROM.h>

#include "H2_Sensors.h"
#include "H2_TiltComp.h"
#include "H2_Orient.h"
#include "H2_Target.h"
#include "H2_PID.h"
#include "H2_Output.h"
#include "H2_Drone_Values.h"
#include "H2_Support.h"
#include "H2_Controller.h"
#include "H2_Mode.h"
#include "H2_Filters.h"
#include "H2_Check_Timing.h"
#include "H2_EEPROM.h"


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
volatile int channel7Cycle;
volatile int channel8Cycle;

int count = 0;
int compli_count = 0;

unsigned long channel1Start, channel2Start,channel3Start,channel4Start;
unsigned long channel5Start,channel6Start,channel7Start,channel8Start;

double channel6Var = 0;

#if defined(SPYDER_EN) || defined(TRI_EN) || defined(HEXACOPTER) || defined(OCTOCOPTER_EN)
  Servo output1;
  Servo output2;
  Servo output3;
  Servo output4;
#endif
#if defined(HEXACOPTER_EN) || defined(OCTOCOPTER_EN)
  Servo output5;
  Servo output6;
#endif
#if defined(OCTOCOPTER_EN)
  Servo output7;
  Servo output8;
#endif
#if defined(ROVER_EN)
  Servo output2;
  Servo output4;
#endif

TinyGPSPlus gps;


//-------------------------------SETUP-----------------------------------//

void setup(){
  Wire.begin();
  Serial.begin(SERIAL0_BAUD);
  #if (defined(GPS_EN) && GPS_SERIAL == 1)
    Serial1.begin(SERIAL1_BAUD);
  #endif
  
  #if defined(PWM_IN)
    pinMode(channel1,INPUT);digitalWrite(channel1,HIGH);PCintPort::attachInterrupt(channel1,&channel1Interrupt,CHANGE);
    pinMode(channel2,INPUT);digitalWrite(channel2,HIGH);PCintPort::attachInterrupt(channel2,&channel2Interrupt,CHANGE);
    pinMode(channel3,INPUT);digitalWrite(channel3,HIGH);PCintPort::attachInterrupt(channel3,&channel3Interrupt,CHANGE);
    pinMode(channel4,INPUT);digitalWrite(channel4,HIGH);PCintPort::attachInterrupt(channel4,&channel4Interrupt,CHANGE);
    pinMode(channel5,INPUT);digitalWrite(channel5,HIGH);PCintPort::attachInterrupt(channel5,&channel5Interrupt,CHANGE);
    pinMode(channel6,INPUT);digitalWrite(channel6,HIGH);PCintPort::attachInterrupt(channel6,&channel6Interrupt,CHANGE);
    pinMode(channel7,INPUT);digitalWrite(channel7,HIGH);PCintPort::attachInterrupt(channel7,&channel7Interrupt,CHANGE);
    pinMode(channel8,INPUT);digitalWrite(channel8,HIGH);PCintPort::attachInterrupt(channel8,&channel8Interrupt,CHANGE);
  #endif
  #if defined(PPM_IN)
    pinMode(PPM_CHANNEL,INPUT);digitalWrite(PPM_CHANNEL,HIGH);PCintPort::attachInterrupt(PPM_CHANNEL,&channel1Interrupt,CHANGE);
  #endif
  
  #if defined(SPYDER_EN) || defined(TRI_EN) || defined(HEXACOPTER) || defined(OCTOCOPTER_EN)
    output1.attach(channel1Pin);
    output2.attach(channel2Pin);
    output3.attach(channel3Pin);
    output4.attach(channel4Pin);
  #endif
  #if defined(HEXACOPTER_EN) || defined(OCTOCOPTER_EN)
    output5.attach(channel5Pin);
    output6.attach(channel6Pin);
  #endif
  #if defined(OCTOCOPTER_EN)
    output7.attach(channel7Pin);
    output8.attach(channel8Pin);
  #endif
  #if defined(ROVER_EN)
    output2.attach(channel2Pin);
    output4.attach(channel4Pin);
  #endif
  //Indicator LED's ... To Be Assigned
  pinMode(RED_LED, OUTPUT);
  //Crius Board Only Features
  #if defined(CRIUS)
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
  #endif
  
  digitalWrite(RED_LED, LOW);
  //Crius Board Only Features
  #if defined(CRIUS)
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  #endif
}

//----------------------------MAIN/SETUP----------------------------------//

void loop(){
  //Sensor Selection
  #if defined(MPU6050_EN)
    MPU6050 mpu;
  #endif
  #if defined(HMC5883L_EN)
    HMC5883L mag;
  #endif
  #if defined(ADXL345_EN)
    ADXL345 accel;
  #endif
  #if defined(L3D4200D_EN)
    L3D4200D gyro;
  #endif
  #if defined(ITG3200_EN)
    ITG3200 gyro;
  #endif
  #if defined(BMP085_EN) || defined(BMP180_EN)
    BMP180 baro;
  #endif
  #if defined(MS5611_EN)
    MS5611 baro;
  #endif
  
  ORIENT_STRUCT orient;
  TARGET_STRUCT target;
  OUTPUT_STRUCT output;
  PID_REGISTER channels;
  
  int rollOverDelay = 0; //Arming Safety Delay
  
  #if defined(SERIAL_COM_EN)
    PACKET_BUFFER packet = {"", "", "", "", ""}; //Prep for Serial COMM
  #endif
  
  output.roll = 0;
  output.pitch = 0;
  output.throttle = 0;
  output.yaw = 0;
  
  byte RC_CONTROL_MODE = 0;
  
  bool MOTOR_EN = 0;
  
  unsigned long compliClockOld, baroClockOld, tempClockOld, commClockOld, controlClockOld, modeClockOld;
  
  //Sensor Initialization
  #if defined(MPU6050_EN)
    mpu.init(MPU6050_GYRO_GAIN, MPU6050_ACCEL_GAIN, MPU6050_DLPF);
  #endif
  #if defined(ITG3200_EN) || defined(L3D4200D_EN)
    gyro.init(GYROALPHA);
  #endif
  #if defined(ADXL345_EN)
    accel.init(ACC_ALPHA);
  #endif
  #if defined(HMC5883L_EN)
    mag.init();
  #endif
  #if defined(BMP180_EN) || defined(BMP085_EN)
    baro.begin(BARO_MODE, altAlpha);
    baro.resetReference();
  #endif
  #if defined(MS5611_EN)
    baro.begin(altAlpha, altVelAlpha);
    baro.resetReference();
  #endif

  #if !defined(ROVER_EN)
    channels.rsPID.updateDefaults(KPRS, KIRS, KDRS, 0, ROLL_STAB_MAXIMUM, -ROLL_STAB_MAXIMUM, KMRS);
    channels.psPID.updateDefaults(KPPS, KIPS, KDPS, 0, PITCH_STAB_MAXIMUM, -PITCH_STAB_MAXIMUM, KMPS);
    channels.ysPID.updateDefaults(KPYS, KIYS, KDYS, 0, YAW_STAB_MAXIMUM, -YAW_STAB_MAXIMUM, KMYS);
  
    channels.rrPID.updateDefaults(KPRR, KIRR, KDRR, 0, ROLL_RATE_MAXIMUM, -ROLL_RATE_MAXIMUM, KMRR);
    channels.prPID.updateDefaults(KPPR, KIPR, KDPR, 0, PITCH_RATE_MAXIMUM, -PITCH_RATE_MAXIMUM, KMPR);
    channels.yrPID.updateDefaults(KPYR, KIYR, KDYR, 0, YAW_RATE_MAXIMUM, -YAW_RATE_MAXIMUM, KMYR);
  
    channels.atPID.updateDefaults(KPT, KIT, KDT, 0, THROTTLE_MAXIMUM, THROTTLE_MINIMUM, KMT);
  #endif
  
  #if defined(GPS_EN)
    channels.arPID.updateDefaults(GPR, GIR, GDR, 0, GPS_ROLL_MAXIMUM, -GPS_ROLL_MAXIMUM, GMR);
    channels.apPID.updateDefaults(GPP, GIP, GDP, 0, GPS_PITCH_MAXIMUM, -GPS_PITCH_MAXIMUM, GMP);
  #endif
  
  delay(500);
  
  #if defined(ITG3200_EN) || defined(L3D4200D_EN)
    gyro.calibrate();
  #endif
  
  #if defined(MPU6050_EN)
    initAngles(orient, mpu);
  #elif defined(ADXL345_EN)
    initAngles(orient, accel);
  #endif
  
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
    #if defined(MPU6050_EN)
      checkCompli(mpu, mag, orient, compliClockOld);
      #if defined(CRIUS)
        checkAccCalib(mpu);
        checkMagCalib(mag);
      #endif
    #elif (defined(ITG3200_EN) || defined(L3D4200D_EN)) && defined(ADXL345_EN)
      checkCompli(gyro, accel, mag, orient, compliClockOld);
      #if defined(CRIUS)
        checkAccCalib(accel);
        checkMagCalib(mag);
      #endif
    #endif
    
    checkBaro(baro, baroClockOld, orient);
    #if defined(BMP180_EN) || defined(BMP085_EN)
      checkTemp(baro, tempClockOld);
    #endif
    
    #if defined(GPS_EN)
      updateGPS(orient, target);
    #endif
    
    #if defined(SERIAL_COM_EN)
      SerialProcess(packet);
    #endif
    
    updateMode(channels, target, orient, RC_CONTROL_MODE, modeClockOld);

    #if defined(ROVER_EN)
      checkArmingRover(MOTOR_EN, rollOverDelay, baro);
    #else
      checkArming(MOTOR_EN, rollOverDelay, baro);
    #endif
    
    if (MOTOR_EN || AUXILIARY_EN){
      updateController(channels, target, orient, output, RC_CONTROL_MODE, controlClockOld);
    } else {
      #if defined(SPYDER_EN) || defined(TRI_EN) || defined(HEXACOPTER_EN) || defined(OCTOCOPTER_EN)
        output1.writeMicroseconds(THROTTLE_MINIMUM);
        output2.writeMicroseconds(THROTTLE_MINIMUM);
        output3.writeMicroseconds(THROTTLE_MINIMUM);
        output4.writeMicroseconds(THROTTLE_MINIMUM);
      #endif
      #if defined(HEXACOPTER_EN) || defined(OCTOCOPTER_EN)
        output5.writeMicroseconds(THROTTLE_MINIMUM);
        output6.writeMicroseconds(THROTTLE_MINIMUM);
      #endif
      #if defined(OCTOCOPTER_EN)
        output7.writeMicroseconds(THROTTLE_MINIMUM);
        output8.writeMicroseconds(THROTTLE_MINIMUM);
      #endif
      #if defined(ROVER_EN)
        output2.writeMicroseconds(THROTTLE_NEUTRAL);
        output4.writeMicroseconds(THROTTLE_NEUTRAL);
      #endif
    }
    
    //Status Feedback
    #if defined(MPU6050_EN) && defined(MS5611_EN)
      transmitData(orient, mpu, baro, output, commClockOld);
    #elif (defined(ITG3200_EN) || defined(L3D4200D_EN)) && defined(ADXL345_EN)
      transmitData(orient, accel, gyro, baro, output, commClockOld); 
    #endif
    
  } 
  
//_______________________________________________________________________// 
//-------------------------=---End Loop----------------------------------//
//_______________________________________________________________________//
}

void updateGPS(struct ORIENT_STRUCT &orient, struct TARGET_STRUCT &target){
  #if (GPS_SERIAL == 0)
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
  #elif (GPS_SERIAL == 1)
    while (Serial1.available() > 0){
      if (gps.encode(Serial1.read())){
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
  #endif
  
}

void transmitData(struct ORIENT_STRUCT &orient, class ADXL345 &acc, class L3D4200D &gyro,  BMP180 baro, struct OUTPUT_STRUCT output, unsigned long &commClockOld){
   if ((millis() - commClockOld) >= COMM_DELAY){
     
     if (EEPROM.read(DEBUG_)){
       
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
       Serial.print("F_Count: ");
       Serial.println(compli_count);
       
     }
     count = 0;
     compli_count = 0;
     commClockOld = millis();
   }
}

void transmitData(struct ORIENT_STRUCT &orient, class MPU6050 mpu,  MS5611 baro, struct OUTPUT_STRUCT output, unsigned long &commClockOld){
   if ((millis() - commClockOld) >= COMM_DELAY){
     
     if (EEPROM.read(DEBUG_) == 1){  
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
       
       /*
       Serial.print("Ch1: ");
       Serial.println(channel1Cycle);
       Serial.print("Ch2: ");
       Serial.println(channel2Cycle);
       Serial.print("Ch3: ");
       Serial.println(channel3Cycle);
       Serial.print("Ch4: ");
       Serial.println(channel4Cycle);
       */
       
       //Serial.print("CH6VAR ");
       //Serial.println(channel6Var);
       Serial.print("F_Count: ");
       Serial.println(compli_count);
       
     }
     count = 0;
     compli_count = 0;
     commClockOld = millis();
   }
}

