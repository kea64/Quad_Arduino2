/*
Q1:
Set up the basics of Quad Feedback and Flight
Used delays to run loops
Kalman Used independent timing code to accurately feed gyro
Simple On/Off Flight to test Stabilization Used

Q2:
Added IR Altitude Sensor
Added code to initiate state control -- Delays are rarely used
Fixed a Major Bug which lead to improper initial kalman pitch predictions
Added Elev()- Adjusts Global Speed to Change and Maintain a Height
Added Land()- Prepares the Landing Procedure
Included the IR Sensor for Landing and the Barometer for Elevation
Re-Added Free-Fall Detection

Q3:
Added RC Control

Q4:
Moved All Motor Control to PID Control
Improved Motor Control by seperating Yaw,Roll,Pitch, and Global Speed Variables
Included individual limits on yaw,roll,pitch, and global speeed
Added a second RC channel for PID Pinned Testing
Realized a single processor is inadequate to stabilize flight

Q5:
Prepared the Arduino as the Main Controller/Interceptor. A dedicated RC receiver will send signals to the Arduino, which may alter signals, and then sends commands to a dedicated flight controller
Added Channel 5 to Control Auto-Pilot
Added Turn North Control for yaw with PID tuning
Added Altitude Hold Control for autopilot
Calibrated Compass and associated equipment
Added GPS Unit at hardware serial receiver. NOTICE: GPS will NOT work with software serial
Added basic GPS navigation
Tuned and Improved Altitude Hold Code
Cleaned up code by eliminating unneccessary functions and variables such as GyroCalibrate2 and freefalldetector
Added Channel 6 for PID tuning

Q6:
Removed Kalman Filter and Replaced it with Complimentary Filter -- Less Memory, Same Performance
Tuned Magnetometer for better precision, accuracy, and tilt compensation

Q7: 
Reverted to No-FC Control System
Re-Added Re-Arm Sequence
Re-Added Pitch and Roll PID'

*/

#include <Servo.h>
#include <Wire.h>
#include <ADXL345.h>
#include <SFE_BMP180.h>
#include <PinChangeInt.h>
#include <TinyGPS++.h>
#include <compass.h>

float GyroX,GyroY,GyroZ,cycle,pitch,roll,yaw,accX,accY,accZ,CMy,CMx;
float pitchAccel;
float rollAccel;

int aX,aY,aZ;
int distanceToWaypoint;
int Aileron;
int Elevator;
int Throttle;
int Rudder;
int motor1,motor2,motor3,motor4;
int g_offx = 0;
int g_offy = 0;
int g_offz = 0;
int totalGyroXValues = 0;
int totalGyroYValues = 0;
int totalGyroZValues = 0;
int targetHeading = 0;
int IRRaw,IR;
int hoverSpeed = 30;
int aileronOut = 1500;
int elevatorOut = 1500;
int throttleOut = 1000;
int rudderOut = 1500;
int waypointCounter = 0;
int RC_CONTROL_MODE = 0;
int THROTTLE_MAX = 1800;
int THROTTLE_MIN = 1130;

volatile int channel1Cycle;
volatile int channel2Cycle;
volatile int channel3Cycle;
volatile int channel4Cycle;
volatile int channel5Cycle;
volatile int channel6Cycle;

double baseline;
double T,P,p0,a;
double alt = 0.0;
double lastAlt = 0.0;
double targetAlt = 1.0;
double ITermP,ITermR,ITermY,ITermT,lastPitch,lastRoll,lastYaw;
double pitchControl,rollControl,yawControl,errorYaw,throttleControl,errorThrottle;
double channel6Var = 0.0;

static const int numWaypoint = 3;
static const double waypoint[] = {40.653065, -76.959676, 3.0,    //Waypoints
                                  40.652973, -76.959376, 3.0,
                                  40.653278, -76.959030, 3.0,
                                  40.653374, -76.959178, 3.0
                                                            };

boolean landed_True = 0;
boolean landing_Enable = 0;
boolean MODE_CHANGE = 0;

char status;

unsigned long compliClockNew,compliClockOld,baroClockOld,tempClockOld,commClockOld,elevClockOld,landClockOld,channel1Start,channel2Start,channel3Start,channel4Start,channel5Start,channel6Start;

ADXL345 adxl;
TinyGPSPlus gps;
SFE_BMP180 pressure;

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
#define ESC_SCALING 1
#define ESC_MIN 90
#define ESC_MAX 180
#define ROLL_OFFSET 0 //Positive Lowers Aft 4
#define PITCH_OFFSET 0 //Positive Lowers Aft 2
#define YAW_OFFSET 0
#define altAlpha 0.6
#define globalAlpha 0.85
#define forwardAlpha 0.6
#define strafeAlpha 0.6
#define compliAlpha 0.98

#define speedAggression 1.5
#define BARO_MODE 3
#define MAX_ALTITUDE 15.0
#define IRPin 0
#define IRAlpha 0.8
#define MAX_YAW 20.0
#define MAX_THROTTLE 1790
#define MIN_THROTTLE 1350
#define MAX_PID 25

#define kpe 0
#define kie 0
#define kde 0

#define kpa 0.5
#define kia 0
#define kda 0.1

//#define kpy 0.2
//#define kiy 15
//#define kdy 0.1

#define kpy 0
#define kiy 0
#define kdy 0

//#define kpt 6.5
//#define kit 0.5
//#define kdt 0

#define kpt 5.5
#define kit 0.2
#define kdt 5

#define motor1Pin A3
#define motor2Pin A2
#define motor3Pin A1
#define motor4Pin A0
#define channel1 4
#define channel2 5
#define channel3 6
#define channel4 7
#define channel5 8
#define channel6 9
#define RC_ENABLE 1

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

void setup() {
  Wire.begin(); 
  Serial.begin(38400);
  
  compass_x_offset = -150;
  compass_y_offset = -20;
  compass_z_offset = 175;
  compass_x_gainError = 0.94;
  compass_y_gainError = 0.99;
  compass_z_gainError = 0.94;
  
  initGyro(); //Setup Gyro
  initAcc();  //Setup Accelerometer
  initMag();  //Setup Magnetometer
  GyroCalibrate(); //Factory Setup Routine
  initAngles(); //Sets the Gyro Angles to initially match the accelerometers
  
  pressure.begin();
  getTemp();
  getBaro();
  baseline = P;
  ARM_Sequence();
  
  adxl.setFreeFallThreshold(8); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(20); //(20 - 70) recommended - 5ms per increment
 
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  
  if (RC_ENABLE == 1){
    pinMode(channel1,INPUT);digitalWrite(channel1,HIGH);PCintPort::attachInterrupt(channel1,&channel1Update,CHANGE);
    pinMode(channel2,INPUT);digitalWrite(channel2,HIGH);PCintPort::attachInterrupt(channel2,&channel2Update,CHANGE);
    pinMode(channel3,INPUT);digitalWrite(channel3,HIGH);PCintPort::attachInterrupt(channel3,&channel3Update,CHANGE);
    pinMode(channel4,INPUT);digitalWrite(channel4,HIGH);PCintPort::attachInterrupt(channel4,&channel4Update,CHANGE);
    pinMode(channel5,INPUT);digitalWrite(channel5,HIGH);PCintPort::attachInterrupt(channel5,&channel5Update,CHANGE);
    pinMode(channel6,INPUT);digitalWrite(channel6,HIGH);PCintPort::attachInterrupt(channel6,&channel6Update,CHANGE);
  }
  
  pinMode(13,OUTPUT); //GPS Lock Indicator
  digitalWrite(13,LOW);
  
  compliClockOld = millis();
  baroClockOld = millis();
  tempClockOld = millis();
  commClockOld = millis();
  elevClockOld = millis();
  landClockOld = millis();
  
}

void loop() {
  if ((millis() - compliClockOld) >= COMPLI_DELAY){
    compli(); //Complimentary Filter
    calcYaw(); //Tilt Compensated Compass Code
    //yawPID();
    motorUpdate();
  }
  
  if ((millis() - baroClockOld) >= BARO_DELAY){
    getBaro();
    calcAlt();
    baroClockOld = millis();
  }
  
  if ((millis() - tempClockOld) >= TEMP_DELAY){
    getTemp();
    tempClockOld = millis();
  }
  
  if ((millis() - commClockOld) >= COMM_DELAY){
    
    Serial.print("C1: ");
    Serial.println(channel1Cycle);
    Serial.print("C2: ");
    Serial.println(channel2Cycle);
    Serial.print("C3: ");
    Serial.println(channel3Cycle);
    Serial.print("C4: ");
    Serial.println(channel4Cycle);
    Serial.print("C5: ");
    Serial.println(channel5Cycle);
    Serial.print("C6: ");
    Serial.println(channel6Cycle);
    Serial.print("M1: ");
    Serial.println(motor1);
    Serial.print("M2: ");
    Serial.println(motor2);
    Serial.print("M3: ");
    Serial.println(motor3);
    Serial.print("M4: ");
    Serial.println(motor4);
    Serial.print("Roll: ");
    Serial.println(roll);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print("CH6: ");
    Serial.println(channel6Var);
    
    /*
    Serial.print("C 3: ");
    Serial.println(channel3Cycle);
    Serial.print("C 4: ");
    Serial.println(channel4Cycle);
    Serial.print("C 5: ");
    Serial.println(channel5Cycle);
    Serial.print("RC Mode: ");
    Serial.println(RC_CONTROL_MODE);
    Serial.print("Thr Out ");
    Serial.println(throttleOut);
    Serial.print("Alt: ");
    Serial.println(alt);
    Serial.print("TargetAlt: ");
    Serial.println(targetAlt);
    Serial.print("YawC: ");
    Serial.println(rudderOut);
    Serial.print("T Heading: ");
    Serial.println(targetHeading);
    Serial.print("Yaw: ");
    Serial.println(yaw);
    Serial.print("Rud: ");
    Serial.println(rudderOut);
    Serial.print("Ch6 Var: ");
    Serial.println(channel6Var);
    */
    
    commClockOld = millis();
  }
  
  getGPS(); //Update GPS Data
  
  //elevPID();
    
}

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

void initMag(){
  compass_init(2);
}

void initAcc(){
  adxl.powerOn();
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(45);
}

void GyroCalibrate(){

 int tmpx = 0;
 int tmpy = 0;
 int tmpz = 0; 

 for (char i = 0;i<10;i++)
    {
    delay(10);  
    getGyro();
    tmpx += GyroX;
    tmpy += GyroY;
    tmpz += GyroZ; 
    }  
 g_offx = tmpx/10;
 g_offy = tmpy/10;
 g_offz = tmpz/10;
 
}

void initAngles(){
  getAcc(); //Obtains Initial Angles; Quad must be motionless
  pitch = atan2(-accX,accZ)*(180/PI) + PITCH_OFFSET; //Accounts for Angle Differences
  roll = atan2(accY,accZ)*(180/PI) + ROLL_OFFSET;
}

void ARM_Sequence(){
  ESC1.attach(motor1Pin);
  ESC2.attach(motor2Pin);
  ESC3.attach(motor3Pin);
  ESC4.attach(motor4Pin);
  delay(20);
  
  ESC1.writeMicroseconds(THROTTLE_MAX);
  ESC2.writeMicroseconds(THROTTLE_MAX);
  ESC3.writeMicroseconds(THROTTLE_MAX);
  ESC4.writeMicroseconds(THROTTLE_MAX);
  delay(2000);
  
  ESC1.writeMicroseconds(THROTTLE_MIN);
  ESC2.writeMicroseconds(THROTTLE_MIN);
  ESC3.writeMicroseconds(THROTTLE_MIN);
  ESC4.writeMicroseconds(THROTTLE_MIN);
  
}

void getGyro() {
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
    
  GyroX = ((buff[4] << 8) | buff[5]) - g_offx;
  GyroY = ((buff[2] << 8) | buff[3]) - g_offy;
  GyroZ = ((buff[6] << 8) | buff[7]) - g_offz;
  //GyroTemp = (buff[0] << 8) | buff[1]; // temperature 
  //GyroTempCelsius = 35 + ((GyroTemp + 13200)/280);
 
  
}

void getAcc(){
  adxl.readAccel(&aX, &aY, &aZ);
  accX = aX * accConv;
  accY = aY * accConv;
  accZ = aZ * accConv;
}

void getMag(){
  compass_scalled_reading();
}

void getBaro(){
  status = pressure.startPressure(BARO_MODE);
  if (status != 0){
    delay(status);
    status = pressure.getPressure(P,T);
  }      
}

void getTemp(){
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
  }
}

void getGPS(){
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      targetHeading = int(TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),waypoint[waypointCounter],waypoint[waypointCounter + 1]));
      if (targetHeading > 180){targetHeading -= 360;}
      distanceCheck();
      
      if (gps.satellites.value() >= 5){ //GPS Lock Indicator
        digitalWrite(13,HIGH);
      } else {
        digitalWrite(13,LOW);
      }
    }
  }
}

void compli(){
  compliClockNew = millis(); //Cycle Timing Code
  cycle = (((compliClockNew - compliClockOld)*1.0)/1000.0);
  getGyro();
  getAcc();
  
  pitchAccel = atan2(-accX,accZ)*(180.0/PI)*ACC_SCALAR + PITCH_OFFSET;
  pitch = compliAlpha * (pitch + ((GyroX)/14.375) * cycle) + (1 - compliAlpha) * pitchAccel;
  
  rollAccel = atan2(accY,accZ)*(180.0/PI)*ACC_SCALAR + ROLL_OFFSET;
  roll = compliAlpha * (roll + ((GyroY)/14.375) * cycle) + (1 - compliAlpha) * rollAccel;
  
  compliClockOld = millis();
}

void calcYaw(){
  getMag();
  CMx = compass_x_scalled * cos(radians(pitch-PITCH_OFFSET)) + compass_z_scalled * sin(radians(pitch-PITCH_OFFSET)); //Adjusts mX reading
  CMy = compass_x_scalled * sin(radians(roll-ROLL_OFFSET)) * sin(radians(pitch-PITCH_OFFSET)) + compass_y_scalled * cos(radians(roll-ROLL_OFFSET)) - compass_z_scalled * sin(radians(roll-ROLL_OFFSET)) * cos(radians(pitch-PITCH_OFFSET)); //Adjusts mY Reading
  yaw = atan2(CMy,CMx) - radians(YAW_OFFSET);
  if (yaw < 0){yaw += 2*PI;}
  if (yaw > 2*PI) {yaw -= 2*PI;}
  yaw = yaw * (180/PI);
  if (yaw <= 360 && yaw > 180) {yaw -= 360;}
}

void calcAlt(){
  a = pressure.altitude(P,baseline);
  alt = altAlpha * alt + (1-altAlpha) * a;
}

void motorUpdate(){
  //Pitch PID
  double errorPitch = map(Elevator,1000,2000,-45,45) - pitch;
  ITermP += (kie * cycle * errorPitch);
  if (ITermP > MAX_PID) {ITermP = MAX_PID;}
  else if (ITermP < -MAX_PID) {ITermP = -MAX_PID;}
  double dPitch = (pitch - lastPitch);
  pitchControl = kpe * errorPitch + ITermP + (kde/cycle) * dPitch;
  if (pitchControl > MAX_PID) {pitchControl = MAX_PID;}
  if (pitchControl <= -MAX_PID) {pitchControl = -MAX_PID;}
  lastPitch = pitch;
  
  
  //Roll PID
  double errorRoll = map(Aileron,1000,2000,-45,45) - roll;
  ITermR += (channel6Var * cycle * errorRoll);
  if (ITermR > MAX_PID) {ITermR = MAX_PID;}
  else if (ITermR < -MAX_PID) {ITermR = -MAX_PID;}
  double dRoll = (roll - lastRoll);
  rollControl = kpa * errorRoll + ITermR - (kda/cycle) * dRoll;
  if (rollControl > MAX_PID) {rollControl = MAX_PID;}
  if (rollControl < -MAX_PID) {rollControl = -MAX_PID;}
  lastRoll = roll;
  
  //Yaw PID
  double errorYaw = map(Rudder,1000,2000,-90,90) - yaw;
  ITermY += (kiy * cycle * errorYaw);
  if (ITermY > MAX_PID) {ITermY = MAX_PID;}
  else if (ITermY < -MAX_PID) {ITermY = -MAX_PID;}
  double dYaw = (yaw - lastYaw);
  yawControl = kpy * errorYaw + ITermY + (kdy/cycle) * dYaw;
  if (yawControl > MAX_PID) {yawControl = MAX_PID;}
  if (yawControl < -MAX_PID) {yawControl = -MAX_PID;}
  lastYaw = yaw;
  
  motor1 = (Throttle - pitchControl);
  motor3 = (Throttle + pitchControl);
  motor2 = (Throttle - rollControl);
  motor4 = (Throttle + rollControl);
  
  //Correct Over-Saturation
  if (motor1 > THROTTLE_MAX) {motor1 = THROTTLE_MAX;}
  if (motor2 > THROTTLE_MAX) {motor2 = THROTTLE_MAX;}
  if (motor3 > THROTTLE_MAX) {motor3 = THROTTLE_MAX;}
  if (motor4 > THROTTLE_MAX) {motor4 = THROTTLE_MAX;}
  
  //Correct Under-Saturation
  if (motor1 < THROTTLE_MIN) {motor1 = THROTTLE_MIN;}
  if (motor2 < THROTTLE_MIN) {motor2 = THROTTLE_MIN;}
  if (motor3 < THROTTLE_MIN) {motor3 = THROTTLE_MIN;}
  if (motor4 < THROTTLE_MIN) {motor4 = THROTTLE_MIN;}
  
  if (Throttle < 1140){
   motor1 = THROTTLE_MIN;
   motor2 = THROTTLE_MIN; 
   motor3 = THROTTLE_MIN;
   motor4 = THROTTLE_MIN;
  }
  
  //Update Motors
  //ESC1.writeMicroseconds(motor1);
  ESC2.writeMicroseconds(motor2);
  //ESC3.writeMicroseconds(motor3);
  ESC4.writeMicroseconds(motor4);
  
  
}
/*
void yawPID(){
  if (RC_CONTROL_MODE == 1){
    //Yaw PID
    errorYaw = yaw - targetHeading;
    if (errorYaw <= -180){errorYaw += 360;}
    if (errorYaw > 180){errorYaw -= 360;}
    ITermY += (kiy * 0.001 * cycle * errorYaw);
    if (ITermY > MAX_YAW) {ITermY = MAX_YAW;}
    else if (ITermY < -MAX_YAW) {ITermY = -MAX_YAW;}
    yawControl = kpy * errorYaw + ITermY - ((channel6Var * (yaw - lastYaw))/(cycle));
    if (yawControl > MAX_YAW) {yawControl = MAX_YAW;}
    if (yawControl < -MAX_YAW) {yawControl = -MAX_YAW;}
    lastYaw = yaw;
    rudderOut = map(yawControl, -90, 90, 1000, 2000)-1500;
    
  }
}
*/

/*
void elevPID(){
  if (RC_ENABLE == 0 || RC_CONTROL_MODE == 1 || RC_CONTROL_MODE == 2){
    if (!landing_Enable){
      if(millis() - elevClockOld > ELEV_DELAY){
        errorThrottle = targetAlt - alt;
        ITermT += (channel6Var * 0.001 * int(millis() - elevClockOld) * errorThrottle);
        if (ITermT > MAX_THROTTLE) {ITermT = MAX_THROTTLE;}
        else if (ITermT < MIN_THROTTLE) {ITermT = MIN_THROTTLE;}
        throttleControl = kpt * errorThrottle + ITermT - ((kdt * (alt - lastAlt))/(0.001 * (millis() - elevClockOld)));
        if (throttleControl > MAX_THROTTLE) {throttleControl = MAX_THROTTLE;}
        if (throttleControl < MIN_THROTTLE) {throttleControl = MIN_THROTTLE;}
        throttleOut = throttleControl;
        lastAlt = alt;
      }
      elevClockOld = millis();
    }
    Throttle.writeMicroseconds(throttleOut);
  }
}
*/

void land(){
  if (RC_ENABLE == 0){
    if(millis() - landClockOld > LAND_DELAY){
      landing_Enable = 1;
    }
  
    if (landing_Enable){
      //globalSpeed = hoverSpeed * 0.9;
      IRRaw = analogRead(IRPin);
      IR = IRAlpha * IR + (1-IRAlpha) * IRRaw;
      if (IR > 375){
        //globalSpeed = 0;
      } else {
        //landed_true = 0;
      }
      //Insert IR CODE
    }
  }
}

void distanceCheck(){
  distanceToWaypoint = int(TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),waypoint[waypointCounter],waypoint[waypointCounter + 1]));
  if(distanceToWaypoint <= 3){
    waypointCounter += 3;
    if ((waypointCounter/3) > numWaypoint){
     waypointCounter -= 3; 
    }
  }
}

void channel1Update(){
  if (RC_CONTROL_MODE == 0){
    if (digitalRead(channel1) == 1){
      channel1Start = micros();
    } else {
      channel1Cycle = micros() - channel1Start;
      Aileron = channel1Cycle;
    }
  } else if (RC_CONTROL_MODE == 1 || RC_CONTROL_MODE == 2){
    if (digitalRead(channel1) == 1){
      channel1Start = micros();
    } else {
      channel1Cycle = micros() - channel1Start;
      Aileron = channel1Cycle;
    }
  }
}

void channel2Update(){
  if (RC_CONTROL_MODE == 0){
    if (digitalRead(channel2) == 1){
      channel2Start = micros();
    } else {
      channel2Cycle = micros() - channel2Start;
      Elevator = channel2Cycle;
    }
  } else if (RC_CONTROL_MODE == 1 || RC_CONTROL_MODE == 2){
    if (digitalRead(channel2) == 1){
      channel2Start = micros();
    } else {
      channel2Cycle = micros() - channel2Start;
      Elevator = channel2Cycle;
    }
  }
}

void channel3Update(){
    if (digitalRead(channel3) == 1){
      channel3Start = micros();
    } else {
      channel3Cycle = micros() - channel3Start;
    }
    
    if (RC_CONTROL_MODE == 0 || RC_CONTROL_MODE == 1){
      Throttle = channel3Cycle;
    } else if (RC_CONTROL_MODE == 2){
      //Extra mode Here
    }
}

void channel4Update(){
  if (RC_CONTROL_MODE == 0 || RC_CONTROL_MODE == 2){
    if (digitalRead(channel4) == 1){
      channel4Start = micros();
    } else {
      channel4Cycle = micros() - channel4Start;
      Rudder = channel4Cycle;
    }
  }
}

void channel5Update(){
  
    if (digitalRead(channel5) == 1){
      channel5Start = micros();
    } else {
      channel5Cycle = micros() - channel5Start;
      if (channel5Cycle < 1300){
        if (RC_CONTROL_MODE != 1){
          targetAlt = alt;
          ITermT = channel3Cycle;
        }
        RC_CONTROL_MODE = 1;
        
      } else if (channel5Cycle >= 1300 && channel5Cycle <= 1700) {
        RC_CONTROL_MODE = 0; //For Safety Purposes
        
      } else if (channel5Cycle > 1700) {
        if (RC_CONTROL_MODE != 2){
          targetAlt = alt;
          ITermT = channel3Cycle;
        }
        RC_CONTROL_MODE = 2;
      }
        
    }
}

void channel6Update(){
    if (digitalRead(channel6) == 1){
      channel6Start = micros();
    } else {
      channel6Cycle = micros() - channel6Start;
      channel6Var = 0.005 * (channel6Cycle - 1000);
      if (channel6Var < 0.0) { channel6Var = 0.0;}
    }
}
