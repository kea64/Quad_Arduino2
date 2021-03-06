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

*/

#include <Servo.h>
#include <Wire.h>
#include <ADXL345.h>
#include <SFE_BMP180.h>
#include <PinChangeInt.h>

float GyroX,GyroY,GyroZ,GyroTemp,GyroTempCelsius,biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ,xAng,yAng,cycle,pitchGyro,rollGyro,pitch,roll,pitchFinal,rollFinal,yaw,accX,accY,accZ,CMy,CMx,altCorrect;
float globalSpeed = 0.0;
float Max_PID = 0.0;
float pitchAccel;
float rollAccel;
float gyroVar = 0.1;
float deltaGyroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1;
float Pvv = 0.1;
float Pxv = 0.1;
float kx,kv;

const float pi = 3.14159;
int aX,aY,aZ;
int g_offx = 0;
int g_offy = 0;
int g_offz = 0;
int totalGyroXValues = 0;
int totalGyroYValues = 0;
int totalGyroZValues = 0;
int i,mX,mY,mZ;
int motor1,motor2,motor3,motor4;
int forward = 0;
int strafe = 0;
int pan = 0;
int IRRaw,IR;
int hoverSpeed = 30;

volatile int channel1Cycle;
volatile int channel2Cycle;

double baseline;
double T,P,p0,a;
double alt = 0.0;
double targetAlt = 1.0;
double ITermP,ITermR,ITermY,lastPitch,lastRoll,lastYaw;
double pitchControl,rollControl,yawControl;

boolean landed_True = 0;
boolean landing_Enable = 0;

char status;

unsigned long kalmanClockNew,kalmanClockOld,baroClockOld,tempClockOld,commClockOld,elevClockOld,landClockOld,channel1Start,channel2Start;

ADXL345 adxl;
SFE_BMP180 pressure;
#define ITG3200_Address 0x68
#define address 0x1E
#define ESC_DELAY 5
#define BARO_DELAY 20
#define TEMP_DELAY 5000
#define COMM_DELAY 500
#define ELEV_DELAY 10
#define LAND_DELAY 20000
#define accConv 0.0039
#define ESC_SCALING 1
#define ESC_MIN 90
#define ESC_MAX 180
#define ROLL_OFFSET -2 //Positive Lowers Aft 4
#define PITCH_OFFSET 4 //Positive Lowers Aft 2
#define YAW_OFFSET 0
#define altAlpha 0.6
#define globalAlpha 0.85
#define forwardAlpha 0.6
#define strafeAlpha 0.6
#define pitchAlpha 0.5
#define rollAlpha 0.5

#define speedAggression 1.5
#define BARO_MODE 1
#define MAX_ALTITUDE 4.0
#define IRPin 0
#define IRAlpha 0.8
#define MAX_PITCH 30.0
#define MIN_PITCH -30.0
#define MAX_ROLL 30.0
#define MIN_ROLL 0.0
#define MAX_YAW 20.0
#define MIN_YAW 0.0
#define MAX_SPEED 60.0

#define kpp 0.07
#define kip 0.08
#define kdp 0.03

#define kpr 0.07
#define kir 0.08
#define kdr 0.03

#define kpy 0.07
#define kiy 0
#define kdy 0

#define pin3 12
#define pin4 11
#define pin1 10
#define pin2 9
#define channel1 8
#define channel2 7
#define RC_ENABLE 1

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

void setup() {
  Wire.begin(); 
  Serial.begin(115200);
  delay(100);
  initGyro(); //Setup Gyro
  initAcc();  //Setup Accelerometer
  initMag();  //Setup Magnetometer
  delay(25);
  GyroCalibrate(); //Factory Setup Routine
  GyroCalibrate2(); //Calibrates Gyro and/or other sensors
  delay(25);
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
  }
  
  kalmanClockOld = millis();
  baroClockOld = millis();
  tempClockOld = millis();
  commClockOld = millis();
  elevClockOld = millis();
  landClockOld = millis();
  
}

void loop() {
  if ((millis() - kalmanClockOld) >= ESC_DELAY){
    kalman(); //Almighty Kalman Filter
    calcYaw(); //Tilt Compensated Compass Code
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
    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print("Roll: ");
    Serial.println(roll);
    Serial.print("Yaw: ");
    Serial.println(yaw);
    Serial.print("Alt: ");
    Serial.println(alt);
    Serial.print("Temp: ");
    Serial.println(T);
    Serial.print("Cycle: ");
    Serial.println(cycle,3);
    Serial.print("Global: ");
    Serial.println(globalSpeed);
    Serial.print("M1: ");
    Serial.println(motor1);
    Serial.print("M2: ");
    Serial.println(motor2);
    Serial.print("M3: ");
    Serial.println(motor3);
    Serial.print("M4: ");
    Serial.println(motor4);
    Serial.print("RControl: ");
    Serial.println(rollControl);
    Serial.print("Strafe: ");
    Serial.println(strafe);
    Serial.print("MPID: ");
    Serial.println(Max_PID);
    
    
    //Serial.print("Gyro: ");
    //Serial.println(rollGyro);
    //Serial.print("Roll: ");
    //Serial.println(roll);
  
  
    /*
    Serial.print("Motor 1: ");
    Serial.println(motor1);
    Serial.print("Motor 2: ");
    Serial.println(motor2);
    Serial.print("Motor 3: ");
    Serial.println(motor3);
    Serial.print("Motor 4: ");
    Serial.println(motor4);
    */
  
    /*
    Serial.print("Cycle: ");
    Serial.println(cycle,4);
    Serial.print("Count: ");
    Serial.println(delayCount);
    Serial.print("Switch: ");
    Serial.println(switchCond);
    Serial.print("Speed: ");
    Serial.println(globalSpeed);
    */
    
    commClockOld = millis();
  }
  
  elev();
  
  land();
  
  freeFallDetector();
  
  
  
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
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
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

 g_offx = 0;
 g_offy = 0;
 g_offz = 0;
 
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

void GyroCalibrate2(){
  for (i = 0; i < 50; i += 1) {
    getGyro();
    getAcc();
    totalGyroXValues += GyroX;
    totalGyroYValues += GyroY;
    totalGyroZValues += GyroZ;
    //totalAccelXValues += aX;
    //totalAccelYValues += aY;
    //totalAccelZValues += aZ;
    delay(20);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  //biasAccelX = totalAccelXValues / 50;
  //biasAccelY = totalAccelYValues / 50;
  //biasAccelZ = (totalAccelZValues / 50) - 256;
  biasAccelX = 1;
  biasAccelY = 1;
  biasAccelZ = 1;
}

void initAngles(){
  getAcc(); //Obtains Initial Angles; Quad must be motionless
  pitchGyro = xAng; //Accounts for Angle Differences
  pitch = xAng;
  rollGyro = yAng;
  roll = yAng;
}

void ARM_Sequence(){
  ESC1.attach(pin1);
  ESC2.attach(pin2);
  ESC3.attach(pin3);
  ESC4.attach(pin4);
  delay(20);
  
  ESC1.write(180);
  ESC2.write(180);
  ESC3.write(180);
  ESC4.write(180);
  delay(2000);
  
  ESC1.write(90);
  ESC2.write(90);
  ESC3.write(90);
  ESC4.write(90);
  delay(3000);
  //delay(1000);
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
  GyroTemp = (buff[0] << 8) | buff[1]; // temperature 
  GyroTempCelsius = 35 + ((GyroTemp + 13200)/280);
 
  
}

void getAcc(){
  adxl.readAccel(&aX, &aY, &aZ);
  accX = aX * accConv;
  accY = aY * accConv;
  accZ = aZ * accConv;
  xAng = atan2(-accX,accZ)*(180/pi) + PITCH_OFFSET;
  yAng = atan2(accY,accZ)*(180/pi) + ROLL_OFFSET;
}

void getMag(){
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    mX = Wire.read()<<8; //X msb
    mX |= Wire.read(); //X lsb
    mZ = Wire.read()<<8; //Z msb
    mZ |= Wire.read(); //Z lsb
    mY = Wire.read()<<8; //Y msb
    mY |= Wire.read(); //Y lsb
  }
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

void kalman(){
  kalmanClockNew = millis(); //Cycle Timing Code
  cycle = (((kalmanClockNew - kalmanClockOld)*1.0)/1000.0);
  getGyro();
  getAcc();
  
  //Pitch Prediction Code
  pitchAccel = atan2(-accX,accZ)*(180.0/pi) + PITCH_OFFSET;
  pitchGyro = pitchGyro + ((GyroX - biasGyroX)/14.375)*cycle;
  pitch = pitch + ((GyroX - biasGyroX)/14.375)*cycle;
  
  //Roll Prediction Code
  rollAccel = atan2(accY,accZ) * 180.0 / pi + ROLL_OFFSET;
  rollGyro = rollGyro - ((-GyroY - biasGyroY) / 14.375) * cycle; 
  roll = roll - ((-GyroY - biasGyroY) / 14.375) * cycle;
  
  //Measurement Mode
  Pxx += cycle * (2 * Pxv + cycle * Pvv);
  Pxv += cycle * Pvv;
  Pxx += cycle * gyroVar;
  Pvv += cycle * deltaGyroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  
  //Finish that Kalman Stuff
  pitch += (pitchAccel - pitch) * kx;
  roll += (rollAccel - roll) * kx;
  
  //Account for Motor Vibrations
  pitchFinal = pitchAlpha * pitchFinal + (1 - pitchAlpha) * pitch;
  rollFinal = rollAlpha * rollFinal + (1 - rollAlpha) * roll;
  
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;
  
  kalmanClockOld = millis();
  //delay(cycleDelay);
}

void calcYaw(){
  getMag();
  CMx = mX * cos(radians(pitch-PITCH_OFFSET)) + mZ * sin(radians(pitch-PITCH_OFFSET)); //Adjusts mX reading
  CMy = mX * sin(radians(roll-ROLL_OFFSET)) * sin(radians(pitch-PITCH_OFFSET)) + mY * cos(radians(roll-ROLL_OFFSET)) - mZ * sin(radians(roll-ROLL_OFFSET)) * cos(radians(pitch-PITCH_OFFSET)); //Adjusts mY Reading
  yaw = atan2(CMy,CMx);
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
  
  altCheck();
  
  //Pitch PID
  double errorPitch = forward - pitchFinal;
  ITermP += (kip * cycle * errorPitch);
  if (ITermP > Max_PID) {ITermP = Max_PID;}
  else if (ITermP < -Max_PID) {ITermP = -Max_PID;}
  double dPitch = (pitchFinal - lastPitch);
  pitchControl = kpp * errorPitch + ITermP - (kdp/cycle) * dPitch;
  if (pitchControl > Max_PID) {pitchControl = Max_PID;}
  if (pitchControl <= -Max_PID) {pitchControl = -Max_PID;}
  lastPitch = pitchFinal;
  
  
  //Roll PID
  double errorRoll = strafe - roll;
  ITermR += (kir * cycle * errorRoll);
  if (ITermR > Max_PID) {ITermR = Max_PID;}
  else if (ITermR < -Max_PID) {ITermR = -Max_PID;}
  double dRoll = (roll - lastRoll);
  rollControl = kpr * errorRoll + ITermR - (kdr/cycle) * dRoll;
  if (rollControl > Max_PID) {rollControl = Max_PID;}
  if (rollControl < -Max_PID) {rollControl = -Max_PID;}
  lastRoll = roll;
  
  //Yaw PID
  double errorYaw = pan - yaw;
  ITermY += (kiy * cycle * errorYaw);
  if (ITermY > Max_PID) {ITermY = Max_PID;}
  else if (ITermY < -Max_PID) {ITermY = -Max_PID;}
  double dYaw = (yaw - lastYaw);
  yawControl = kpy * errorYaw + ITermY - (kdy/cycle) * dYaw;
  if (yawControl > Max_PID) {yawControl = Max_PID;}
  if (yawControl < -Max_PID) {yawControl = -Max_PID;}
  lastYaw = yaw;
  
  //Prepare Motor Variables
  //motor1 = globalSpeed + pitchControl + yawControl + 90;
  //motor2 = globalSpeed - pitchControl + yawControl + 90;
  //motor3 = globalSpeed + rollControl - yawControl + 90;
  //motor4 = globalSpeed - rollControl - yawControl + 90;
  
  motor1 = map((100.0 * (globalSpeed - pitchControl + yawControl + 90.0)),9000.0,18000.0,1500.0,2000.0);
  motor2 = map((100.0 * (globalSpeed + pitchControl + yawControl + 90.0)),9000.0,18000.0,1500.0,2000.0);
  
  motor3 = map((100.0 * (globalSpeed - rollControl - yawControl + 90.0)),9000.0,18000.0,1500.0,2000.0);
  motor4 = map((100.0 * (globalSpeed + rollControl - yawControl + 90.0)),9000.0,18000.0,1500.0,2000.0);
  
  //Correct Over-Saturation
  if (motor1 > 2000) {motor1 = 2000;}
  if (motor2 > 2000) {motor2 = 2000;}
  if (motor3 > 2000) {motor3 = 2000;}
  if (motor4 > 2000) {motor4 = 2000;}
  
  //Correct Under-Saturation
  if (motor1 < 1500) {motor1 = 1500;}
  if (motor2 < 1500) {motor2 = 1500;}
  if (motor3 < 1500) {motor3 = 1500;}
  if (motor4 < 1500) {motor4 = 1500;}
  
  //Update Motors
  ESC1.writeMicroseconds(motor1);
  ESC2.writeMicroseconds(motor2);
  ESC3.writeMicroseconds(motor3);
  ESC4.writeMicroseconds(motor4);
  
  //ESC3.write(globalSpeed + 90);
  //ESC4.write(globalSpeed + 90);
  
}

void elev(){
  if (RC_ENABLE == 0){
    if (!landing_Enable){
      if(millis() - elevClockOld > ELEV_DELAY){
        globalSpeed = (targetAlt - alt) * speedAggression + globalSpeed;
        if (alt >= MAX_ALTITUDE){globalSpeed *= 0.95;}
        if (globalSpeed > MAX_SPEED){globalSpeed = MAX_SPEED;}
        if (globalSpeed <= 0.0){globalSpeed = 0.0;}
        if (abs(targetAlt-alt) < .2) {hoverSpeed = globalSpeed;}
      }
      elevClockOld = millis();
    }
  }
}

void land(){
  if (RC_ENABLE == 0){
    if(millis() - landClockOld > LAND_DELAY){
      landing_Enable = 1;
    }
  
    if (landing_Enable){
      globalSpeed = hoverSpeed * 0.9;
      IRRaw = analogRead(IRPin);
      IR = IRAlpha * IR + (1-IRAlpha) * IRRaw;
      if (IR > 375){
        globalSpeed = 0;
      } else {
        //landed_true = 0;
      }
      //Insert IR CODE
    }
  }
}

void freeFallDetector(){
  if (RC_ENABLE == 0){
    byte interrupts = adxl.getInterruptSource();
    if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
      if(landing_Enable){
        globalSpeed += 40;
      } else {
        targetAlt = alt;
      }
    }
  }
}

void altCheck(){
  if (alt > MAX_ALTITUDE) {globalSpeed *= 0.85;}
}

void channel1Update(){
  
    if (digitalRead(channel1) == 1){
      channel1Start = micros();
    } else {
      channel1Cycle = micros() - channel1Start;
      if (channel1Cycle >= 1590) {channel1Cycle = 1590;}
      if (channel1Cycle <= 1500) {channel1Cycle = 1500;}
      float globalSpeedRaw = channel1Cycle - 1500;
      globalSpeed = globalAlpha * globalSpeed + (1 - globalAlpha) * globalSpeedRaw;
      if (globalSpeed >= MAX_SPEED) {globalSpeed = MAX_SPEED;}
      if (globalSpeed <= 0.0) {globalSpeed = 0.0;}
      if (globalSpeed >= 45.0){Max_PID = 90.0 - globalSpeed;}
      if (globalSpeed < 45.0){Max_PID = globalSpeed;}
    }
}

void channel2Update(){
  
    if (digitalRead(channel2) == 1){
      channel2Start = micros();
    } else {
      channel2Cycle = micros() - channel2Start;
      if (channel2Cycle >= 1590) {channel2Cycle = 1590;}
      if (channel2Cycle <= 1410) {channel2Cycle = 1410;}
      float strafeRaw = int(0.4 * (channel2Cycle - 1500));
      strafe = strafeAlpha * strafe + (1 - strafeAlpha) * strafeRaw;
    }
}
