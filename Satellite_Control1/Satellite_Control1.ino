#define NO_PORTC_PINCHANGES
#define NO_PORTB_PINCHANGES

#include <Servo.h>
#include <Wire.h>
#include <PinChangeInt.h>
#include <TinyGPS++.h>
#include <Quad_HMC5883L.h>
#include <Quad_ADXL345.h>
#include <Quad_L3D4200D.h>
#include <Quad_BMP180.h>
#include <Quad_Registers.h>

#define xMagError 0.96
#define yMagError 1.01
#define zMagError 0.95
#define xMagOffset -23
#define yMagOffset -102
#define zMagOffset 0
#define ROLL_OFFSET 0 //Positive Lowers Aft 4
#define PITCH_OFFSET 0 //Positive Lowers Aft 2
#define YAW_OFFSET 0
#define ACC_SCALAR 0.93

#define altAlpha 0.9
#define compliAlpha 0.98

#define BARO_MODE 3
#define COMPLI_DELAY 5
#define BARO_DELAY 20
#define TEMP_DELAY 1000
#define COMM_DELAY 250
#define ELEV_DELAY 10
#define LAND_DELAY 20000

HMC5883L mag;     
ADXL345 accel;
L3D4200D gyro;
BMP180 baro;
ORIENTATION_REGISTER orient;

//-------------------------------SETUP-----------------------------------//

void setup(){
  Wire.begin();
  Serial.begin(38400);
  
  gyro.init();
  accel.init();
  mag.init(xMagError, yMagError, zMagError, xMagOffset, yMagOffset, zMagOffset);
  baro.begin(BARO_MODE, altAlpha);
  //mode.init(0);
  
}

//----------------------------MAIN/SETUP----------------------------------//

void loop(){
  int elevatorInitial, aileronInitial;
  
  double cycle;
  double ITermT, ITermP, ITermR;
  
  unsigned long compliClockNew, compliClockOld, baroClockOld, tempClockOld, commClockOld, elevClockOld, landClockOld;

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
    
    transmitData(orient, baro, commClockOld);
  }
  
}

void checkCompli(class L3D4200D &gyro, class ADXL345 &acc, class ORIENTATION_REGISTER &orient, unsigned long &compliClockOld){
   //Main Sensor Reading and Motor Control
  if ((millis() - compliClockOld) >= COMPLI_DELAY){
    compli(gyro, accel, orient, compliClockOld); //Complimentary Filter
    calcYaw(orient); //Tilt Compensated Compass Code
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
  
  compliClockOld = millis();
}

void calcYaw(class ORIENTATION_REGISTER &orient){
  mag.update();
  //double CMx = compass_x_scalled * cos(radians(oriRegister.pitch-PITCH_OFFSET)) + compass_z_scalled * sin(radians(oriRegister.pitch-PITCH_OFFSET)); //Adjusts mX reading
  //double CMy = compass_x_scalled * sin(radians(oriRegister.roll-ROLL_OFFSET)) * sin(radians(oriRegister.pitch-PITCH_OFFSET)) + compass_y_scalled * cos(radians(oriRegister.roll-ROLL_OFFSET)) - compass_z_scalled * sin(radians(oriRegister.roll-ROLL_OFFSET)) * cos(radians(oriRegister.pitch-PITCH_OFFSET)); //Adjusts mY Reading
  double CMx = mag.xScaled * cos(radians(orient.pitch-PITCH_OFFSET)) + mag.zScaled * sin(radians(orient.pitch-PITCH_OFFSET)); //Adjusts mX reading
  double CMy = mag.xScaled * sin(radians(orient.roll-ROLL_OFFSET)) * sin(radians(orient.pitch-PITCH_OFFSET)) + mag.yScaled * cos(radians(orient.roll-ROLL_OFFSET)) - mag.zScaled * sin(radians(orient.roll-ROLL_OFFSET)) * cos(radians(orient.pitch-PITCH_OFFSET)); //Adjusts mY Reading
  
  orient.yaw = atan2(CMy,CMx) - radians(YAW_OFFSET);
  if (orient.yaw < 0){orient.yaw += 2*PI;}
  if (orient.yaw > 2*PI) {orient.yaw -= 2*PI;}
  orient.yaw = orient.yaw * (180/PI);
  if (orient.yaw <= 360 && orient.yaw > 180) {orient.yaw -= 360;}
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
