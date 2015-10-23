#ifndef H2_Check_Timing_h
#define H2_Check_Timing_h

#include <Arduino.h>
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

void checkCompli(class L3D4200D &gyro, class ADXL345 &acc, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld);

void checkCompli(class ITG3200 &gyro, class ADXL345 &acc, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld);

void checkCompli(class MPU6050 &mpu, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld);

void checkBaro(class BMP180 &baro, unsigned long &baroClockOld, struct ORIENT_STRUCT &orient);

void checkBaro(class MS5611 &baro, unsigned long &baroClockOld, struct ORIENT_STRUCT &orient);

void checkTemp(class BMP180 &baro, unsigned long &tempClockOld);

void checkAccCalib(class MPU6050 &mpu);

void checkAccCalib(class ADXL345 &accel);

void checkMagCalib(class HMC5883L &mag);

#endif
