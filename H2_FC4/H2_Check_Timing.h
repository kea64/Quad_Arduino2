#ifndef H2_Check_Timing_h
#define H2_Check_Timing_h

#include <Arduino.h>
#include "H2_HMC5883L.h"
#include "H2_ADXL345.h"
#include "H2_ITG3200.h"
#include "H2_L3D4200D.h"
#include "H2_BMP180.h"
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

void checkBaro(class BMP180 &baro, unsigned long &baroClockOld, struct ORIENT_STRUCT &orient);

void checkTemp(class BMP180 &baro, unsigned long &tempClockOld);

#endif
