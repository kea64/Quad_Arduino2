#ifndef H2_TiltComp_h
#define H2_TiltComp_h

#include <Arduino.h>
#include "H2_TiltComp.h"
#include "H2_HMC5883L.h"
#include "H2_ADXL345.h"
#include "H2_ITG3200.h"
#include "H2_L3D4200D.h"
#include "H2_BMP180.h"
#include "H2_Orient.h"
#include "H2_Target.h"
#include "H2_PID.h"
#include "H2_Output.h"
#include "H2_Drone_Values.h"
#include "H2_Support.h"
#include "H2_Controller.h"
#include "H2_Mode.h"
#include "H2_Filters.h"

void calcYaw(class HMC5883L &mag, struct ORIENT_STRUCT &orient);

#endif
