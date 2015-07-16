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

void calcYaw(class HMC5883L &mag, struct ORIENT_STRUCT &orient){
	mag.update();
	
	double CMx = mag.xScaled * cos(radians(orient.pitch - PITCH_OFFSET)) + mag.zScaled * sin(radians(orient.pitch - PITCH_OFFSET)); //Adjusts mX reading
	double CMy = mag.xScaled * sin(radians(orient.roll - ROLL_OFFSET)) * sin(radians(orient.pitch - PITCH_OFFSET)) + mag.yScaled * cos(radians(orient.roll - ROLL_OFFSET)) - mag.zScaled * sin(radians(orient.roll - ROLL_OFFSET)) * cos(radians(orient.pitch - PITCH_OFFSET)); //Adjusts mY Reading

	orient.yaw = atan2(CMy, CMx) - radians(YAW_OFFSET);
	if (orient.yaw < 0){ orient.yaw += 2 * PI; }
	if (orient.yaw > 2 * PI) { orient.yaw -= 2 * PI; }
	orient.yaw = orient.yaw * (180 / PI);
	if (orient.yaw <= 360 && orient.yaw > 180) { orient.yaw -= 360; }
}
