#include <Arduino.h>
#include "H2_TiltComp.h"
#include "H2_Sensors.h"
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
        #if defined(CRIUS)
          double CMx = mag.yScaled * cos(radians(-(orient.pitch - PITCH_OFFSET))) + mag.zScaled * sin(radians(-(orient.pitch - PITCH_OFFSET))); //Adjusts mX reading
	  double CMy = mag.yScaled * sin(radians(orient.roll - ROLL_OFFSET)) * sin(radians(-(orient.pitch - PITCH_OFFSET))) - mag.xScaled * cos(radians(orient.roll - ROLL_OFFSET)) - mag.zScaled * sin(radians(orient.roll - ROLL_OFFSET)) * cos(radians(-(orient.pitch - PITCH_OFFSET))); //Adjusts mY Reading
          orient.yaw = atan2(CMy, CMx) - radians(YAW_OFFSET);
          //orient.yaw = atan2(-mag.xScaled, mag.yScaled) - radians(YAW_OFFSET);
        #elif defined(NANO)
	  double CMx = mag.xScaled * cos(radians(orient.pitch - PITCH_OFFSET)) + mag.zScaled * sin(radians(orient.pitch - PITCH_OFFSET)); //Adjusts mX reading
	  double CMy = mag.xScaled * sin(radians(orient.roll - ROLL_OFFSET)) * sin(radians(orient.pitch - PITCH_OFFSET)) + mag.yScaled * cos(radians(orient.roll - ROLL_OFFSET)) - mag.zScaled * sin(radians(orient.roll - ROLL_OFFSET)) * cos(radians(orient.pitch - PITCH_OFFSET)); //Adjusts mY Reading
	  orient.yaw = atan2(CMy, CMx) - radians(YAW_OFFSET);
        #endif
        
	if (orient.yaw < 0){ orient.yaw += 2 * PI; }
	if (orient.yaw > 2 * PI) { orient.yaw -= 2 * PI; }
	orient.yaw = orient.yaw * (180 / PI);
	if (orient.yaw <= 360 && orient.yaw > 180) { orient.yaw -= 360; }
}
