
#include "H2_Sensors.h"
#include "H2_TiltComp.h"
#include "H2_Target.h"
#include "H2_PID.h"
#include "H2_Output.h"
#include "H2_Drone_Values.h"
#include "H2_Support.h"
#include "H2_Controller.h"
#include "H2_Mode.h"
#include "H2_Filters.h"
#include "H2_Orient.h"

void initAngles(struct ORIENT_STRUCT &orient, class ADXL345 &acc){
	acc.update(); //Obtains Initial Angles; Quad must be motionless
	orient.roll = atan2(acc.y, acc.z) * RAD_TO_DEGREE + ROLL_OFFSET; //Accounts for Angle Differences
	orient.pitch = atan2(-acc.x, acc.z) * RAD_TO_DEGREE + PITCH_OFFSET;

}

void initAngles(struct ORIENT_STRUCT &orient, class MPU6050 &mpu){
	mpu.readData(); //Obtains Initial Angles; Quad must be motionless
	orient.roll = atan2(mpu.ay, mpu.az) * RAD_TO_DEGREE + ROLL_OFFSET; //Accounts for Angle Differences
	orient.pitch = atan2(-mpu.ax, mpu.az) * RAD_TO_DEGREE + PITCH_OFFSET;

}
