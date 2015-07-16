#ifndef H2_Orient_h
#define H2_Orient_h

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

struct ORIENT_STRUCT{
	double roll, pitch, yaw, alt, latitude, longitude;
	double rollGyro, pitchGyro, yawGyro;

	int throttle;
	bool GPS_LOCK;
};

void initAngles(struct ORIENT_STRUCT &orient, class ADXL345 &acc);

#endif
