#ifndef H2_Target_h
#define H2_Target_h

#include "Arduino.h"

struct TARGET_STRUCT {
	double roll, pitch, yaw, alt, latitude, longitude, throttle, holdAlt, holdYaw, holdLatitude, holdLongitude;
	byte waypointCounter;

};



#endif