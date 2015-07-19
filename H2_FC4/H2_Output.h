#ifndef H2_Output_h
#define H2_Output_h

#include <Arduino.h>
#include <Servo.h>
#include "H2_Output.h"
#include "H2_Drone_Values.h"
#include "H2_Support.h"
//#include "H2_Sensors.h"

struct OUTPUT_STRUCT{
	int roll;
	int pitch;
	int throttle;
	int yaw;
};

void processMotors(struct OUTPUT_STRUCT output);

void checkArming(bool &MOTOR_EN, class MS5611 &baro);
void checkArming(bool &MOTOR_EN, class BMP180 &baro);

#endif
