#include <Arduino.h>
#include <Wire.h>
#include "H2_Channel.h"
#include "H2_Registers.h"
#include "H2_ADXL345.h"

#define SERVO_OFFSET 1000
#define SERVO_CONV 0.18
#define SERVO_CONV_SP 0.09

int CHANNEL_REGISTER::servo2HalfDegrees(int servoIn){
	int degreesOut = (servoIn - SERVO_OFFSET) * SERVO_CONV;
	return(degreesOut);
}

int CHANNEL_REGISTER::servo2QuarterDegrees(int servoIn){
	int degreesOut = (servoIn - SERVO_OFFSET) * SERVO_CONV_SP;
	return(degreesOut);
}

int CHANNEL_REGISTER::halfDegrees2Servo(int degreesIn){
	int servoOut = ((1.0 * degreesIn) / SERVO_CONV) + SERVO_OFFSET;
	return(servoOut);
}

int CHANNEL_REGISTER::quarterDegrees2Servo(int degreesIn){
	int servoOut = ((1.0 * degreesIn) / SERVO_CONV_SP) + SERVO_OFFSET;
	return(servoOut);
}

void CHANNEL_REGISTER::setVariableSensitivity(int intensity){
	variableConv_ = intensity;
}

void CHANNEL_REGISTER::processVariable(int cycle){
	variableSelection_ = variableConv_ * (cycle - SERVO_OFFSET);
	if (variableSelection_ < 0.0) { variableSelection_ = 0.0; }
}