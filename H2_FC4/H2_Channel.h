#ifndef H2_Channel_h
#define H2_Channel_h
#include "H2_Channel.h"
#include "Arduino.h"

class CHANNEL{
public:
	double iTerm;
	double lastPos;
	int initial;
	int control;

private:

};

class CHANNEL_REGISTER{
public:
	int servo2HalfDegrees(int servoIn);
	int servo2QuarterDegrees(int servoIn);
	int halfDegrees2Servo(int degreesIn);
	int quarterDegrees2Servo(int degreesIn);
	void setVariableSensitivity(int intensity);
	void processVariable(int cycle);

	CHANNEL ROLL;
	CHANNEL PITCH;
	CHANNEL THROTTLE;
	CHANNEL YAW;
	CHANNEL CONTROL;
	CHANNEL VARIABLE;

	CHANNEL AUTO_ROLL;
	CHANNEL AUTO_PITCH;
	CHANNEL AUTO_THROTTLE;
	CHANNEL AUTO_YAW;

private:
	int variableConv_;

	double variableSelection_;
};



#endif