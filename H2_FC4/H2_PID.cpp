#include <Arduino.h>
#include "H2_PID.h"
#include "H2_Drone_Values.h"

#define iTermDelay 2000

H2_PID::H2_PID(){


}

H2_PID::H2_PID(double kp, double ki, double kd, int maximum, int minimum, int iMax){
	iError_ = 0;
	dError_ = 0;
	dLast_ = 0;

	control_ = 0;

	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
	maximum_ = maximum;
	minimum_ = minimum;
	iMax_ = iMax;
	tLast_ = millis();
}

void H2_PID::calc(double error, double sensor, double cycle){
	error_ = error;
	sensor_ = sensor;
	cycle_ = cycle;

	//Reset iTerm after period of rest. Prevents rough resets.
	if (millis() - tLast_ > iTermDelay){
		setIntegral(0);
	}

	calcPError();
	calcIError();
	calcDError();

	control_ = int(pError_ + iError_ - dError_);

	if (control_ > maximum_){
		control_ = maximum_;
	}
	else if (control_ < minimum_){
		control_ = minimum_;
	}

	tLast_ = millis();
}

void H2_PID::updateDefaults(double kp, double ki, double kd, double iTerm, int maximum, int minimum, int iMax){
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
	control_ = 0;
	dLast_ = 0;
	sensor_ = 0;
	iError_ = iTerm;
	maximum_ = maximum;
	minimum_ = minimum;
	iMax_ = iMax;
}

void H2_PID::updateGain(double kp, double ki, double kd){
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}

void H2_PID::updateBounds(int maximum, int minimum, int iMax){
	maximum_ = maximum;
	minimum_ = minimum;
	iMax_ = iMax;
}

void H2_PID::setIntegral(double newITerm){
	iError_ = newITerm;
}

void H2_PID::calcPError(){
	pError_ = error_ * kp_;
}

void H2_PID::calcIError(){
	iError_ += (error_ * ki_ * cycle_);

	if (iError_ > (iMax_)){
		iError_ = (iMax_);
	}
	else if (iError_ < (-iMax_)){
		iError_ = (-iMax_);
	}
	
}

void H2_PID::calcDError(){
	dError_ = (kd_ * (sensor_ - dLast_)) / cycle_;
	//double dAlpha = 0.5;
	dLast_ = dAlpha * dLast_ + (1 - dAlpha) * sensor_;
	//dLast_ = sensor_;
}

int H2_PID::getControl(){
	return(control_);
}