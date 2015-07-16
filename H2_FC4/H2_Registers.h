#ifndef H2_Registers_h
#define H2_Registers_h
#include "H2_Registers.h"
#include "Arduino.h"

class MODE_REGISTER{
public:
	void switchModes();
	
	void init(int RCMODE);

	int RC_CONTROL_MODE;
	bool CH1_ENABLE;
	bool CH2_ENABLE;
	bool CH3_ENABLE;
	bool CH4_ENABLE;
	bool GPS_CH1_ENABLE;
	bool GPS_CH2_ENABLE;
	bool GPS_NAV_ENABLE;

private:

};

class MODE2_REGISTER{
public:
	void switchModes(int RCMode);

	int RC_CONTROL_MODE;

	bool AUXI_EN;
	
	bool CH1_AUTO_EN;
	bool CH1_HOLD_EN;
	bool CH1_RATE_EN;

	bool CH2_AUTO_EN;
	bool CH2_HOLD_EN;
	bool CH2_RATE_EN;

	bool CH3_AUTO_EN;
	bool CH3_HOLD_EN;
	bool CH3_RATE_EN;

	bool CH4_AUTO_EN;
	bool CH4_HOLD_EN;
	bool CH4_RATE_EN;

private:

};

class ORIENTATION_REGISTER{
public:
	void initAngles(class ADXL345 &acc, const double ROLL_OFFSET, const double PITCH_OFFSET);
	void updateAlt(double newAlt);

	double roll, pitch, yaw, alt, latitude, longitude, throttle;

private:


};

class TARGET_REGISTER{
public:

	double roll, pitch, yaw, alt, intAlt, latitude, longitude, throttle, holdAlt, holdYaw, holdLatitude, holdLongitude;
	int waypointCounter;

	void incAlt();
private:


};

class PID_SINGLE{
public:
	double iTerm, lastPos;
	int control;
	int initial;

private:

};

class PID_REGISTER{
public:
	PID_SINGLE roll;
	PID_SINGLE pitch;
	PID_SINGLE throttle;
	PID_SINGLE yaw;
	PID_SINGLE GPSRoll;
	PID_SINGLE GPSPitch;
private:

};


#endif