#include <Arduino.h>
#include <Wire.h>
#include "H2_Registers.h"
#include "H2_ADXL345.h"

void MODE_REGISTER::switchModes(){
	switch (RC_CONTROL_MODE){
	case 0:
		CH1_ENABLE = 1;
		CH2_ENABLE = 1;
		CH3_ENABLE = 1;
		CH4_ENABLE = 1;
		GPS_CH1_ENABLE = 0;
		GPS_CH2_ENABLE = 0;
		GPS_NAV_ENABLE = 0;
		break;

	case 1:
		CH1_ENABLE = 1;
		CH2_ENABLE = 1;
		CH3_ENABLE = 0;
		CH4_ENABLE = 0;
		GPS_CH1_ENABLE = 0;
		GPS_CH2_ENABLE = 0;
		GPS_NAV_ENABLE = 1;
		break;

	case 2:
		CH1_ENABLE = 0;
		CH2_ENABLE = 0;
		CH3_ENABLE = 0;
		CH4_ENABLE = 0;
		GPS_CH1_ENABLE = 1;
		GPS_CH2_ENABLE = 1;
		GPS_NAV_ENABLE = 0;
		break;
	}
}

void MODE_REGISTER::init(int RCMODE){
	
	RC_CONTROL_MODE = RCMODE;
	switchModes();

}

void MODE2_REGISTER::switchModes(int RCMode){
	RC_CONTROL_MODE = RCMode;
	switch (RC_CONTROL_MODE){
	case 0:
		AUXI_EN = 1;

		CH1_AUTO_EN = 0;
		CH1_HOLD_EN = 0;
		CH1_RATE_EN = 0;

		CH2_AUTO_EN = 0;
		CH2_HOLD_EN = 0;
		CH2_RATE_EN = 0;

		CH3_AUTO_EN = 0;
		CH3_HOLD_EN = 0;
		CH3_RATE_EN = 0;

		CH4_AUTO_EN = 0;
		CH4_HOLD_EN = 0;
		CH4_RATE_EN = 0;

		break;

	case 1:
		AUXI_EN = 1;

		CH1_AUTO_EN = 0;
		CH1_HOLD_EN = 0;
		CH1_RATE_EN = 0;

		CH2_AUTO_EN = 0;
		CH2_HOLD_EN = 0;
		CH2_RATE_EN = 0;

		CH3_AUTO_EN = 0;
		CH3_HOLD_EN = 0;
		CH3_RATE_EN = 0;

		CH4_AUTO_EN = 0;
		CH4_HOLD_EN = 0;
		CH4_RATE_EN = 0;

		break;

	case 2:
		AUXI_EN = 1;

		CH1_AUTO_EN = 0;
		CH1_HOLD_EN = 0;
		CH1_RATE_EN = 0;

		CH2_AUTO_EN = 0;
		CH2_HOLD_EN = 0;
		CH2_RATE_EN = 0;

		CH3_AUTO_EN = 0;
		CH3_HOLD_EN = 0;
		CH3_RATE_EN = 0;

		CH4_AUTO_EN = 0;
		CH4_HOLD_EN = 0;
		CH4_RATE_EN = 0;

		break;
	}
}


void ORIENTATION_REGISTER::updateAlt(double newAlt){
	alt = newAlt;
}

void ORIENTATION_REGISTER::initAngles(class ADXL345 &acc, const double ROLL_OFFSET, const double PITCH_OFFSET){
	acc.update(); //Obtains Initial Angles; Quad must be motionless
	roll = atan2(acc.y, acc.z)*(180 / PI) + ROLL_OFFSET; //Accounts for Angle Differences
	pitch = atan2(-acc.x, acc.z)*(180 / PI) + PITCH_OFFSET;
}

void TARGET_REGISTER::incAlt(){
	//Elevation Change Smoothing -- Delays Target Altitude to Maximize I Gain
	if (intAlt < alt){ intAlt += 0.03; }
	if (intAlt > alt){ intAlt -= 0.03; }

}
