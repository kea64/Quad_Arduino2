

#include <Arduino.h>
#include <Wire.h>
#include "H2_HMC5883L.h"

// Definitions ----------------------------------------------------------------
#define compass_address 0x1E       // The I2C address of the Magnetometer
//#define compass_XY_excitation 1160 // The magnetic field excitation in X and Y direction during Self Test (Calibration)
//#define compass_Z_excitation 1080  // The magnetic field excitation in Z direction during Self Test (Calibration)
//#define compass_rad2degree 57.3


//#define compass_cal_x_offset 116   // Manually calculated offset in X direction
//#define compass_cal_y_offset 225   // Manually calculated offset in Y direction
//#define compass_cal_x_gain 1.1     // Stored Gain offset at room temperature
//#define compass_cal_y_gain 1.12    // Stored Gain offset at room temperature


void HMC5883L::init(double xGainError, double yGainError, double zGainError, double xOffset, double yOffset, double zOffset){
	Wire.beginTransmission(compass_address);
	Wire.write(0x01);

	byte gain_reg = 0b01000000;
	gainFactor_ = 1.22;
	
	Wire.write(gain_reg); // bit configuration = g2 g1 g0 0 0 0 0 0, g2 g1 g0 = 0 0 1 for 1.3 guass and 0 1 0 for 1.9 Guass
	Wire.write(0b00000011);  // Putting the Magnetometer in idle
	// Writing the register value 0000 0000 for continous mode
	// Writing the register value 0000 0001 for single
	// Writing the register value 0000 0011 for Idel
	Wire.endTransmission();

	xGainError_ = xGainError;
	yGainError_ = yGainError;
	zGainError_ = zGainError;

	xOffset_ = xOffset;
	yOffset_ = yOffset;
	zOffset_ = zOffset;
}


void HMC5883L::update(){

	poll();

	xScaled = mX_ * gainFactor_ * xGainError_ + xOffset_;
	yScaled = mY_ * gainFactor_ * yGainError_ + yOffset_;
	zScaled = mZ_ * gainFactor_ * zGainError_ + zOffset_;

}

void HMC5883L::poll(){

	Wire.beginTransmission(compass_address);
	Wire.write(0x02);
	Wire.write(0b10000001);
	// Writing the register value 0000 0000 for continous mode
	// Writing the register value 0000 0001 for single
	Wire.endTransmission();
	Wire.requestFrom(compass_address, 6);

	if (6 <= Wire.available()){

		mX_ = Wire.read() << 8 | Wire.read();
		mZ_ = Wire.read() << 8 | Wire.read(); //Yeah, that's intuitive
		mY_ = Wire.read() << 8 | Wire.read();

	}


}
