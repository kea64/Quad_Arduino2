#include <Arduino.h>
#include <Wire.h>
#include "H2_L3D4200D.h"

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define gyroConversion (0.0175)
#define L3G4200D_Address 105

void L3D4200D::init(double gyroAlpha){

	gyroAlpha_ = gyroAlpha;

	x = 0;
	y = 0;
	z = 0;

	writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

	// If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
	writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

	// Configure CTRL_REG3 to generate data ready interrupt on INT2
	// No interrupts used on INT1, if you'd like to configure INT1
	// or INT2 otherwise, consult the datasheet:
	writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

	// CTRL_REG4 controls the full-scale range, among other things:

	
	writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);

	// CTRL_REG5 controls high-pass filtering of outputs, use it
	// if you'd like:
	writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);

}

void L3D4200D::calibrate(){
	INIT_EN_ = 1;
	
	do {
		int tmpx = 0;
		int tmpy = 0;
		int tmpz = 0;

		xOffset_ = 0;
		yOffset_ = 0;
		zOffset_ = 0;

		for (char i = 0; i < 25; i++)
		{
			delay(10);
			update();
			tmpx += x;
			tmpy += y;
			tmpz += z;
		}
		xOffset_ = tmpx / 25.0;
		yOffset_ = tmpy / 25.0;
		zOffset_ = tmpz / 25.0;

		update();

	} while (x - xOffset_ > 1.75 || x - xOffset_ < -1.75);

	INIT_EN_ = 0;
	
}

void L3D4200D::update(){
	byte xMSB = readRegister(L3G4200D_Address, 0x29);
	byte xLSB = readRegister(L3G4200D_Address, 0x28);
	int gX = ((xMSB << 8) | xLSB);
	if (INIT_EN_){
		x = gX * gyroConversion;
	}
	else {
		x = gyroAlpha_ * (gX * gyroConversion - xOffset_) + (1 - gyroAlpha_) * x;
	}

	byte yMSB = readRegister(L3G4200D_Address, 0x2B);
	byte yLSB = readRegister(L3G4200D_Address, 0x2A);
	int gY = ((yMSB << 8) | yLSB);
	if (INIT_EN_){
		y = gY * gyroConversion;
	}
	else {
		y = gyroAlpha_ * (gY * gyroConversion - yOffset_) + (1 - gyroAlpha_) * y;
	}

	byte zMSB = readRegister(L3G4200D_Address, 0x2D);
	byte zLSB = readRegister(L3G4200D_Address, 0x2C);
	int gZ = ((zMSB << 8) | zLSB);
	if (INIT_EN_){
		z = gZ * gyroConversion;
	}
	else {
		z = gyroAlpha_ * (gZ * gyroConversion - zOffset_) + (1 - gyroAlpha_) * z;
	}
}

void L3D4200D::writeRegister(int deviceAddress, byte address, byte val) {
	Wire.beginTransmission(deviceAddress); // start transmission to device 
	Wire.write(address);       // send register address
	Wire.write(val);         // send value to write
	Wire.endTransmission();     // end transmission
}

int L3D4200D::readRegister(int deviceAddress, byte address){

	int v;
	Wire.beginTransmission(deviceAddress);
	Wire.write(address); // register to read
	Wire.endTransmission();

	Wire.requestFrom(deviceAddress, 1); // read a byte

	while (!Wire.available()) {
		// waiting
	}

	v = Wire.read();
	return v;
}