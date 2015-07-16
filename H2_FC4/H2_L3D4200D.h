#ifndef H2_L3D4200D_h
#define H2_L3D4200D_h
#include "H2_L3D4200D.h"
#include "Arduino.h"

class L3D4200D{
public:
	void init(double gyroAlpha);
	void calibrate();
	void update();

	double x, y, z;

private:
	void writeRegister(int deviceAddress, byte address, byte val);
	int readRegister(int deviceAddress, byte address);

	double xOffset_, yOffset_, zOffset_;
	double gyroAlpha_;

	bool INIT_EN_;
};

#endif