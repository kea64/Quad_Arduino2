#ifndef H2_ITG3200_h
#define H2_ITG3200_h
#include "H2_ITG3200.h"
#include "Arduino.h"

class ITG3200{
public:
	void init(double gyroAlpha);
	void calibrate();
	void update();

	double x, y, z;

private:
	int xOffset_, yOffset_, zOffset_;

	double gyroAlpha_;

	bool INIT_EN_;
};

#endif