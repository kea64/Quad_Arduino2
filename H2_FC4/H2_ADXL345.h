#ifndef H2_ADXL345_h
#define H2_ADXL345_h
#include "H2_ADXL345.h"
#include "Arduino.h"

class ADXL345{
public:
	void init(double accAlpha);
	void update();
	void readFrom(byte address, int num, byte _buff[]);
	void writeTo(byte address, byte val);

	double x, y, z;
private:
	byte _buff[6];
	int aX_, aY_, aZ_;
	double accAlpha_;
};

#endif