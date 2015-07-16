#ifndef H2_BMP180_h
#define H2_BMP180_h
#include "H2_BMP180.h"
#include "Arduino.h"

class BMP180{
public:
	char begin(char MODE, double alpha);
	char startTemperature();
	char getTemperature();
	char startPressure();
	char getPressure();
	double calculateAltitude();
	void updatePressure();
	void updateTemperature();

	double P, P0, T, alt;

private:
	char readInt(char address, int &value);
	char readUInt(char address, unsigned int &value);
	char readBytes(unsigned char *values, char length);
	char writeBytes(unsigned char *values, char length);

	int AC1, AC2, AC3, VB1, VB2, MB, MC, MD;
	unsigned int AC4, AC5, AC6;
	double c5, c6, mc, md, x0, x1, x2, y0, y1, y2, p0, p1, p2;
	char _error;
	double baseline_, a_;
	double altAlpha_;
	char BARO_MODE_;
};

#endif