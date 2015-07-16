#ifndef H2_Sensors_h
#define H2_Sensors_h

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

class HMC5883L{
	public:
		//Quad_HMC5883L();
		void init(double xGainError, double yGainError, double zGainError, double xOffset, double yOffset, double zOffset);
		void update();
		void poll();

		double xScaled;
		double yScaled;
		double zScaled;

	private:
		double gainFactor_;
		double xGainError_, yGainError_, zGainError_;
		double xOffset_, yOffset_, zOffset_;
		int mX_, mY_, mZ_;
};

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
