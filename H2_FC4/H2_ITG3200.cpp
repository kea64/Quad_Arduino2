#include <Arduino.h>
#include <Wire.h>
#include "H2_ITG3200.h"

#define ITG3200_Address 0x68
#define gyroConversion (0.06957)

void ITG3200::init(double gyroAlpha) {

	gyroAlpha_ = gyroAlpha;

	x = 0;
	y = 0;
	z = 0;

	Wire.beginTransmission(ITG3200_Address);
	Wire.write(0x3E);
	Wire.write(0x00);
	Wire.endTransmission();

	Wire.beginTransmission(ITG3200_Address);
	Wire.write(0x15);
	Wire.write(0x07);
	Wire.endTransmission();

	Wire.beginTransmission(ITG3200_Address);
	Wire.write(0x16);
	Wire.write(0x1E);   // +/- 2000 dgrs/sec, 1KHz, 1E, 19
	Wire.endTransmission();

	Wire.beginTransmission(ITG3200_Address);
	Wire.write(0x17);
	Wire.write(0x00);
	Wire.endTransmission();
}

void ITG3200::calibrate(){
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

void ITG3200::update() {
	Wire.beginTransmission(ITG3200_Address);
	Wire.write(0x1B);
	Wire.endTransmission();

	Wire.beginTransmission(ITG3200_Address);
	Wire.requestFrom(ITG3200_Address, 8);    // request 8 bytes from ITG3200

	int i = 0;
	byte buff[8];
	while (Wire.available())
	{
		buff[i] = Wire.read();
		i++;
	}
	Wire.endTransmission();

	//int gX = ((buff[4] << 8) | buff[5]);
	int gX = ((buff[2] << 8) | buff[3]);
	if (INIT_EN_){
		x = gX * gyroConversion;
	}
	else {
		x = gyroAlpha_ * (gX * gyroConversion - xOffset_) + (1 - gyroAlpha_) * x;
	}

	//int gY = ((buff[2] << 8) | buff[3]);
	int gY = ((buff[4] << 8) | buff[5]);
	if (INIT_EN_){
		y = gY * gyroConversion;
	}
	else {
		y = gyroAlpha_ * (gY * gyroConversion - yOffset_) + (1 - gyroAlpha_) * y;
	}

	int gZ = ((buff[6] << 8) | buff[7]);
	if (INIT_EN_){
		z = gZ * gyroConversion;
	}
	else {
		z = gyroAlpha_ * (gZ * gyroConversion - zOffset_) + (1 - gyroAlpha_) * z;
	}

}