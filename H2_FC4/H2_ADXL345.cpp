#include <Arduino.h>
#include <Wire.h>
#include "H2_ADXL345.h"

#define ADXL345_DEVICE (0x53)
#define ADXL345_TO_READ (6)
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_DATAX0 0x32

void ADXL345::init(double accAlpha) {
	accAlpha_ = accAlpha;

	x = 0;
	y = 0;
	z = 0;

	Wire.begin();        // join i2c bus (address optional for master)
	//Turning on the ADXL345
	writeTo(ADXL345_POWER_CTL, 0);
	writeTo(ADXL345_POWER_CTL, 16);
	writeTo(ADXL345_POWER_CTL, 8);
}

void ADXL345::update() {
	const double accConv = 0.0039;
	readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff); //read the acceleration data from the ADXL345

	// each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
	// thus we are converting both bytes in to one int
	aX_ = (((int)_buff[1]) << 8) | _buff[0];
	aY_ = (((int)_buff[3]) << 8) | _buff[2];
	aZ_ = (((int)_buff[5]) << 8) | _buff[4];

	x = (accAlpha_ * aX_ * accConv) + (1 - accAlpha_) * x;
	y = (accAlpha_ * aY_ * accConv) + (1 - accAlpha_) * y;
	z = (accAlpha_ * aZ_ * accConv) + (1 - accAlpha_) * z;
}

void ADXL345::readFrom(byte address, int num, byte _buff[]) {
	Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device 
	Wire.write(address);             // sends address to read from
	Wire.endTransmission();         // end transmission

	Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device
	Wire.requestFrom(ADXL345_DEVICE, num);    // request 6 bytes from device

	int i = 0;
	while (Wire.available())         // device may send less than requested (abnormal)
	{
		_buff[i] = Wire.read();    // receive a byte
		i++;
	}
	
	Wire.endTransmission();         // end transmission
}

void ADXL345::writeTo(byte address, byte val) {
	Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device 
	Wire.write(address);             // send register address
	Wire.write(val);                 // send value to write
	Wire.endTransmission();         // end transmission
}