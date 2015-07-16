#include <Arduino.h>
#include <Wire.h>
#include "H2_BMP180.h"

#define BMP180_ADDR 0x77 // 7-bit address

#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6

#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4

char BMP180::begin(char MODE, double alpha)
// Initialize library for subsequent pressure measurements
{

	BARO_MODE_ = MODE;
	altAlpha_ = alpha;
	
	alt = 0;

	double c3, c4, b1;

	// Start up the Arduino's "wire" (I2C) library:

	Wire.begin();

	// The BMP180 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.

	// Retrieve calibration data from device:

	if (readInt(0xAA, AC1) &&
		readInt(0xAC, AC2) &&
		readInt(0xAE, AC3) &&
		readUInt(0xB0, AC4) &&
		readUInt(0xB2, AC5) &&
		readUInt(0xB4, AC6) &&
		readInt(0xB6, VB1) &&
		readInt(0xB8, VB2) &&
		readInt(0xBA, MB) &&
		readInt(0xBC, MC) &&
		readInt(0xBE, MD))
	{


		c3 = 160.0 * pow(2, -15) * AC3;
		c4 = pow(10, -3) * pow(2, -15) * AC4;
		b1 = pow(160, 2) * pow(2, -30) * VB1;
		c5 = (pow(2, -15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2, 11) / pow(160, 2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2, -13) * AC2;
		x2 = pow(160, 2) * pow(2, -25) * VB2;
		y0 = c4 * pow(2, 15);
		y1 = c4 * c3;
		y2 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2, -20);
		p2 = 3038.0 * 100.0 * pow(2, -36);

		updateTemperature();
		updatePressure();

		baseline_ = P;

		// Success!
		return(1);
	}
	else
	{
		// Error reading calibration data; bad component or connection?
		return(0);
	}
}

char BMP180::readInt(char address, int &value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (readBytes(data, 2))
	{
		value = (((int)data[0] << 8) | (int)data[1]);
		//if (*value & 0x8000) *value |= 0xFFFF0000; // sign extend if negative
		return(1);
	}
	value = 0;
	return(0);
}

char BMP180::readUInt(char address, unsigned int &value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (readBytes(data, 2))
	{
		value = (((unsigned int)data[0] << 8) | (unsigned int)data[1]);
		return(1);
	}
	value = 0;
	return(0);
}

char BMP180::readBytes(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
	char x;

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values[0]);
	_error = Wire.endTransmission();
	if (_error == 0)
	{
		Wire.requestFrom(BMP180_ADDR, length);
		while (Wire.available() != length); // wait until bytes are ready
		for (x = 0; x<length; x++)
		{
			values[x] = Wire.read();
		}
		return(1);
	}
	return(0);
}

char BMP180::writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
	char x;

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(values, length);
	_error = Wire.endTransmission();
	if (_error == 0)
		return(1);
	else
		return(0);
}

char BMP180::startTemperature()
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
{
	unsigned char data[2], result;

	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	result = writeBytes(data, 2);
	if (result) // good write?
		return(5); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}

char BMP180::getTemperature()
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
	unsigned char data[2];
	char result;
	double tu, a;

	data[0] = BMP180_REG_RESULT;

	result = readBytes(data, 2);
	if (result) // good read, calculate temperature
	{
		tu = (data[0] * 256.0) + data[1];

		//example from Bosch datasheet
		//tu = 27898;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
		//tu = 0x69EC;

		a = c5 * (tu - c6);
		T = a + (mc / (a + md));

		/*
		Serial.println();
		Serial.print("tu: "); Serial.println(tu);
		Serial.print("a: "); Serial.println(a);
		Serial.print("T: "); Serial.println(*T);
		*/
	}
	return(result);
}

char BMP180::startPressure()
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
	unsigned char data[2], result, delay;

	data[0] = BMP180_REG_CONTROL;

	switch (BARO_MODE_)
	{
	case 0:
		data[1] = BMP180_COMMAND_PRESSURE0;
		delay = 5;
		break;
	case 1:
		data[1] = BMP180_COMMAND_PRESSURE1;
		delay = 8;
		break;
	case 2:
		data[1] = BMP180_COMMAND_PRESSURE2;
		delay = 14;
		break;
	case 3:
		data[1] = BMP180_COMMAND_PRESSURE3;
		delay = 26;
		break;
	default:
		data[1] = BMP180_COMMAND_PRESSURE0;
		delay = 5;
		break;
	}
	result = writeBytes(data, 2);
	if (result) // good write?
		return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}

char BMP180::getPressure()
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
	unsigned char data[3];
	char result;
	double pu, s, x, y, z;

	data[0] = BMP180_REG_RESULT;

	result = readBytes(data, 3);
	if (result) // good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2] / 256.0);

		//example from Bosch datasheet
		//pu = 23843;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;	
		//pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);

		s = T - 25.0;
		x = (x2 * pow(s, 2)) + (x1 * s) + x0;
		y = (y2 * pow(s, 2)) + (y1 * s) + y0;
		z = (pu - x) / y;
		P = (p2 * pow(z, 2)) + (p1 * z) + p0;
	}
	return(result);
}

double BMP180::calculateAltitude()
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	a_ = 44330.0*(1 - pow(P / (1.0 * baseline_), 1 / 5.255));
	alt = altAlpha_ * alt + (1 - altAlpha_) * a_;
	return(44330.0*(1 - pow(P / (1.0 * baseline_), 1 / 5.255)));
}

void BMP180::updatePressure(){
	char status = startPressure();
	if (status != 0){
		delay(status);
		status = getPressure();
	}
}

void BMP180::updateTemperature(){
	char status = startTemperature();
	if (status != 0)
	{
		delay(status);
		status = getTemperature();
	}
}





