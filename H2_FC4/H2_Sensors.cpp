#include <Arduino.h>
#include <Wire.h>
#include "H2_Sensors.h"

//ADXL345
#define ADXL345_DEVICE (0x53)
#define ADXL345_TO_READ (6)
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_DATAX0 0x32

//BMP180/BMP085
#define BMP180_ADDR 0x77 // 7-bit address

#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6

#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4

//HMC5883L
#define compass_address 0x1E       // The I2C address of the Magnetometer
//#define compass_XY_excitation 1160 // The magnetic field excitation in X and Y direction during Self Test (Calibration)
//#define compass_Z_excitation 1080  // The magnetic field excitation in Z direction during Self Test (Calibration)
//#define compass_rad2degree 57.3


//#define compass_cal_x_offset 116   // Manually calculated offset in X direction
//#define compass_cal_y_offset 225   // Manually calculated offset in Y direction
//#define compass_cal_x_gain 1.1     // Stored Gain offset at room temperature
//#define compass_cal_y_gain 1.12    // Stored Gain offset at room temperature

//ITG3200
#define ITG3200_Address 0x68
#define gyroConversion (0.06957)

//L3G4200D
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define gyroConversion (0.0175)
#define L3G4200D_Address 105

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
