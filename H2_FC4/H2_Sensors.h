#ifndef H2_Sensors_h
#define H2_Sensors_h

#include "Arduino.h"

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

void i2c_writeReg(byte addr, byte reg, byte val);

class MPU6050{
  public:
    MPU6050();
    void init(int gyroGain, int accelGain, int DLPF);
    void readData();
    void resetWake();
    void setDLPF(int BW);
    void setGains(int gyro, int accel);
    void offsetCal();
    
    float temp;
    float ax, ay, az;
    float gx, gy, gz;
    
    float axr, ayr, azr;
    float gxr, gyr, gzr;
    
  private: 
    int accel_x_OC;
    int accel_y_OC;
    int accel_z_OC;
    int gyro_x_OC;
    int gyro_y_OC;
    int gyro_z_OC;
    
    float accel_scale_fact;
    float gyro_scale_fact;
    
    
};

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

class MS5611
{
    public:

	bool begin(double altAlpha, double altVelAlpha, ms5611_osr_t osr = MS5611_HIGH_RES);
	uint32_t readRawTemperature(void);
	uint32_t readRawPressure(void);
	double readTemperature(bool compensation = false);
	int32_t readPressure(bool compensation = false);
	double getAltitude(double pressure, double seaLevelPressure = 101325);
	double getSeaLevel(double pressure, double altitude);
	void setOversampling(ms5611_osr_t osr);
	ms5611_osr_t getOversampling(void);
        void update();
        void resetReference();
        
        double rawAlt;
        double alt;
        double altVelocity;

    private:

	uint16_t fc[6];
	uint8_t ct;
	uint8_t uosr;
	int32_t TEMP2;
	int64_t OFF2, SENS2;

	void reset(void);
	void readPROM(void);

	uint16_t readRegister16(uint8_t reg);
	uint32_t readRegister24(uint8_t reg);

        double referencePressure_;
        double altAlpha_;
        double altVelAlpha_;
        double lastAlt_;
        unsigned long lastTime_;
};

#endif
