#include <Arduino.h>
#include <EEPROM.h>
#include "H2_Sensors.h"
#include "H2_TiltComp.h"
#include "H2_Orient.h"
#include "H2_Target.h"
#include "H2_PID.h"
#include "H2_Output.h"
#include "H2_Drone_Values.h"
#include "H2_Support.h"
#include "H2_Controller.h"
#include "H2_Mode.h"
#include "H2_Filters.h"
#include "H2_Check_Timing.h"
#include "H2_EEPROM.h"

extern int compli_count;

void checkCompli(class L3D4200D &gyro, class ADXL345 &acc, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
	//Main Sensor Reading and Motor Control
	if ((millis() - compliClockOld) >= COMPLI_DELAY){
                compli_count += 1; //Complimentary Clock Cycle Counter
		compli(gyro, acc, orient, compliClockOld); //Complimentary Filter
		calcYaw(mag, orient); //Tilt Compensated Compass Code

		compliClockOld = millis();
	}
}

void checkCompli(class ITG3200 &gyro, class ADXL345 &acc, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
	//Main Sensor Reading and Motor Control
	if ((millis() - compliClockOld) >= COMPLI_DELAY){
                compli_count += 1; //Complimentary Clock Cycle Counter
		compli(gyro, acc, orient, compliClockOld); //Complimentary Filter
		calcYaw(mag, orient); //Tilt Compensated Compass Code

		compliClockOld = millis();
	}
}

void checkCompli(class MPU6050 &mpu, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
	//Main Sensor Reading and Motor Control
	if ((millis() - compliClockOld) >= COMPLI_DELAY){
                compli_count += 1; //Complimentary Clock Cycle Counter
		compli(mpu, orient, compliClockOld); //Complimentary Filter
		//calcYaw(mag, orient); //Tilt Compensated Compass Code

		compliClockOld = millis();
	}
}

void checkBaro(class BMP180 &baro, unsigned long &baroClockOld, struct ORIENT_STRUCT &orient){
	if ((millis() - baroClockOld) >= BARO_DELAY){
		baro.updatePressure();
		baro.calculateAltitude();
		baroClockOld = millis();
	}
}

void checkBaro(class MS5611 &baro, unsigned long &baroClockOld, struct ORIENT_STRUCT &orient){
	if ((millis() - baroClockOld) >= BARO_DELAY){
		baro.update();
		baroClockOld = millis();
	}
}

void checkTemp(class BMP180 &baro, unsigned long &tempClockOld){
	if ((millis() - tempClockOld) >= TEMP_DELAY){
		baro.updateTemperature();
		tempClockOld = millis();
	}
}

void checkAccCalib(class MPU6050 &mpu){
  if (EEPROM.read(ACCEL_CALIB_SCHEDULE_) == 1){
    mpu.accelCalib();
    
    EEPROM.put(ACCEL_CALIB_SCHEDULE_, 0);
  }
  
}

void checkAccCalib(class ADXL345 &accel){
  if (EEPROM.read(ACCEL_CALIB_SCHEDULE_) == 1){
    accel.accelCalib();
    
    EEPROM.put(ACCEL_CALIB_SCHEDULE_, 0);
  }
}

void checkMagCalib(class HMC5883L &mag){
  if (EEPROM.read(MAG_CALIB_SCHEDULE_) == 1){
    mag.calibrate();
    
    EEPROM.put(MAG_CALIB_SCHEDULE_, 0);
  }
}
