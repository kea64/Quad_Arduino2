#include <Arduino.h>
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

void checkCompli(class L3D4200D &gyro, class ADXL345 &acc, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
	//Main Sensor Reading and Motor Control
	if ((millis() - compliClockOld) >= COMPLI_DELAY){
		compli(gyro, acc, orient, compliClockOld); //Complimentary Filter
		calcYaw(mag, orient); //Tilt Compensated Compass Code

		compliClockOld = millis();
	}
}

void checkCompli(class ITG3200 &gyro, class ADXL345 &acc, class HMC5883L &mag, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
	//Main Sensor Reading and Motor Control
	if ((millis() - compliClockOld) >= COMPLI_DELAY){
		compli(gyro, acc, orient, compliClockOld); //Complimentary Filter
		calcYaw(mag, orient); //Tilt Compensated Compass Code

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

void checkTemp(class BMP180 &baro, unsigned long &tempClockOld){
	if ((millis() - tempClockOld) >= TEMP_DELAY){
		baro.updateTemperature();
		tempClockOld = millis();
	}
}
