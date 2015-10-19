
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

void compli(class L3D4200D &gyro, class ADXL345 &accel, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
	//Complimentary Filter to Mix Gyro and Accelerometer Data
	gyro.update();
	accel.update();

	orient.rollGyro = gyro.x;
	orient.pitchGyro = gyro.y;
	orient.yawGyro = gyro.z;

	double cycle = (millis() - compliClockOld) * DIV_BY_MILL;

	double pitchAccel = atan2(-accel.x, accel.z)*RAD_TO_DEGREE*ACC_SCALAR + PITCH_OFFSET;
	//orient.pitch = compliAlpha * (orient.pitch + (gyro.y) * cycle) + (1 - compliAlpha) * pitchAccel;
	orient.pitch = compliAlpha * (orient.pitch + (gyro.y * cycle) - pitchAccel) + pitchAccel;

	double rollAccel = atan2(accel.y, accel.z)*RAD_TO_DEGREE*ACC_SCALAR + ROLL_OFFSET;
	//orient.roll = compliAlpha * (orient.roll + (gyro.x) * cycle) + (1 - compliAlpha) * rollAccel;
	orient.roll = compliAlpha * (orient.roll + (gyro.x * cycle) - rollAccel) + rollAccel;
}

void compli(class ITG3200 &gyro, class ADXL345 &accel, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
	//Complimentary Filter to Mix Gyro and Accelerometer Data
	gyro.update();
	accel.update();

	orient.rollGyro = gyro.x;
	orient.pitchGyro = gyro.y;
	orient.yawGyro = gyro.z;

	double cycle = (millis() - compliClockOld) * DIV_BY_MILL;

	double pitchAccel = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z))*RAD_TO_DEGREE*ACC_SCALAR + PITCH_OFFSET;
	//orient.pitch = compliAlpha * (orient.pitch + (gyro.y) * cycle) + (1 - compliAlpha) * pitchAccel;
	orient.pitch = compliAlpha * (orient.pitch + (gyro.y * cycle) - pitchAccel) + pitchAccel;

	double rollAccel = atan2(accel.y, sqrt(accel.x * accel.x + accel.z * accel.z))*RAD_TO_DEGREE*ACC_SCALAR + ROLL_OFFSET;
	//orient.roll = compliAlpha * (orient.roll + (gyro.x) * cycle) + (1 - compliAlpha) * rollAccel;
	orient.roll = compliAlpha * (orient.roll + (gyro.x * cycle) - rollAccel) + rollAccel;
}

void compli(class MPU6050 &mpu, struct ORIENT_STRUCT &orient, unsigned long &compliClockOld){
	//Complimentary Filter to Mix Gyro and Accelerometer Data
	mpu.readData();

	orient.rollGyro = mpu.gx;
	orient.pitchGyro = mpu.gy;
	orient.yawGyro = mpu.gz;

	double cycle = (millis() - compliClockOld) * DIV_BY_MILL;

        //double pitchAccel = atan2(mpu.ay, mpu.az)*RAD_TO_DEGREE*ACC_SCALAR + PITCH_OFFSET;
        double pitchAccel = atan2(mpu.ay, sqrt(mpu.ax * mpu.ax + mpu.az * mpu.az))*RAD_TO_DEGREE*ACC_SCALAR + PITCH_OFFSET;
	//orient.pitch = compliAlpha * (orient.pitch + (gyro.y) * cycle) + (1 - compliAlpha) * pitchAccel;
	orient.pitch = compliAlpha * (orient.pitch + (mpu.gx * cycle) - pitchAccel) + pitchAccel;

	//double rollAccel = atan2(-mpu.ax, mpu.az)*RAD_TO_DEGREE*ACC_SCALAR + ROLL_OFFSET;
        double rollAccel = atan2(-mpu.ax, sqrt(mpu.ay * mpu.ay + mpu.az * mpu.az))*RAD_TO_DEGREE*ACC_SCALAR + ROLL_OFFSET;
	//orient.roll = compliAlpha * (orient.roll + (gyro.x) * cycle) + (1 - compliAlpha) * rollAccel;
	orient.roll = compliAlpha * (orient.roll + (mpu.gy * cycle) - rollAccel) + rollAccel;
}
