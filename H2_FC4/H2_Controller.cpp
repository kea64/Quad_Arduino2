#include <Arduino.h>
#include "H2_HMC5883L.h"
#include "H2_ADXL345.h"
#include "H2_ITG3200.h"
#include "H2_L3D4200D.h"
#include "H2_BMP180.h"
#include "H2_TiltComp.h"
#include "H2_Orient.h"
#include "H2_Target.h"
#include "H2_PID.h"
#include "H2_Output.h"
#include "H2_Drone_Values.h"
#include "H2_Support.h"
#include "H2_Controller.h"
#include "H2_Mode.h"

extern volatile int channel1Cycle, channel2Cycle, channel3Cycle, channel4Cycle, channel5Cycle, channel6Cycle;

extern int count;

extern double channel6Var;

void updateController(struct PID_REGISTER &channels, struct TARGET_STRUCT target, struct ORIENT_STRUCT orient, struct OUTPUT_STRUCT &output, byte RC_CONTROL_MODE, unsigned long &controlClockOld){
	if ((millis() - controlClockOld) >= CONTROL_DELAY){
		double cycle = (millis() - controlClockOld) * DIV_BY_MILL;
		count += 1;

		channel6Var = intMap(channel6Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, 0, 2);
		
		channels.rrPID.updateGain(KPRR, channel6Var, KDRR);
		channels.prPID.updateGain(KPPR, channel6Var, KDPR);
		//channels.yrPID.updateGain(KPYR, channel6Var, KDYR);
		//channels.rrPID.updateGain(KPRR, channel6Var, KDRR);
		//channels.yrPID.updateGain(KPYR, channel6Var, KDYR);

		//channels.rsPID.updateGain(channel6Var, KIRS, KDRS);
		//hannels.psPID.updateGain(channel6Var, KIPS, KDPS);

		double rollChannel = intMap(channel1Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, -90, 90);
		double pitchChannel = intMap(channel2Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, 90, -90);
		double yawChannel = intMap(channel4Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, -180, 180);

		double errorLongitude;
		double errorLatitude;
		double errorRoll;
		double errorPitch;

		if (GPS_EN){
			errorLongitude = target.longitude - orient.longitude;
			errorLatitude = target.latitude - orient.latitude;
			errorRoll = (errorLongitude * cos(radians(orient.yaw)) + errorLatitude * sin(radians(orient.yaw)));
			errorPitch = -(errorLatitude * cos(radians(orient.yaw)) + errorLongitude * sin(radians(orient.yaw)));

			channels.arPID.calc(errorRoll, 0, cycle);
			channels.apPID.calc(errorPitch, 0, cycle);
		}

		switch (RC_CONTROL_MODE){
		case 0:
			if (AUXILIARY_EN){
				output.roll = channel1Cycle;
				output.pitch = channel2Cycle;
				output.throttle = channel3Cycle;
				output.yaw = channel4Cycle;
			}
			else {
				//channels.rollPID.calc(error, sensor, cycle);
				if (ACRO_EN && channel3Cycle >= THROTTLE_CUTOFF){
					channels.rrPID.calc(rollChannel - orient.rollGyro, orient.rollGyro, cycle);
					channels.prPID.calc(pitchChannel - orient.pitchGyro, orient.pitchGyro, cycle);
				}
				else if (channel3Cycle >= THROTTLE_CUTOFF) {
					channels.rsPID.calc(rollChannel - orient.roll, orient.roll, cycle);
					channels.psPID.calc(pitchChannel - orient.pitch, orient.pitch, cycle);

					//count = channels.psPID.getControl();

					channels.rrPID.calc(channels.rsPID.getControl() - orient.rollGyro, orient.rollGyro, cycle);
					channels.prPID.calc(channels.psPID.getControl() - orient.pitchGyro, orient.pitchGyro, cycle);
				}


				channels.yrPID.calc(yawChannel - orient.yawGyro, orient.yawGyro, cycle);

				output.roll = channels.rrPID.getControl();
				output.pitch = channels.prPID.getControl();
				output.throttle = channel3Cycle;
				output.yaw = channels.yrPID.getControl();
			}
			break;
		case 1:
			channels.rsPID.calc(channels.arPID.getControl() - orient.roll, orient.roll, cycle);
			channels.psPID.calc(channels.apPID.getControl() - orient.pitch, orient.pitch, cycle);
			channels.ysPID.calc(target.yaw - orient.yaw, orient.yaw, cycle);

			channels.rrPID.calc(channels.rsPID.getControl() - orient.rollGyro, orient.rollGyro, cycle);
			channels.prPID.calc(channels.psPID.getControl() - orient.pitchGyro, orient.pitchGyro, cycle);
			channels.yrPID.calc(channels.ysPID.getControl() - orient.yawGyro, orient.yawGyro, cycle);

			channels.atPID.calc(target.alt - orient.alt, orient.alt, cycle);

			if (AUXILIARY_EN){
				//ADD QUAD GPS CODE HERE- NEED TO PASS GPSROLL TO MAP TO OUTPUT
			}
			else {
				output.roll = channels.rrPID.getControl();
				output.pitch = channels.prPID.getControl();
				output.throttle = channels.atPID.getControl();
				output.yaw = channels.yrPID.getControl();
			}
			break;
		case 2:
			channels.rsPID.calc(channels.arPID.getControl() - orient.roll, orient.roll, cycle);
			channels.psPID.calc(channels.apPID.getControl() - orient.pitch, orient.pitch, cycle);
			channels.ysPID.calc(target.yaw - orient.yaw, orient.yaw, cycle);

			channels.rrPID.calc(channels.rsPID.getControl() - orient.rollGyro, orient.rollGyro, cycle);
			channels.prPID.calc(channels.psPID.getControl() - orient.pitchGyro, orient.pitchGyro, cycle);
			channels.yrPID.calc(channels.ysPID.getControl() - orient.yawGyro, orient.yawGyro, cycle);

			channels.atPID.calc(target.alt - orient.alt, orient.alt, cycle);

			if (AUXILIARY_EN){
				//ADD QUAD GPS CODE HERE- NEED TO PASS GPSROLL TO MAP TO OUTPUT
			}
			else {
				output.roll = channels.rrPID.getControl();
				output.pitch = channels.prPID.getControl();
				output.throttle = channels.atPID.getControl();
				output.yaw = channels.yrPID.getControl();
			}
			break;
		}

		processMotors(output);

		controlClockOld = millis();
	}
}
