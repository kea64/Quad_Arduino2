#include <Arduino.h>
#include <TinyGPS++.h>
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

extern double waypoint[];

extern TinyGPSPlus gps;

void updateMode(struct PID_REGISTER &channels, struct TARGET_STRUCT &target, struct ORIENT_STRUCT &orient, byte &RC_CONTROL_MODE, unsigned long &modeClockOld){
	if (millis() - modeClockOld > MODE_DELAY){
		double rollChannel = intMap(channel1Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, -90, 90);
		double pitchChannel = intMap(channel2Cycle, SERVO_MINIMUM, SERVO_MAXIMUM, -90, 90);

		if (channel5Cycle < 1300){
			if (RC_CONTROL_MODE != 1){
				target.alt = waypoint[target.waypointCounter + 2];
				channels.atPID.setIntegral(channel3Cycle);
				//channels.autoRollPID.updateBounds(channel1Cycle + GPS_ROLL_MAXIMUM, channel1Cycle - GPS_ROLL_MAXIMUM);
				//channels.autoPitchPID.updateBounds(channel2Cycle + GPS_PITCH_MAXIMUM, channel2Cycle - GPS_PITCH_MAXIMUM);
				channels.arPID.setIntegral(rollChannel);
				channels.apPID.setIntegral(pitchChannel);
			}

			if (GPS_EN && orient.GPS_LOCK){
				target.yaw = int(TinyGPSPlus::courseTo(orient.latitude, orient.longitude, waypoint[target.waypointCounter], waypoint[target.waypointCounter + 1]));
				target.latitude = waypoint[target.waypointCounter];
				target.longitude = waypoint[target.waypointCounter + 1];
			}
			else {
				target.yaw = orient.yaw;
			}

			RC_CONTROL_MODE = 1;

		}
		else if (channel5Cycle >= 1300 && channel5Cycle <= 1700){
			if (RC_CONTROL_MODE != 0){

			}

			RC_CONTROL_MODE = 0;

		}
		else if (channel5Cycle > 1700){
			if (RC_CONTROL_MODE != 2){
				target.alt = orient.alt;
				channels.atPID.setIntegral(channel3Cycle);
				//channels.autoRollPID.updateBounds(channel1Cycle + GPS_ROLL_MAXIMUM, channel1Cycle - GPS_ROLL_MAXIMUM);
				//channels.autoPitchPID.updateBounds(channel2Cycle + GPS_PITCH_MAXIMUM, channel2Cycle - GPS_PITCH_MAXIMUM);
				channels.arPID.setIntegral(rollChannel);
				channels.apPID.setIntegral(pitchChannel);
				target.latitude = orient.latitude;
				target.longitude = orient.longitude;
				target.yaw = orient.yaw;
			}

			RC_CONTROL_MODE = 2;

		}

		modeClockOld = millis();
	}
}
