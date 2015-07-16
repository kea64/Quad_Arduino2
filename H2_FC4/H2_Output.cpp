#include <Arduino.h>
#include <Servo.h>
#include "H2_Output.h"
#include "H2_Drone_Values.h"
#include "H2_Support.h"

extern Servo output1;
extern Servo output2;
extern Servo output3;
extern Servo output4;

extern volatile int channel3Cycle, channel4Cycle;


void processMotors(struct OUTPUT_STRUCT output){

	if (QUAD_EN){
		int op1 = output.throttle + output.roll - output.pitch + output.yaw;
		int op2 = output.throttle - output.roll - output.pitch - output.yaw;
		int op3 = output.throttle - output.roll + output.pitch + output.yaw;
		int op4 = output.throttle + output.roll + output.pitch - output.yaw;

		withinBounds(op1, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
		withinBounds(op2, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
		withinBounds(op3, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
		withinBounds(op4, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);

		if (output.throttle > THROTTLE_CUTOFF){
			output1.writeMicroseconds(op1);
			output2.writeMicroseconds(op2);
			output3.writeMicroseconds(op3);
			output4.writeMicroseconds(op4);
		}
		else {
			output1.writeMicroseconds(THROTTLE_MINIMUM);
			output2.writeMicroseconds(THROTTLE_MINIMUM);
			output3.writeMicroseconds(THROTTLE_MINIMUM);
			output4.writeMicroseconds(THROTTLE_MINIMUM);
		}
	}
	else if (TRI_EN){

		int op1 = output.throttle + output.roll - 0.8 * output.pitch;
		int op2 = output.throttle - output.roll - 0.8 * output.pitch;
		int op3 = output.throttle + output.pitch;
		int op4 = SERVO_MIDPOINT + output.yaw + 30;

		double tailConv = intMap(op4, TAIL_SERVO_MIN, TAIL_SERVO_MAX, TAIL_SERVO_MIN_DEGREE, TAIL_SERVO_MAX_DEGREE) - TAIL_SERVO_OFFSET;

		op3 = ((op3 - SERVO_MINIMUM) / sin(radians(tailConv))) + SERVO_MINIMUM;

		withinBounds(op1, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
		withinBounds(op2, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
		withinBounds(op3, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
		withinBounds(op4, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);

		if (output.throttle > THROTTLE_CUTOFF){
			output1.writeMicroseconds(op1);
			output2.writeMicroseconds(op2);
			output3.writeMicroseconds(op3);
			output4.writeMicroseconds(op4);
		}
		else {
			output1.writeMicroseconds(THROTTLE_MINIMUM);
			output2.writeMicroseconds(THROTTLE_MINIMUM);
			output3.writeMicroseconds(THROTTLE_MINIMUM);
			if (TRI_EN){
				output4.writeMicroseconds(SERVO_MIDPOINT);
			}
			else {
				output4.writeMicroseconds(THROTTLE_MINIMUM);
			}
			
		}
	}
}

void checkArming(bool &MOTOR_EN){
	if (millis() > INITIAL_ARM_DELAY){
		if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle <= ARM_ENGAGE_THRESHOLD && channel3Cycle > 100 && channel4Cycle > 100){
			MOTOR_EN = 1;
			digitalWrite(13, HIGH);

		}
		else if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle >= ARM_DISENGAGE_THRESHOLD){
			MOTOR_EN = 0;
			digitalWrite(13, LOW);
		}
	}
	//Perhaps Add PID DISABLE Code Here
}

