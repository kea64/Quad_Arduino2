#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
#include "H2_Sensors.h"
#include "H2_Output.h"
#include "H2_Drone_Values.h"
#include "H2_Support.h"
#include "H2_EEPROM.h"

#if defined(SPYDER_EN) || defined(TRI_EN) || defined(HEXACOPTER_EN) || defined(OCTOCOPTER_EN)
  extern Servo output1;
  extern Servo output2;
  extern Servo output3;
  extern Servo output4;
#endif
#if defined(HEXACOPTER_EN) || defined(OCTOCOPTER_EN)
  extern Servo output5;
  extern Servo output6;
#endif
#if defined(OCTOCOPTER_EN)
  extern Servo output7;
  extern Servo output8;
#endif
#if defined(ROVER_EN)
  extern Servo output2;
  extern Servo output4;
#endif

extern volatile int channel1Cycle, channel2Cycle, channel3Cycle, channel4Cycle;


void processMotors(struct OUTPUT_STRUCT output){

	#if defined(SPYDER_EN)
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
		} else {
			output1.writeMicroseconds(THROTTLE_MINIMUM);
			output2.writeMicroseconds(THROTTLE_MINIMUM);
			output3.writeMicroseconds(THROTTLE_MINIMUM);
			output4.writeMicroseconds(THROTTLE_MINIMUM);
		}
	#elif defined(TRI_EN)

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
		} else {
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

   #elif defined(ROVER_EN)
      int op2 = output.throttle;
      int op4 = output.yaw;

      withinBounds(op2, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);
      withinBounds(op4, THROTTLE_MAXIMUM, THROTTLE_MINIMUM);

      output2.writeMicroseconds(op2);
      output4.writeMicroseconds(op4);
   
   #endif
}

void checkArming(bool &MOTOR_EN, int &rollOverDelay, class MS5611 &baro){
	if (millis() > INITIAL_ARM_DELAY){
		if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle <= ARM_ENGAGE_THRESHOLD && channel3Cycle > 100 && channel4Cycle > 100){
			if(MOTOR_EN != 1){
                          #if defined(M5611_EN)
                              baro.resetReference();
                          #endif
                          rollOverDelay++;
                        }
                        if(rollOverDelay >= ARM_DELAY){
                          if(EEPROM.read(ACCEL_CHECK_) != 0 && EEPROM.read(MAG_CHECK_) != 0){
                            MOTOR_EN = 1;
                          } else {
                            Serial.println("Sensors not Calibrated");
                          }
                          rollOverDelay = 0;
			  digitalWrite(13, HIGH);
                        }

		}
		else if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle >= ARM_DISENGAGE_THRESHOLD){
			if(MOTOR_EN != 0){
                          rollOverDelay--;
                        }
                        if(rollOverDelay <= -ARM_DELAY){
                          MOTOR_EN = 0;
                          rollOverDelay = 0;
			  digitalWrite(13, LOW);
                        }
		}
                else{
                  //Stick not in arm/disarm positions
                  rollOverDelay = 0; //Reset roll over
                }
	}
	//Perhaps Add PID DISABLE Code Here
}

void checkArming(bool &MOTOR_EN, int &rollOverDelay, class BMP180 &baro){
	if (millis() > INITIAL_ARM_DELAY){
		if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle <= ARM_ENGAGE_THRESHOLD && channel3Cycle > 100 && channel4Cycle > 100){
			if(MOTOR_EN != 1){
                          //Insert alt reset code for BMP180 HERE
                          
                          rollOverDelay++;
                        }
                        
                        if(rollOverDelay >= ARM_DELAY){
                          if (EEPROM.read(ACCEL_CHECK_) != 0 && EEPROM.read(MAG_CHECK_) != 0){
                            MOTOR_EN = 1;
                          } else {
                            Serial.println("Sensors not Calibrated");
                          }
                          rollOverDelay = 0;
			  digitalWrite(13, HIGH);
                        }

		}
		else if (channel3Cycle <= ARM_THROTTLE_THRESHOLD && channel4Cycle >= ARM_DISENGAGE_THRESHOLD){
			if(MOTOR_EN != 0){
                          rollOverDelay--;
                        }
                        if(rollOverDelay <= -ARM_DELAY){
                          MOTOR_EN = 0;
                          rollOverDelay = 0;
			  digitalWrite(13, LOW);
                        }
                        
		}
                else{
                  //Stick not in arm/disarm positions
                  rollOverDelay = 0; //Reset roll over
                }
	}
	//Perhaps Add PID DISABLE Code Here
}

void checkArmingRover(bool &MOTOR_EN, int &rollOverDelay, class MS5611 &baro){
  if (millis() > INITIAL_ARM_DELAY){
    if (channel4Cycle >= ARM_ENGAGE_THRESHOLD && channel2Cycle - THROTTLE_NEUTRAL < 50 && channel4Cycle > 100 && channel2Cycle > 100){
      if(MOTOR_EN != 1){
                          #if defined(M5611_EN)
                              baro.resetReference();
                          #endif
                          rollOverDelay++;
                        }
                        if(rollOverDelay >= ARM_DELAY){
                          if(EEPROM.read(ACCEL_CHECK_) != 0 && EEPROM.read(MAG_CHECK_) != 0){
                            MOTOR_EN = 1;
                          } else {
                            Serial.println("Sensors not Calibrated");
                          }
                          rollOverDelay = 0;
        digitalWrite(13, HIGH);
                        }

    }
    else if (channel4Cycle <= ARM_DISENGAGE_THRESHOLD && channel2Cycle - THROTTLE_NEUTRAL < 50){
      if(MOTOR_EN != 0){
                          rollOverDelay--;
                        }
                        if(rollOverDelay <= -ARM_DELAY){
                          MOTOR_EN = 0;
                          rollOverDelay = 0;
        digitalWrite(13, LOW);
                        }
    }
                else{
                  //Stick not in arm/disarm positions
                  rollOverDelay = 0; //Reset roll over
                }
  }
  //Perhaps Add PID DISABLE Code Here
}

