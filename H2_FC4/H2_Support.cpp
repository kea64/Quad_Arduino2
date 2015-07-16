#include <Arduino.h>
#include "H2_Support.h"
#include "H2_Drone_Values.h"

extern unsigned long channel1Start, channel2Start, channel3Start, channel4Start, channel5Start, channel6Start;

extern volatile int channel1Cycle, channel2Cycle, channel3Cycle, channel4Cycle, channel5Cycle, channel6Cycle;

void channel1Interrupt(){
	if (digitalRead(channel1) == 1){
		channel1Start = micros();
	}
	else {
		channel1Cycle = micros() - channel1Start;
	}
}

void channel2Interrupt(){
	if (digitalRead(channel2) == 1){
		channel2Start = micros();
	}
	else {
		channel2Cycle = micros() - channel2Start;
	}
}

void channel3Interrupt(){
	if (digitalRead(channel3) == 1){
		channel3Start = micros();
	}
	else {
		channel3Cycle = micros() - channel3Start;
	}
}

void channel4Interrupt(){
	if (digitalRead(channel4) == 1){
		channel4Start = micros();
	}
	else {
		channel4Cycle = micros() - channel4Start;
	}
}

void channel5Interrupt(){
	if (digitalRead(channel5) == 1){
		channel5Start = micros();
	}
	else {
		channel5Cycle = micros() - channel5Start;
	}
}

void channel6Interrupt(){
	if (digitalRead(channel6) == 1){
		channel6Start = micros();
	}
	else {
		channel6Cycle = micros() - channel6Start;
	}
}

double intMap(int inValue, int inLow, int inHigh, int outLow, int outHigh){
	return ((inValue - inLow) * (1.0 * (outHigh - outLow)) / (1.0 * (inHigh - inLow)) + outLow);
}

void withinBounds(int &value, int upper, int lower){
	if (value > upper){ value = upper; }
	if (value < lower){ value = lower; }
}
