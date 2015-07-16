#ifndef H2_Support_h
#define H2_Support_h

#include <Arduino.h>
#include "H2_Support.h"
#include "H2_Drone_Values.h"

//Pin Interrupt Routines
void channel1Interrupt();
void channel2Interrupt();
void channel3Interrupt();
void channel4Interrupt();
void channel5Interrupt();
void channel6Interrupt();

//Basic Formatting Functions
double intMap(int inValue, int inLow, int inHigh, int outLow, int outHigh);
void withinBounds(int &value, int upper, int lower);

#endif
