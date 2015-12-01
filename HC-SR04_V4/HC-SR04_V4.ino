#include <PinChangeInt.h>

#define trig1Pin 3
#define trig2Pin 5

#define echo1Pin 2
#define echo2Pin 4

#define SAMPLE_PERIOD 24000

void setup() {
  Serial.begin(115200);
  
  pinMode(trig1Pin, OUTPUT); digitalWrite(trig1Pin, LOW);
  pinMode(trig2Pin, OUTPUT); digitalWrite(trig2Pin, LOW);
  
  pinMode(echo1Pin, INPUT); digitalWrite(echo1Pin, HIGH); PCintPort::attachInterrupt(echo1Pin,&pingInterrupt1,CHANGE);
  pinMode(echo2Pin, INPUT); digitalWrite(echo2Pin, HIGH); PCintPort::attachInterrupt(echo2Pin,&pingInterrupt1,CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:

}
