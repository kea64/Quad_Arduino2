#include <PinChangeInt.h>
#include "HCSR04.h"
#include "Linked_List_Template.h"

#define trig1Pin 3
#define trig2Pin 5
#define echo1Pin 2
#define echo2Pin 4

#define cutOffTime 24000

HCSR04* sensor1 = new HCSR04(trig1Pin, echo1Pin, cutOffTime);
HCSR04* sensor2 = new HCSR04(trig2Pin, echo2Pin, cutOffTime);


void setup() {
  Serial.begin(115200);
  
  PCintPort::attachInterrupt(echo1Pin, int1Service, CHANGE);
  PCintPort::attachInterrupt(echo2Pin, int2Service, CHANGE);
  
}

void loop() {
  Serial.println("Break");
  Serial.println("Break");
  Serial.println("Break");
  Serial.println("Break");
  sensor1->ping();
  //sensor2->ping();
  delay(25);
  Serial.print("Sen 1: ");
  Serial.println(sensor1->getDist());
  sensor2->ping();
  delay(25);
  Serial.print("Sen 2: ");
  Serial.println(sensor2->getDist());

}

void int1Service(){
  sensor1->pingInterrupt();
}

void int2Service(){
  sensor2->pingInterrupt();
}
