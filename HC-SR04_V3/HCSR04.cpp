#include <Arduino.h>
//#include <PinChangeInt.h>
#include "HCSR04.h"

HCSR04::HCSR04(int trigPin, int echoPin, int cutOffTime = 24000){
  //Default Constructor
  trigPin_ = trigPin;
  echoPin_ = echoPin;
  cutOffTime_ = cutOffTime;
  
  //Set pins on MCU
  pinMode(echoPin_, INPUT); digitalWrite(echoPin_, HIGH); 
  pinMode(trigPin_, OUTPUT); digitalWrite(trigPin_, LOW);
  
}

void HCSR04::pingInterrupt(){ 
  //Ping Interrupt Service
  //Samples the response of the sensor to determine the raw distance
  //Will set -1 if distance is greater than about 4 meters
  if (digitalRead(echoPin_) == HIGH){
    startClock_ = micros();
  } else {
    int cycle = micros() - startClock_;
    if (cycle <= cutOffTime_){
      pingDist_ = cycle / 58.2;
    } else {
      pingDist_ = -1;
    }
  }
  
}

void HCSR04::ping(){
  //Trigger the ultrasonic sensor
  //Will only execute if trigger hasn't been activated recently
  if (micros() - startClock_ >= cutOffTime_){
    pingDist_ = 0; //reset distance
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);
  }
}

int HCSR04::getDist(){
  return(pingDist_);
}
