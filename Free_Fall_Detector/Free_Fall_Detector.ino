#include <Wire.h>
#include <ADXL345.h>

unsigned long oldTimer;
int cycle;

ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

void setup(){
  Serial.begin(115200);
  adxl.powerOn();

  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(8); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(20); //(20 - 70) recommended - 5ms per increment
 
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  oldTimer = millis();
}

void loop(){
  
  //Boring accelerometer stuff 
    
  int x,y,z;  
  adxl.readAccel(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z

  
   byte interrupts = adxl.getInterruptSource();
  
  // freefall
  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
    Serial.println("freefall");
    //add code here to do when freefall is sensed
  } 
  
  //cycle = millis() - oldTimer;
  //oldTimer = millis();
  //Serial.println(cycle);
}
