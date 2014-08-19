#include <Servo.h>       //Improts servo library
#include <Wire.h>        //Imports Wire library for Compass

long incomingByte = 0;   // for incoming serial data

Servo Esc;
Servo frontWheel;

int escPin=9;
int printTest;
int HMC6352Address = 0x42;
int slaveAddress;
int currentHeading; //for comapss method, do not know if it will be used
int curHeading;     //used for the getHeading method
int startNumber=0;  //1= start, 0 = waiting for command to start, itll change when it recieves data
int ledPin = 13;
boolean ledState = false;
byte headingData[2];
int i, headingValue;

void setup() {
     Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
     Serial.print("Serial Port Connected"); 
     Serial.write(13); Serial.write(10);
     Esc.attach(9);
     slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
     Wire.begin();
     delay(1000);
}

void loop(){
  Serial.println(getHeading());
}

int getHeading()
{
    Wire.beginTransmission(slaveAddress);
    Wire.write("A");              //Get Data
    Wire.endTransmission();
    delay(10);                   // 70us delay
    Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading
    i = 0;
    while(Wire.available() && i < 2)
    {
      headingData[i] = Wire.read();
      i++;
    }
    headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together
    curHeading=headingValue/10;
    return curHeading;
}
