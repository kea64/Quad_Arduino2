#include <Servo.h>

const int pin3 = 12;
const int pin4 = 11;
const int pin1 = 10;
const int pin2 = 9;

#define motorSpeed 155

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

void setup() {
  //delay(2000);
  //pinMode(pin1,OUTPUT);
  ARM_Sequence();

}

void loop() {
  ESC1.write(motorSpeed);
  ESC2.write(motorSpeed);
  ESC3.write(motorSpeed);
  ESC4.write(motorSpeed);
  delay(3000);
  ESC1.write(90);
  ESC2.write(90);
  ESC3.write(90);
  ESC4.write(90);
  delay(4000);

}

void ARM_Sequence(){
  ESC1.attach(pin1);
  ESC2.attach(pin2);
  ESC3.attach(pin3);
  ESC4.attach(pin4);
  delay(20);
  
  ESC1.write(180);
  ESC2.write(180);
  ESC3.write(180);
  ESC4.write(180);
  delay(2000);
  
  ESC1.write(90);
  ESC2.write(90);
  ESC3.write(90);
  ESC4.write(90);
  delay(3000);
  //delay(1000);
}
