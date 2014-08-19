#include <Servo.h> 
//92 is the neutral frequency, it does not move the motor at all
Servo myServo;
Servo frontWheel;
int pin=9;
void setup() 
{ 
  myServo.attach(9);
  frontWheel.attach(10);
  frontWheel.write(87);
  delay(1000);
  pinMode(pin,OUTPUT);
  digitalWrite(pin,HIGH);
  delay(12);
  digitalWrite(pin,LOW);
  delay(8.5);

//  myServo.write(110);
//  delay(2000);
//  myServo.write(90);
//  delay(1500);
//  myServo.write(70);
//  delay(1500);
//  myServo.write(90);

 
 
//  myServo.write();
//  delay(11);
//  myServo.write(180);
//  delay(11);
//  myServo.write(90);
  
}

void loop() 
{

myServo.write(107);
delay(1000);



myServo.write(77);
delay(500);

myServo.write(92);
delay(1000);

myServo.write(82);
delay(1000);
//
myServo.write(92);
delay(1000);
//myServo.write(92);
} 
