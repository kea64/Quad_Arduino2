#include <Servo.h> 

Servo myServo;
int pin=9;
void setup() 
{ 
  myServo.attach(9);
  delay(20);
  pinMode(pin,OUTPUT);
  digitalWrite(pin,HIGH);
  delay(12);
  digitalWrite(pin,LOW);
  myServo.write(90);
  delay(1000);
  
 
//  myServo.write();
//  delay(11);
//  myServo.write(180);
//  delay(11);
//  myServo.write(90);
  
}

void loop() 
{
//analogWrite(pin,127);
myServo.write(110);

} 
