//Code which takes RC heli servo control signal, filters it, and then writes to a servo

#include <Servo.h>
#define alpha 0.35 //defines filter constant--closer to 1 means less filtering
int pin = 7;
float steerOld,steerNew,raw; //variables to hold translations

unsigned long duration;
Servo steering;

void setup()
{
  Serial.begin(115200);
  steering.attach(9);//servo setup
  steering.write(90);
  pinMode(pin, INPUT);
}

void loop()
{
  duration = pulseIn(pin, HIGH); //records servo pulse
  Serial.println(duration);
  
  if (duration >= 1000 && duration <= 2000){ //Ensures servo signal is correct
    steerNew = (180.0*(duration-1000)/1000.0 //
    steerOld = (1-alpha)*steerOld+alpha*steerNew;//Applies Noise Filter
    steering.write(steerOld);
    Serial.println(steerOld);
   
  }
  
}
