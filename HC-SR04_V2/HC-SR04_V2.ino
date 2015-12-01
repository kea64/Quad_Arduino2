#include <PinChangeInt.h>

#define trigPin1 3
#define echoPin1 2

#define cutOffTime 24000

volatile int ping1Dist = -1;

void setup() {
  Serial.begin(115200);
  
  pinMode(echoPin1,INPUT);digitalWrite(echoPin1,HIGH);PCintPort::attachInterrupt(echoPin1,&pingInterrupt1,CHANGE);
  pinMode(trigPin1, OUTPUT); digitalWrite(trigPin1, LOW);
  
}

void loop() {
  ping1Start();
  delay(40);
  Serial.print("Distance: ");
  Serial.println(ping1Dist);
  
  delay(30);
  

}

void ping1Start(){
  ping1Dist = 0;
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
}

void pingInterrupt1(){
  static unsigned long startClock;
  if (digitalRead(echoPin1) == HIGH){
    startClock = micros();
  } else {
    int cycle = micros() - startClock;
    if (cycle <= cutOffTime){
      ping1Dist = cycle / 58.2;
    } else {
      ping1Dist = -1;
    }
  }
}
