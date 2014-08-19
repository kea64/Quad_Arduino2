#include <PinChangeInt.h>

unsigned long channel1Start;

volatile int channel1Cycle;

#define channel1 8
#define channel2 7
#define channel3 6

#define RC_ENABLE 1

void setup() {
  Serial.begin(115200);
  if (RC_ENABLE == 1){
    pinMode(channel1,INPUT);digitalWrite(channel1,HIGH);PCintPort::attachInterrupt(channel1,&channel1Update,CHANGE);
    //pinMode(channel2,INPUT);digitalWrite(channel2,HIGH);//PCintPort::attachInterrupt(channel2,&channel2Update,CHANGE);
    //pinMode(channel3,INPUT);digitalWrite(channel3,HIGH);//PCintPort::attachInterrupt(channel3,&channel3Update,CHANGE);
  }
}

void loop() {
  Serial.println(channel1Cycle);
  delay(50);
}

void channel1Update(){
  
    if (digitalRead(channel1) == 1){
      channel1Start = micros();
    } else {
      channel1Cycle = micros() - channel1Start;
      //PCintPort::detachInterrupt(channel1);
      //PCintPort::detachInterrupt(channel3);
      //PCintPort::attachInterrupt(channel2,&channel2Update,CHANGE);
    }
}



