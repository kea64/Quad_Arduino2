#include <PinChangeInt.h>

unsigned long channel1Start, channel2Start, channel3Start;

volatile int channel1Cycle,channel2Cycle,channel3Cycle;

#define channel1 8
#define channel2 7
#define channel3 6
#define RC_ENABLE 1

void setup() {
  Serial.begin(115200);
  if (RC_ENABLE == 1){
    pinMode(channel1,INPUT);digitalWrite(channel1,HIGH);PCintPort::attachInterrupt(channel1,&channel1Update,CHANGE);
    pinMode(channel2,INPUT);digitalWrite(channel2,HIGH);//PCintPort::attachInterrupt(channel2,&channel2Update,CHANGE);
    pinMode(channel3,INPUT);digitalWrite(channel3,HIGH);//PCintPort::attachInterrupt(channel3,&channel3Update,CHANGE);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //PCintPort::detachInterrupt(channel1);
}

void channel1Update(){
  
    if (digitalRead(channel1) == 1){
      channel1Start = micros();
    } else {
      channel1Cycle = micros() - channel1Start;
      PCintPort::detachInterrupt(channel1);
      PCintPort::detachInterrupt(channel3);
      PCintPort::attachInterrupt(channel2,&channel2Update,CHANGE);
    }
}

void channel2Update(){
    if (digitalRead(channel2) == 1){
      channel2Start = micros();
    } else {
      channel2Cycle = micros() - channel2Start;
      PCintPort::detachInterrupt(channel2);
      PCintPort::detachInterrupt(channel1);
      PCintPort::attachInterrupt(channel3,&channel3Update,CHANGE);
    }
}

void channel3Update(){
    if (digitalRead(channel3) == 1){
      channel2Start = micros();
    } else {
      channel3Cycle = micros() - channel3Start;
      PCintPort::detachInterrupt(channel3);
      PCintPort::detachInterrupt(channel2);
      PCintPort::attachInterrupt(channel1,&channel1Update,CHANGE);
    }
}

