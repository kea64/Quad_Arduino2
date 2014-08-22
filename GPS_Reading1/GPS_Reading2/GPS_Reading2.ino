#include <TinyGPS++.h>

unsigned long GPSClockOld,simClockOld;
int simInc = 0;

#define GPS_DELAY 0
#define SIM_DELAY 1

TinyGPSPlus gps;

void setup(){
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  Serial.begin(38400); 
  GPSClockOld = millis();
}

void loop(){
  if ((millis() - GPSClockOld) >= GPS_DELAY){
    incSim();
    getGPS();
    GPSClockOld = millis();
  }
  
  if ((millis() - simClockOld) >= SIM_DELAY){
    //incSim();
    //simClockOld = millis();
  }
  
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  
}

void incSim(){
  simInc += 1;
}

void getGPS(){
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      Serial.println(TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),40.653259,-76.959808));
      Serial.println(TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),40.653259,-76.959808));
      Serial.println(simInc);
      if (gps.satellites.value() >= 5){
        digitalWrite(13,HIGH);
      } else {
        digitalWrite(13,LOW);
      }
    }
  }
}
