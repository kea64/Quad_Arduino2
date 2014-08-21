#include <TinyGPS++.h>

TinyGPSPlus gps;

void setup(){
  Serial.begin(38400); 
}

void loop(){
  getGPS();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  
}

void getGPS(){
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      Serial.println(TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),40.653259,-76.959808));
      Serial.println(TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),40.653259,-76.959808));
    }
  }
}
