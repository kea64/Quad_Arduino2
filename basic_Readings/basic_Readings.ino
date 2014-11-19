#include <Quad_HMC5883L.h>
#include <Wire.h>

HMC5883L mag;

void setup(){
  Serial.begin(38400);
  Wire.begin();
  mag.init(0.94,0.99,0.94,-90,-85,225);
}

void loop(){
  mag.readMag();
  
  Serial.print("X: ");
  Serial.println(mag.xScaled);
  Serial.print("Y: ");
  Serial.println(mag.yScaled);
  Serial.print("Z: ");
  Serial.println(mag.zScaled);
  
  delay(50);
}
