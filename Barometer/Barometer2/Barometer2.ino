#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 pressure;

double baseline,T,P,p0,a;
double alt = 0.0;
double globalSpeedRaw = 0.0;

char status;

unsigned long newTimer,oldTimer;
int globalSpeed = 0;
int cycle;
int i = 1002;

#define altAlpha 0.6
#define targetAlt 1.0
#define speedAggression 0.5

void setup()
{
  Serial.begin(115200);
  pressure.begin();
  getTemp();
  getBaro();
  baseline = P; 
  oldTimer = millis();
}

void loop()
{
  if (i > 1000){
    getTemp();
    i = 0;
  }
  else{
    i += 1;
  }
  getBaro();
  calcAlt();
  elev();
  cycleTimer();
  
  Serial.print("Alt: ");
  Serial.println(alt);
  //Serial.print("Speed: ");
  //Serial.println(globalSpeedRaw);
  //Serial.print("Cycle: ");
  Serial.print("Cycle: ");
  Serial.println(cycle);
  Serial.print("Temp: ");
  Serial.println(T);
  
  
}

void getBaro(){
  status = pressure.startPressure(0);
  if (status != 0){
    delay(status);
    status = pressure.getPressure(P,T);
  }   
  //alt = altAlpha * alt + (1-altAlpha) * pressure.altitude(P,baseline); 
}

void elev(){
  //a = pressure.altitude(P,baseline);
  //alt = altAlpha * alt + (1-altAlpha) * a;
  globalSpeedRaw = (targetAlt - alt) * speedAggression + globalSpeedRaw;
  if (globalSpeedRaw > 90.0){globalSpeedRaw = 90.0;}
  if (globalSpeedRaw <= 0.0){globalSpeedRaw = 0.0;}
  globalSpeed = int(globalSpeedRaw);
}

void cycleTimer(){
  newTimer = millis();
  cycle = newTimer - oldTimer;
  oldTimer = newTimer;
}

void getTemp(){
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
  }
}

void calcAlt(){
  a = pressure.altitude(P,baseline);
  alt = altAlpha * alt + (1-altAlpha) * a;
}
