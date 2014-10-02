#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 pressure;

double baseline;
double alt = 0.0;
double globalSpeedRaw = 0.0;

unsigned long newTimer,oldTimer;
int globalSpeed = 0;
int cycle;
int count = 0;
int maxCount = 300;

#define altAlpha 0.3
#define targetAlt 1.0
#define speedAggression 0.5

void setup()
{
  Serial.begin(38400);
  pressure.begin();
  baseline = getPressure(); 
  oldTimer = millis();
}

void loop()
{
  getAlt();
  //elev();
  //cycleTimer();
  
  //Serial.print("Alt: ");
  //Serial.println(alt);
  //Serial.print("Speed: ");
  //Serial.println(globalSpeedRaw);
  //Serial.print("Cycle: ");
  //Serial.println(cycle);
  count += 1;
  if (count <= maxCount){
      Serial.println(alt);
  }
  delay(20);
}


double getPressure()
{
  char status;
  double T,P,p0,a;
  status = pressure.startTemperature();
  if (status != 0)
  {
    //delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        //delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

void getAlt(){
  double a,P;
  P = getPressure();
  a = pressure.altitude(P,baseline);
  
  alt = altAlpha * alt + (1-altAlpha) * a;
  
}

void elev(){
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
