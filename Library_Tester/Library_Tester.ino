#include <ADXL345.h>
#include <Wire.h>
#include <compass.h>
#include <Quad_HMC5883L.h>

class stuff{
  public:
    int inches(int &y, int &z);
    int retrieve() const;
    void change(int x);
    int moreStuff;
  private:
    int ans_;
    int multi_;
};

int y = 5;
int z = 7;
int x = 100;

  
void setup(){
  Serial.begin(38400);
  Wire.begin();
  ADXL345 adxl;
  //Quad_HMC5883L;
  //mag.init(y);
  adxl.powerOn();
  compass_init(2);
  compass_scalled_reading();
  //compass_heading();
}

void loop(){
  stuff stuff;
  stuff.inches(y,z);
  stuff.change(x);
  stuff.moreStuff = 101;
  Serial.println(y);
  delay(50);
}

int stuff::inches(int &y, int &z){
  y = z + y;
  int e = z + y;
  return(e);
}

int stuff::retrieve() const{
  return(ans_);
}

void stuff::change(int x){
  ans_ = x;
}

