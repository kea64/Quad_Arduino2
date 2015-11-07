//MPU6050 Speed Test

#include <Wire.h>

#define TX_DELAY 250
#define TEST_DELAY 50
#define TEST2_DELAY 1
#define DATA_DELAY 1

//MPU6050
#define MPU6050_address 0x68
#define MPU6050_I2C_BYPASS 0x37
#define MPU6050_self_test_x 13       // R/W
#define MPU6050_self_test_y 14       // R/W
#define MPU6050_self_test_z 15       // R/W
#define MPU6050_self_test_A 16       // R/W
#define MPU6050_sample_div 25        // R/W
#define MPU6050_config 26            // R/W
#define MPU6050_gyro_config 27       // R/W
#define MPU6050_accel_config 28      // R/W
#define MPU6050_data_start 59
#define MPU6050_PWR1 107
#define MPU6050_PWR2 108
#define g 9.81                       // Gravitational acceleration

class MPU6050{
  public:
    MPU6050();
    void init(int gyroGain, int accelGain, int DLPF);
    void enableI2CBypass();
    void readData();
    void resetWake();
    void setDLPF(int BW);
    void setGains(int gyro, int accel);
    void offsetCal();
    void accelCalib();
    
    float temp;
    float ax, ay, az;
    float gx, gy, gz;
    
    float axr, ayr, azr;
    float gxr, gyr, gzr;
    
  private: 
    int accel_x_OC;
    int accel_y_OC;
    int accel_z_OC;
    int gyro_x_OC;
    int gyro_y_OC;
    int gyro_z_OC;
    
    float accel_scale_fact;
    float gyro_scale_fact;
    
    
};

MPU6050 mpu;

unsigned long TXOldClock = 0;
unsigned long dataOldClock = 0;
unsigned long TestOldClock = 0;
unsigned long Test2OldClock = 0;

int dataCount = 0;
int dataCycle = 0;

void setup() {
  Serial.begin(115200);
  
  TXOldClock = millis();
  dataOldClock = millis();
  TestOldClock = millis();
  Test2OldClock = millis();
}

void loop() {
  
  mpu.init(1, 1, 3);
  
  while(1==1){
    if (millis() - TXOldClock > TX_DELAY){
      
      Serial.print("GX: ");
      Serial.println(mpu.gx);
      
      Serial.print("Data: ");
      Serial.println(dataCount * (1000 / TX_DELAY));
      
      Serial.print("Cycle: ");
      Serial.println(dataCycle);
     
      dataCount = 0;
      TXOldClock = millis();
      
      
    }
    
    if (millis() - TestOldClock > TEST_DELAY){
      delay(1);
      
      TestOldClock = millis();
    }
    
    if (millis() - Test2OldClock > TEST2_DELAY){
      delay(1);
      
      Test2OldClock = millis();
    }
    
    checkFunction(mpu);
  }
}

void checkFunction(class MPU6050 &mpu){
  if (millis() - dataOldClock > DATA_DELAY){
    unsigned long start = micros();
    mpu.readData();
    dataCount++;
    
    dataCycle = micros() - start;
    
    dataOldClock = millis();
    
    
  }
}

MPU6050::MPU6050(){
  accel_x_OC = 0;
  accel_y_OC = 0;
  accel_z_OC = 0;
  gyro_x_OC = 0;
  gyro_y_OC = 0;
  gyro_z_OC = 0;
  
  ax = 0;
  ay = 0;
  az = 0;
  gx = 0;
  gy = 0;
  gz = 0;
  temp = 0;
  
  axr = 0;
  ayr = 0;
  azr = 0;
  gxr = 0;
  gyr = 0;
  gzr = 0;
  
  accel_scale_fact = 1;
  gyro_scale_fact = 1;
}

void MPU6050::init(int gyroGain, int accelGain, int DLPF){
  resetWake();
  setGains(gyroGain, accelGain);
  setDLPF(0);
  offsetCal();
  setDLPF(DLPF);
  enableI2CBypass();
  
  //Accel Offset loading
  //EEPROM.get(ACCEL_OFFSET_X_, accel_x_OC);
  //EEPROM.get(ACCEL_OFFSET_Y_, accel_y_OC);
  //EEPROM.get(ACCEL_OFFSET_Z_, accel_z_OC);
}

void MPU6050::enableI2CBypass(){
  Wire.beginTransmission(MPU6050_address);
  Wire.write(MPU6050_I2C_BYPASS);
  Wire.write(2);
  Wire.endTransmission();
}

void MPU6050::readData(){
  Wire.beginTransmission(MPU6050_address);
  Wire.write(MPU6050_data_start);
  Wire.endTransmission();

  int read_bytes = 14;
  int temp = 0;

  Wire.requestFrom(MPU6050_address,read_bytes);

  if(Wire.available() == read_bytes){
  
    axr = Wire.read()<<8 | Wire.read();
    ayr = Wire.read()<<8 | Wire.read();
    azr = Wire.read()<<8 | Wire.read();
  
    temp = Wire.read()<<8 | Wire.read();
  
    gxr = Wire.read()<<8 | Wire.read();
    gyr = Wire.read()<<8 | Wire.read();
    gzr = Wire.read()<<8 | Wire.read();
  
  }
  
  
  
  ax = (float)(axr-accel_x_OC)*accel_scale_fact/1000; // divided by 1000 as the Scale factor is in milli units
  ay = (float)(ayr-accel_y_OC)*accel_scale_fact/1000;
  az = (float)(azr-accel_z_OC)*accel_scale_fact/1000;
  
  gx = (float)(gxr-gyro_x_OC)*gyro_scale_fact/1000;
  gy = (float)(gyr-gyro_y_OC)*gyro_scale_fact/1000;
  gz = ((float)(gzr-gyro_z_OC)*gyro_scale_fact/1000);
  
  temp = (float)temp/340+36.53;
}

void MPU6050::resetWake(){
  
//  //Serial.println("Resetting MPU6050 and waking it up.....");
//  Wire.beginTransmission(MPU6050_address);
//  Wire.write(MPU6050_PWR1);
//  Wire.write(0b10000000);
//  Wire.endTransmission();
//  
//  delay(100); // Waiting for the reset to complete
//   
//  Wire.beginTransmission(MPU6050_address);
//  Wire.write(MPU6050_PWR1);
// 
//  Wire.write(0b00000000);
//  Wire.endTransmission();

  i2c_writeReg(MPU6050_address, 0x6B, 0x80); //Reset
  delay(5);
  i2c_writeReg(MPU6050_address, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
}

void MPU6050::setDLPF(int BW){
    if (BW < 0 || BW > 6){
      BW = 0;
    }
  Wire.beginTransmission(MPU6050_address);
  Wire.write(MPU6050_config); // Address to the configuration register
/*       config Discription ---- x x 0 0 0 F2 F1 F0 
  I am only intrested in the Digital Low Pass Filter (DLPF)
  F2 F1 F0    Bandwidth [Hz]
  0  0  0        
  0  0  1      184
  0  1  0      94
  0  1  1      44
  1  0  0      21  
  1  0  1      10  
  1  1  0      5
*/

  Wire.write(BW);
  Wire.endTransmission();
}

void MPU6050::setGains(int gyro,int accel){
    byte gyro_byte,accel_byte;
    
    // Setting up Gyro
    Wire.beginTransmission(MPU6050_address);
    Wire.write(MPU6050_gyro_config); // Address to the configuration register
    if (gyro==0)
    {
      gyro_scale_fact =(float)250*0.0305; // each data is of 16 bits that means, 250 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      gyro_byte = 0b00000000;
    }else if (gyro == 1)
    {
      gyro_scale_fact = 500*0.0305; // each data is of 16 bits that means, 500 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      gyro_byte = 0b00001000;
    }else if (gyro == 2)
    {
      gyro_scale_fact = 1000*0.0305;// each data is of 16 bits that means, 1000 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      gyro_byte = 0b00010000;
    }else if (gyro == 3)
    {
      gyro_scale_fact = 2000*0.0305;  // each data is of 16 bits that means, 2000 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
      gyro_byte = 0b00011000;
    }else
    {
      gyro_scale_fact = 1;
    }  
      
    Wire.write(gyro_byte);
    Wire.endTransmission();
    //Serial.print("The gyro scale is set to ");
    //Serial.print(gyro_scale_fact);
    //Serial.println(" milli Degree/s");
    
    
    // Setting up Accel
    Wire.beginTransmission(MPU6050_address);
    Wire.write(MPU6050_accel_config); // Address to the configuration register
    if (accel==0)
    {
      accel_scale_fact =(float)2*g*0.0305; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767       
      accel_byte = 0b00000000;
    }else if (accel == 1)
    {
      accel_scale_fact = 4*g*0.0305; // each data is of 16 bits that means, 4g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      accel_byte = 0b00001000;
    }else if (accel == 2)
    {
      accel_scale_fact = 8*g*0.0305;// each data is of 16 bits that means, 8g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      accel_byte = 0b00010000;
    }else if (accel == 3)
    {
      accel_scale_fact = 16*g*0.0305; // each data is of 16 bits that means, 16g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
      accel_byte = 0b00011000;
    }else
    {
      accel_scale_fact = 1;
    }  
      
    Wire.write(accel_byte);
    Wire.endTransmission();
    //Serial.print("The accel scale is set to ");
    //Serial.print(accel_scale_fact);
    //Serial.println(" milli m/s^2");  
}

void MPU6050::offsetCal(){
  //Serial.println("Calibrating gyroscope .... dont move the hardware ..........");
  int x=0,y=0,z=0,i;
  
  readData();
  readData();
  
  // Gyro Offset Calculation
  x=gxr;
  y=gyr;
  z=gzr;
    
  for (i=1;i<=50;i++){
    readData();
    x=(x+gxr)/2;
    y=(y+gyr)/2;
    z=(z+gzr)/2;
    //Serial.print(".");
    delay(10);
  }
  //Serial.println(".");
  gyro_x_OC=x;
  gyro_y_OC=y;
  gyro_z_OC=z;
  
  //Serial.print("gyro_x register offset = ");
  //Serial.println(x);
 
  
  //Serial.print("gyro_y register offect = ");
  //Serial.println(y);
 
  
  //Serial.print("gyro_z register offset = ");
  //Serial.println(z);
 
  
  /*
  // Accel Offset Calculation
  //Serial.println("Calibrating accelrometer .... dont move the hardware ..........");
  x=axr;
  y=ayr;
  z=azr;
    
  for (i=1;i<=50;i++){
    readData();
    x=(x+axr)/2;
    y=(y+ayr)/2;
    z=(z+azr)/2;
    //Serial.print(".");
    delay(10);
  }
  //Serial.println(".");
  accel_x_OC=x;
  accel_y_OC=y;
  accel_z_OC=z-(float)g*1000/accel_scale_fact;
  */
}

void MPU6050::accelCalib(){
  #if defined(CRIUS)
  float rawExtrema[6] = {0,0,0,0,0,0};
  Serial.println("Start Calibration");
  
  //Obtain new data
  for (int i = 1; i <= 6; i++){
    Serial.print("S"); Serial.println(i);
       
       if(SerialRequest() == 1){
         readData();
         if (axr > rawExtrema[0]){
           rawExtrema[0] = axr;
         } else if (axr < rawExtrema[1]){
           rawExtrema[1] = axr;
         }
         if (ayr > rawExtrema[2]){
           rawExtrema[2] = ayr;
         } else if (ayr < rawExtrema[3]){
           rawExtrema[3] = ayr;
         }
         if (azr > rawExtrema[4]){
           rawExtrema[4] = azr;
         } else if (azr < rawExtrema[5]){
           rawExtrema[5] = azr;
         }
       } else {
         i = 7;
       }
       
    Serial.println("End Calibration");
  }
  
  //Error checking for weird values or timeouts
  bool error = 0;
  for (int i = 0; i < 6; i++){
    if (rawExtrema[i] == 0.0){
      error = 1;
    }
  }
  
  //Write new offsets
  if (!error){
    //Serial.println(rawExtrema[0]);
    //Serial.println(rawExtrema[1]);
    //Serial.println(rawExtrema[2]);
    //Serial.println(rawExtrema[3]);
    //Serial.println(rawExtrema[4]);
    //Serial.println(rawExtrema[5]);
    //EEPROM Write Here!
    accel_x_OC = (rawExtrema[0] + rawExtrema[1])/2;
    accel_y_OC = (rawExtrema[2] + rawExtrema[3])/2;
    accel_z_OC = ((rawExtrema[4] + rawExtrema[5])/2);// - (float)g*1000/accel_scale_fact;
    EEPROM.put(ACCEL_OFFSET_X_, accel_x_OC);
    EEPROM.put(ACCEL_OFFSET_Y_, accel_y_OC);
    EEPROM.put(ACCEL_OFFSET_Z_, accel_z_OC); 
    
  } else {
    Serial.println("E3");
  }
  #endif
}

void i2c_writeReg(byte addr, byte reg, byte val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
