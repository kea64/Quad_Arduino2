//Arduino 1.0+ only

#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define gyroConversion (0.0175)

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;
double GyroX;
double GyroY;
double GyroZ;

double g_offx = 0;
double g_offy = 0;
double g_offz = 0;

double gyroHeading = 0;

unsigned long clockOld;

void setup(){

  Wire.begin();
  Serial.begin(38400);

  Serial.println("starting up L3G4200D");
  setupL3G4200D(500); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  GyroCalibrate();
}

void loop(){
   getGyroValues();  // This will update x, y, and z with new values
  
  int clockNew = millis();
  int cycle = clockNew - clockOld;
  clockOld = clockNew;
  gyroHeading = gyroHeading + (1.0)*GyroY*((cycle*1.0)/1000.0);
  
  Serial.print("Heading: ");
  Serial.println(gyroHeading);
  
  Serial.print("X:");
  Serial.print(GyroX);

  Serial.print(" Y:");
  Serial.print(GyroY);

  Serial.print(" Z:");
  Serial.println(GyroZ);

  delay(100); //Just here to slow down the serial to make it more readable
}

void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);
  GyroX = x * gyroConversion - g_offx;
  
  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);
  GyroY = y * gyroConversion - g_offy;
  
  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
  GyroZ = z * gyroConversion - g_offz;
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}

void GyroCalibrate(){

 int tmpx = 0;
 int tmpy = 0;
 int tmpz = 0; 

 g_offx = 0;
 g_offy = 0;
 g_offz = 0;
 
 for (char i = 0;i<25;i++)
    {
    delay(10);  
    getGyroValues();
    tmpx += GyroX;
    tmpy += GyroY;
    tmpz += GyroZ; 
    }  
 g_offx = tmpx/25.0;
 g_offy = tmpy/25.0;
 g_offz = tmpz/25.0;
 
 Serial.println(g_offx);
 Serial.println(g_offy);
 Serial.println(g_offz);
  
}
