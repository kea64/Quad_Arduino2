#include <Wire.h>

//HMC5883L
#define compass_address 0x1E       // The I2C address of the Magnetometer
//#define compass_XY_excitation 1160 // The magnetic field excitation in X and Y direction during Self Test (Calibration)
//#define compass_Z_excitation 1080  // The magnetic field excitation in Z direction during Self Test (Calibration)
//#define compass_rad2degree 57.3


//#define compass_cal_x_offset 116   // Manually calculated offset in X direction
//#define compass_cal_y_offset 225   // Manually calculated offset in Y direction
//#define compass_cal_x_gain 1.1     // Stored Gain offset at room temperature
//#define compass_cal_y_gain 1.12    // Stored Gain offset at room temperature

#define calibRunTime 30000

double xScaled;
double yScaled;
double zScaled;

double gainFactor_;
double xGainError_, yGainError_, zGainError_;
double xOffset_, yOffset_, zOffset_;
int mX_, mY_, mZ_;


void setup() {
  Serial.begin(115200);
  init(1,1,1,0,0,0);
  calibrate();

}

void loop() {
  update();
  Serial.print("mX: ");
  Serial.println(xScaled);
  Serial.print("mY: ");
  Serial.println(yScaled);
  Serial.print("mZ: ");
  Serial.println(zScaled);
  Serial.print("Heading: ");
  Serial.println(atan2(yScaled, xScaled)*(180/PI));
  
  delay(100);

}

void calibrate(){
  Serial.println("Begin Calibration");
  init(1,1,1,0,0,0);
  
  unsigned long startTime = millis();
  
  float xMin = 0;
  float xMax = 0;
  float yMin = 0;
  float yMax = 0;
  float zMin = 0;
  float zMax = 0;
  
  while(millis() - startTime < calibRunTime){
    update();
    
    /*
    if ((mX_ * gainFactor_) > xMax){
      xMax = (mX_ * gainFactor_);
    } else if ((mX_ * gainFactor_) < xMin){
      xMin = (mX_ * gainFactor_);
    }
    
    if ((mY_ * gainFactor_) > yMax){
      yMax = (mY_ * gainFactor_);
    } else if ((mY_ * gainFactor_) < yMin){
      yMin = (mY_ * gainFactor_);
    }
    
    if ((mZ_ * gainFactor_) > zMax){
      zMax = (mZ_ * gainFactor_);
    } else if ((mZ_ * gainFactor_) < zMin){
      zMin = (mZ_ * gainFactor_);
    }
    */
    if (mX_ > xMax){
      xMax = mX_;
    } else if (mX_ < xMin){
      xMin = mX_;
    }
    
    if (mY_ > yMax){
      yMax = mY_;
    } else if (mY_ < yMin){
      yMin = mY_;
    }
    
    if (mZ_ > zMax){
      zMax = mZ_;
    } else if (mZ_ < zMin){
      zMin = mZ_;
    }
  }
  
  Serial.println("End Calibration");
  
  xOffset_ = (xMax + xMin) / 2;
  yOffset_ = (yMax + yMin) / 2;
  zOffset_ = (zMax + zMin) / 2;
  
  float F_AVG = ((xMax - xOffset_) + (yMax - yOffset_) + (zMax - zOffset_)) / 3;
  
  xGainError_ = F_AVG / (xMax - xOffset_);
  yGainError_ = F_AVG / (yMax - yOffset_);
  zGainError_ = F_AVG / (zMax - zOffset_);
  
  Serial.print("xOffset: ");
  Serial.println(xOffset_);
  Serial.print("yOffset: ");
  Serial.println(yOffset_);
  Serial.print("zOffset: ");
  Serial.println(zOffset_);
  Serial.print("xGain: ");
  Serial.println(xGainError_);
  Serial.print("yGain: ");
  Serial.println(yGainError_);
  Serial.print("zGain: ");
  Serial.println(zGainError_);
  
}

void init(double xGainError, double yGainError, double zGainError, double xOffset, double yOffset, double zOffset){
	Wire.beginTransmission(compass_address);
	Wire.write(0x01);

	byte gain_reg = 0b01000000;
	gainFactor_ = 1.22;
	
	Wire.write(gain_reg); // bit configuration = g2 g1 g0 0 0 0 0 0, g2 g1 g0 = 0 0 1 for 1.3 guass and 0 1 0 for 1.9 Guass
	Wire.write(0b00000011);  // Putting the Magnetometer in idle
	// Writing the register value 0000 0000 for continous mode
	// Writing the register value 0000 0001 for single
	// Writing the register value 0000 0011 for Idel
	Wire.endTransmission();

	xGainError_ = xGainError;
	yGainError_ = yGainError;
	zGainError_ = zGainError;

	xOffset_ = xOffset;
	yOffset_ = yOffset;
	zOffset_ = zOffset;
}


void update(){

	poll();

	//xScaled = mX_ * gainFactor_ * xGainError_ - xOffset_;
	//yScaled = mY_ * gainFactor_ * yGainError_ - yOffset_;
	//zScaled = mZ_ * gainFactor_ * zGainError_ - zOffset_;

        xScaled = (mX_ - xOffset_) * gainFactor_ * xGainError_;
        yScaled = (mY_ - yOffset_) * gainFactor_ * yGainError_;
        zScaled = (mZ_ - zOffset_) * gainFactor_ * zGainError_;
        
        //xScaled = ((mX_ * gainFactor_) - xOffset_) * xGainError_;
        //yScaled = ((mY_ * gainFactor_) - yOffset_) * yGainError_;
        //zScaled = ((mZ_ * gainFactor_) - zOffset_) * zGainError_;
	

}

void poll(){

	Wire.beginTransmission(compass_address);
	Wire.write(0x02);
	Wire.write(0b10000001);
	// Writing the register value 0000 0000 for continous mode
	// Writing the register value 0000 0001 for single
	Wire.endTransmission();
	Wire.requestFrom(compass_address, 6);

	if (6 <= Wire.available()){

		mX_ = Wire.read() << 8 | Wire.read();
		mZ_ = Wire.read() << 8 | Wire.read(); //Yeah, that's intuitive
		mY_ = Wire.read() << 8 | Wire.read();

	}
}
