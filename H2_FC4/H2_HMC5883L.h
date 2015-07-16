
#ifndef H2_HMC5883L_h
#define H2_HMC5883L_h
#include "H2_HMC5883L.h"
#include "Arduino.h"

class HMC5883L{
	public:
		//Quad_HMC5883L();
		void init(double xGainError, double yGainError, double zGainError, double xOffset, double yOffset, double zOffset);
		void update();
		void poll();

		double xScaled;
		double yScaled;
		double zScaled;

	private:
		double gainFactor_;
		double xGainError_, yGainError_, zGainError_;
		double xOffset_, yOffset_, zOffset_;
		int mX_, mY_, mZ_;
};


#endif