#ifndef H2_PID_H
#define H2_PID_H

//#include "Arduino.h"

class H2_PID{
public:
	H2_PID();
	H2_PID(double kp, double ki, double kd, int maximum, int minimum, int iMax);

	void calc(double error, double sensor, double cycle);
	void updateDefaults(double kp, double ki, double kd, double iTerm, int maximum, int minimum, int iMax);
	void updateGain(double kp, double ki, double kd);
	void updateBounds(int maximum, int minimum, int iMax);
	void setIntegral(double newITerm);

	void calcPError();
	void calcIError();
	void calcDError();

	int getControl();

private:
	double error_;
	double sensor_;
	double pError_;
	double iError_;
	double dError_;
	double kp_;
	double ki_;
	double kd_;
	double dLast_;
	double cycle_;

	int control_;
	int iMax_;
	int maximum_;
	int minimum_;

	unsigned long tLast_;

};

struct PID_REGISTER{
	//Manual Stability PID's
	H2_PID rsPID;
	H2_PID psPID;
	H2_PID ysPID;
	
	//Manual Rate PID's
	H2_PID rrPID;
	H2_PID prPID;
	H2_PID yrPID;

	//Auto PID's
	H2_PID arPID;
	H2_PID apPID;
	H2_PID atPID;

};
#endif