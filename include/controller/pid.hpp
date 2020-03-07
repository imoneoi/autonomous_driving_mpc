#ifndef __CAR_MODEL_CONTROLLER_PID_H__
#define __CAR_MODEL_CONTROLLER_PID_H__

#include "ros/ros.h"

struct ErrorList
{
    double value[3];

	ErrorList() {
		value[0] = value[1] = value[2] = 0;
	}
};

class PID {
private:
	ErrorList error_;
	ErrorList error_filtered_;
	ErrorList error_derivative_;
	ErrorList error_derivative_filtered_;

	double kp_, ki_, kd_, maxi_, outrange_;

	double _error_int_;

	ros::Time last_looptime_;

public:
	PID();

	void setCoefficients(double kp, double ki, double kd, double maxi, double outrange);
	double update(double input, double setpoint);
};

PID::PID() {
	kp_ = 0;
	ki_ = 0;
	kd_ = 0;
	maxi_ = 10;
	outrange_ = 1;

	_error_int_ = 0;

	last_looptime_ = ros::Time(0);
}

void PID::setCoefficients(double kp, double ki, double kd, double maxi, double outrange) {
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
	maxi_ = maxi;
	outrange_ = outrange;
}

double PID::update(double input, double setpoint) {
	if (last_looptime_.isZero())
	{
		//Return at zero start time
		last_looptime_ = ros::Time::now();
		return 0.0;
	}

	//get Time
	double dt = (ros::Time::now() - last_looptime_).toSec();
	last_looptime_ = ros::Time::now();

	if (!dt)
	{
		//Return at zero delta time
		return 0.0;
	}

	//PID Closeloop
	//Reference: https://bitbucket.org/AndyZe/pid
	double c_ = 1.0;

	//Calculate Error
	double error = setpoint - input;

	error_.value[2] = error_.value[1];
	error_.value[1] = error_.value[0];
	error_.value[0] = error;

	//intergrate error
	_error_int_ += error_.value[0] * dt;
	if (_error_int_ > maxi_ ) _error_int_ = maxi_;
	if (_error_int_ < -maxi_) _error_int_ = -maxi_;

	error_filtered_.value[2] = error_filtered_.value[1];
	error_filtered_.value[1] = error_filtered_.value[0];
	error_filtered_.value[0] = (1 / (1 + c_ * c_ + 1.414 * c_)) * (error_.value[2] + 2 * error_.value[1] + error_.value[0] -
																   (c_ * c_ - 1.414 * c_ + 1) * error_filtered_.value[2] -
																   (-2 * c_ * c_ + 2) * error_filtered_.value[1]);

	//calculate error derivative
	error_derivative_.value[2] = error_derivative_.value[1];
	error_derivative_.value[1] = error_derivative_.value[0];
	error_derivative_.value[0] = (error_.value[0] - error_.value[1]) / dt;

	//filter error derivative
	error_derivative_filtered_.value[2] = error_derivative_filtered_.value[1];
	error_derivative_filtered_.value[1] = error_derivative_filtered_.value[0];

	error_derivative_filtered_.value[0] =
		(1 / (1 + c_ * c_ + 1.414 * c_)) *
		(error_derivative_.value[2] + 2 * error_derivative_.value[1] + error_derivative_.value[0] -
		 (c_ * c_ - 1.414 * c_ + 1) * error_derivative_filtered_.value[2] - (-2 * c_ * c_ + 2) * error_derivative_filtered_.value[1]);

	//output power
	double output = kp_ * error_filtered_.value[0] + ki_ * _error_int_ + kd_ * error_derivative_filtered_.value[0];

	//limit output
	if(output > outrange_ ) output = outrange_;
	if(output < -outrange_) output = -outrange_;
	return output;
}

#endif