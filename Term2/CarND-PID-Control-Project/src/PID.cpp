#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	p_error = d_error = i_error = 0.0;

}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	double output = -(Kp * p_error +
		Ki * i_error +
		Kd * d_error);

	if (output > maxSteering)
	{
		std::cout << "Too large steering output: " << output << std::endl;
		output = maxSteering;
	}
	else if (output < minSteering)
	{
		std::cout << "Too large steering output: " << output << std::endl;
		output = minSteering;
	}

	return output;
}


