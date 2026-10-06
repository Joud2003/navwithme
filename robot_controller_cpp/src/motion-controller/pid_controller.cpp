#include "robot_controller_cpp/pid_controller.hpp"

PIDController::PIDController(
	double proportional_gain,
	double derivative_gain,
	double set_point)
: proportional_gain_(proportional_gain),
	derivative_gain_(derivative_gain),
	set_point_(set_point),
	previous_error_(0.0)
{}

double PIDController::update(double current_value)
{
	const double error = set_point_ - current_value;
	const double proportional_term = proportional_gain_ * error;
	const double derivative_term = derivative_gain_ * (error - previous_error_);
	previous_error_ = error;
	return proportional_term + derivative_term;
}

void PIDController::setPoint(double set_point)
{
	set_point_ = set_point;
	previous_error_ = 0.0;
}

void PIDController::setPD(double proportional_gain, double derivative_gain)
{
	proportional_gain_ = proportional_gain;
	derivative_gain_ = derivative_gain;
}
