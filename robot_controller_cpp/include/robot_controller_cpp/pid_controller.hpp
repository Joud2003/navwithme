#pragma once

class PIDController
{
public:
  explicit PIDController(double proportional_gain = 0.0,
    double derivative_gain = 0.0, double set_point = 0.0);

  double update(double current_value);
  void setPoint(double set_point);
  void setPD(double proportional_gain = 0.0, double derivative_gain = 0.0);

private:
  double proportional_gain_;
  double derivative_gain_;
  double set_point_;
  double previous_error_;
};