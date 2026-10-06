#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "robot_controller_cpp/pid_controller.hpp"

class MotionController
{
public:
  MotionController();

  geometry_msgs::msg::Twist handleControl(
    double front_distance, double x, double y, double theta);
  bool explorationComplete() const;

private:
  enum class State
  {
    Forward,
    Turn1,
    Shift,
    Turn2
  };

  static double normalizeAngle(double angle);
  static double clamp(double value, double lower, double upper);

  PIDController distance_controller_;
  PIDController heading_controller_;
  int sweep_direction_;
  State state_;
  double lane_width_;
  double forward_speed_;
  double wall_threshold_;
  double start_x_;
  double start_y_;
  double target_theta_;
  int total_lanes_;
  int current_lane_;
  bool exploration_complete_;
};