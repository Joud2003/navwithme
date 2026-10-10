#include "robot_controller_cpp/motion_controller.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTurnTolerance = 0.05;
constexpr double kShiftTolerance = 0.05;
}

MotionController::MotionController()
: distance_controller_(1.0, 0.1, 1.0),
  heading_controller_(1.0, 0.5, 0.0),
  sweep_direction_(-1),
  state_(State::Forward),
  lane_width_(1.0),
  forward_speed_(0.3),
  wall_threshold_(0.5),
  start_x_(0.0),
  start_y_(0.0),
  target_theta_(0.0),
  total_lanes_(5),
  current_lane_(1),
  exploration_complete_(false)
{}

geometry_msgs::msg::Twist MotionController::handleControl(
  double front_distance, double x, double y, double theta)
{
  geometry_msgs::msg::Twist command;
  if (!std::isfinite(front_distance) || !std::isfinite(x) ||
    !std::isfinite(y) || !std::isfinite(theta))
  {
    return command;
  }

  if (current_lane_ >= total_lanes_) {
    exploration_complete_ = true;
    std::cout << "Exploration complete. Total lanes covered: " << current_lane_ << std::endl;
    return command;
  }

  switch (state_) {
    case State::Forward:
      if (front_distance > wall_threshold_) {
        distance_controller_.setPoint(front_distance);
        const double speed = clamp(
          distance_controller_.update(wall_threshold_), -forward_speed_, forward_speed_);
        command.linear.x = std::abs(speed);
        command.angular.z = 0.0;
      } else {
        command.linear.x = 0.0;
        target_theta_ = normalizeAngle(theta + sweep_direction_ * (kPi / 2.0));
        heading_controller_.setPoint(target_theta_);
        state_ = State::Turn1;
      }
      break;

    case State::Turn1: {
      const double error = normalizeAngle(target_theta_ - theta);
      if (std::abs(error) > kTurnTolerance) {
        command.angular.z = clamp(heading_controller_.update(theta), -0.2, 0.3);
        command.linear.x = 0.0;
      } else {
        start_x_ = x;
        start_y_ = y;
        distance_controller_.setPoint(lane_width_);
        state_ = State::Shift;
      }
      break;
    }

    case State::Shift: {
      const double dx = x - start_x_;
      const double dy = y - start_y_;
      const double distance = std::hypot(dx, dy);
      const double error = lane_width_ - distance;
      if (std::abs(error) > kShiftTolerance) {
        const double speed = clamp(
          distance_controller_.update(distance), -forward_speed_, forward_speed_);
        command.linear.x = std::abs(speed);
        command.angular.z = 0.0;
      } else {
        command.linear.x = 0.0;
        command.angular.z = 0.0;
        target_theta_ = normalizeAngle(theta + sweep_direction_ * (kPi / 2.0));
        heading_controller_.setPoint(target_theta_);
        state_ = State::Turn2;
      }
      break;
    }

    case State::Turn2: {
      const double error = normalizeAngle(target_theta_ - theta);
      if (std::abs(error) > kTurnTolerance) {
        command.angular.z = clamp(heading_controller_.update(theta), -0.2, 0.3);
        command.linear.x = 0.0;
      } else {
        current_lane_++;
        command.angular.z = 0.0;
        sweep_direction_ *= -1;
        state_ = State::Forward;
      }
      break;
    }
    default:
      command.linear.x = 0.0;
      command.angular.z = 0.0;
      break;
  }

  return command;
}

bool MotionController::explorationComplete() const
{
  return exploration_complete_;
}

double MotionController::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  } 
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double MotionController::clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}