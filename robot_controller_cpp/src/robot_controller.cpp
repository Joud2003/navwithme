#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_controller_cpp/motion_controller.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

class RobotController : public rclcpp::Node
{
public:
    RobotController()
        : Node("robot_controller"),
          pose_x_(0.0),
          pose_y_(0.0),
          pose_theta_(0.0),
          pose_received_(false),
          completion_logged_(false)
    {
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(
                &RobotController::lidar_callback,
                this,
                std::placeholders::_1
            )
        );
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(
                &RobotController::odometry_callback,
                this,
                std::placeholders::_1
            )
        );
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pose_x_ = msg->pose.pose.position.x;
        pose_y_ = msg->pose.pose.position.y;

        const auto & orientation = msg->pose.pose.orientation;
        pose_theta_ = std::atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        );
        pose_received_ = true;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float front_readings = std::numeric_limits<float>::infinity();
        constexpr float front_angle = 0.2617994f;
        bool has_no_return = false;

        for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
            const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            const float range = msg->ranges[i];
            if (std::abs(angle) <= front_angle) {
                if (std::isinf(range) && range > 0.0f) {
                    has_no_return = true;
                } else if (std::isfinite(range) &&
                    range >= msg->range_min && range <= msg->range_max)
                {
                    front_readings = std::min(front_readings, range);
                }
            }
        }
        if (std::isinf(front_readings) && has_no_return) {
            front_readings = msg->range_max;
        }

        geometry_msgs::msg::Twist cmd_vel;
        if (pose_received_) {
            cmd_vel = motion_controller_.handleControl(
                front_readings, pose_x_, pose_y_, pose_theta_);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Front lidar: %.2f m, command: linear=%.2f angular=%.2f",
            front_readings,
            cmd_vel.linear.x,
            cmd_vel.angular.z
        );
        if (motion_controller_.explorationComplete() && !completion_logged_) {
            RCLCPP_INFO(this->get_logger(), "Exploration complete");
            completion_logged_ = true;
        }
        cmd_vel_publisher_->publish(cmd_vel);
    }

    MotionController motion_controller_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    double pose_x_;
    double pose_y_;
    double pose_theta_;
    bool pose_received_;
    bool completion_logged_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}