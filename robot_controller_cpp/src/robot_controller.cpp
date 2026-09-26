#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <limits>

class RobotController : public rclcpp::Node
{
public:
    RobotController()
        : Node("robot_controller")
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
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
            float front_readings = std::numeric_limits<float>::infinity();
            constexpr float front_angle = 0.2617994f;

            for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
                const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
                const float range = msg->ranges[i];
                if (std::abs(angle) <= front_angle && std::isfinite(range) &&
                    range >= msg->range_min && range <= msg->range_max)
                {
                    front_readings = std::min(front_readings, range);
                }
            }

      geometry_msgs::msg::Twist cmd_vel;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Front LIDAR reading: %.2f", front_readings);
            if (front_readings <= 0.2f) {
        cmd_vel.linear.x = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obstacle detected! Stopping the robot.");
      } else {
        cmd_vel.linear.x = 0.2;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path is clear. Moving forward.");
      }
      cmd_vel_publisher_->publish(cmd_vel);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}