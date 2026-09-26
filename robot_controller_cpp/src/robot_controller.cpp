#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello World! Received LiDAR scan.");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}