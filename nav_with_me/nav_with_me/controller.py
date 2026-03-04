from math import pi, atan2
import numpy as np
import threading
import rclpy
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from .utils import PIDController


class Turtlebot3:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("turtlebot3_move_square")
        self.node.get_logger().info("Pass Ctrl + C to terminate")
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.rate = self.node.create_rate(1)

        t = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        t.start()
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = self.node.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.lidar_sub = self.node.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.pd_x = PIDController(1.0, 0.1, 1.0)
        self.pd_theta = PIDController(1.0, 0.1, 0.3)
        self.front_readings = [4.0]
        self.state = "FORWARD"
        self.sweep_direction = -1  # 1 = left, -1 = right
        self.lane_width = 0.2  # lateral shift distance
        self.forward_speed = 0.3
        self.turn_speed = 0.3
        self.wall_threshold = 0.5

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.target_theta = 0.0

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def run(self):
        msg = Twist()
        while rclpy.ok():
            front_dist = np.mean(self.front_readings)
            if self.state == "FORWARD":
                if front_dist > self.wall_threshold:
                    linear_v = self.pd_x.update(self.wall_threshold + 0.2)
                    self.node.get_logger().info(
                        f"Front distance: {front_dist:.2f}, Linear velocity: {linear_v:.2f}"
                    )
                    if abs(linear_v) > self.forward_speed:
                        linear_v = self.forward_speed * np.sign(linear_v)
                    msg.linear.x = abs(linear_v)
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)

                else:
                    msg.linear.x = 0.0
                    self.vel_pub.publish(msg)

                    self.target_theta = self.normalize_angle(
                        self.pose.theta + self.sweep_direction * (pi / 2)
                    )
                    self.pd_theta.setPoint(self.target_theta)
                    self.node.get_logger().info(
                        f"Front distance: {front_dist:.2f}, Target theta: {self.target_theta:.2f}"
                    )
                    self.state = "TURN_1"

            elif self.state == "TURN_1":

                error = self.normalize_angle(self.target_theta - self.pose.theta)
                self.node.get_logger().info(f"Turning 1 error: {error:.2f}")
                if abs(error) > 0.05:
                    msg.angular.z = max(
                        min(self.pd_theta.update(self.pose.theta), 0.6), -0.6
                    )
                    msg.linear.x = 0.0
                    self.vel_pub.publish(msg)

                else:
                    msg.angular.z = 0.0
                    self.start_x = self.pose.x
                    self.start_y = self.pose.y
                    self.pd_x.setPoint(self.pose.x + self.lane_width)
                    self.node.get_logger().info(f"Now switching to shift state")

                    self.state = "SHIFT"
                    self.vel_pub.publish(msg)

            elif self.state == "SHIFT":
                dx = self.pose.x - self.start_x
                dy = self.pose.y - self.start_y
                dist = np.sqrt(dx**2 + dy**2)

                error = self.lane_width - dist

                if abs(error) > 0.05:
                    linear_v = self.pd_x.update(dist)
                    if abs(linear_v) > self.forward_speed:
                        linear_v = self.forward_speed * np.sign(linear_v)
                    self.node.get_logger().info(f"Shifting, velocity: {linear_v:.2f}")

                    msg.linear.x = abs(linear_v)
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)

                else:
                    msg.linear.x = 0.0
                    self.vel_pub.publish(msg)
                    self.target_theta = self.normalize_angle(
                        self.pose.theta + self.sweep_direction * (pi / 2)
                    )
                    self.pd_theta.setPoint(self.target_theta)
                    self.state = "TURN_2"

            elif self.state == "TURN_2":

                error = self.normalize_angle(self.target_theta - self.pose.theta)
                self.node.get_logger().info(
                    f"Last turn, distance: {error:.2f}, target: {self.target_theta:.2f} current: {self.pose.theta:.2f}  "
                )

                if abs(error) > 0.05:
                    msg.angular.z = max(
                        min(self.pd_theta.update(self.pose.theta), 0.6), -0.2
                    )
                    msg.linear.x = 0.0
                    self.vel_pub.publish(msg)
                else:
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)
                    self.sweep_direction *= -1
                    self.state = "FORWARD"
                    self.node.get_logger().info(
                        f"Completed one sweep, switching back to forward"
                    )

        self.trajectory.append([self.pose.x, self.pose.y])

    def odom_callback(self, msg):
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        (_, _, yaw) = euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x

    def scan_callback(self, msg):
        samples_20_deg = int(0.349 / msg.angle_increment)
        front_left = msg.ranges[0:samples_20_deg]
        front_right = msg.ranges[-samples_20_deg:]

        self.front_readings.append(min(front_left + front_right))
        if len(self.front_readings) > 5:
            self.front_readings.pop(0)


def main(args=None):
    data = []
    for _ in range(10):
        turtlebot = Turtlebot3()
        turtlebot.i = 0
        turtlebot.node.get_logger().info("Main is called, node is created")
        turtlebot.run()
        data.append(turtlebot.trajectory)
        turtlebot.node.destroy_node()
        rclpy.shutdown()
    np.savetxt("trajectory.csv", np.array(data), delimiter=",")


if __name__ == "__main__":
    main()
