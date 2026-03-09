from math import pi

import numpy as np

import rclpy


def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


def handleControl(self, msg):
    front_dist = np.mean(self.front_readings)
    self.pd_x.setPoint(front_dist)
    if front_dist < self.wall_threshold:
        msg.linear.x = 0.0
        self.vel_pub.publish(msg)
    if self.state == "FORWARD":
        self.node.get_logger().info("Forward state")

        if front_dist > self.wall_threshold:
            linear_v = self.pd_x.update(self.wall_threshold)
            if abs(linear_v) > self.forward_speed:
                linear_v = self.forward_speed * np.sign(linear_v)
            msg.linear.x = abs(linear_v)
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)

        else:
            msg.linear.x = 0.0
            self.vel_pub.publish(msg)

            self.target_theta = normalize_angle(
                self.pose.theta + self.sweep_direction * (pi / 2)
            )
            self.pd_theta.setPoint(self.target_theta)
            self.node.get_logger().info("Now switching to first turn state")
            self.state = "TURN_1"

    elif self.state == "TURN_1":
        self.node.get_logger().info("First turn state")
        error = normalize_angle(self.target_theta - self.pose.theta)
        if abs(error) > 0.05:
            msg.angular.z = max(min(self.pd_theta.update(self.pose.theta), 0.3), -0.2)
            msg.linear.x = 0.0
            self.vel_pub.publish(msg)

        else:
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            self.start_x = self.pose.x
            self.start_y = self.pose.y
            self.pd_x.setPoint(self.pose.x + self.lane_width)
            self.node.get_logger().info(f"Now switching to shift state")
            self.state = "SHIFT"

    elif self.state == "SHIFT":
        self.node.get_logger().info("Shift state")
        dx = self.pose.x - self.start_x
        dy = self.pose.y - self.start_y
        dist = np.sqrt(dx**2 + dy**2)

        error = self.lane_width - dist
        if abs(error) > 0.05:
            linear_v = self.pd_x.update(dist)
            if abs(linear_v) > self.forward_speed:
                linear_v = self.forward_speed * np.sign(linear_v)

            msg.linear.x = abs(linear_v)
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            self.target_theta = -1 * normalize_angle(
                self.pose.theta + self.sweep_direction * (pi / 2)
            )
            self.pd_theta.setPoint(self.target_theta)
            self.node.get_logger().info("Now switching to second turn state")

            self.state = "TURN_2"

    elif self.state == "TURN_2":
        self.node.get_logger().info("Second turn state")

        error = normalize_angle(self.target_theta - self.pose.theta)
        if abs(error) > 0.05:
            msg.angular.z = max(min(self.pd_theta.update(self.pose.theta), 0.3), -0.2)
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
    else:
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        self.vel_pub.publish(msg)
        self.node.get_logger().info(f"Idle state")
