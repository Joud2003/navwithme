from math import pi
import numpy as np
from nav_with_me.pid_controller import PIDController


class MotionController:
    def __init__(self, robot):
        self.pd_x = PIDController(1.0, 0.1, 1.0)
        self.pd_theta = PIDController(1.0, 0.5, 0.0)
        self.robot = robot
        self.sweep_direction = -1  # 1 = left, -1 = right
        self.state = "FORWARD"
        self.lane_width = 1.0  # lateral shift distance
        self.forward_speed = 0.3
        self.wall_threshold = 0.5
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_theta = 0.0

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def handleControl(self, robot, msg):
        front_dist = np.mean(robot.front_readings)
        self.pd_x.setPoint(front_dist)
        if front_dist <= self.wall_threshold:
            msg.linear.x = 0.0
            robot.vel_pub.publish(msg)
            robot.node.get_logger().warn(
                f"Obstacle detected within {front_dist:.2f}m, stopping"
            )
        elif self.state == "FORWARD":
            # robot.node.get_logger().info("Forward state")

            if front_dist > self.wall_threshold:
                linear_v = self.pd_x.update(self.wall_threshold)
                if abs(linear_v) > self.forward_speed:
                    linear_v = self.forward_speed * np.sign(linear_v)
                msg.linear.x = abs(linear_v)
                msg.angular.z = 0.0
                robot.vel_pub.publish(msg)

            else:
                msg.linear.x = 0.0
                robot.vel_pub.publish(msg)

                self.target_theta = self.normalize_angle(
                    robot.pose.theta + self.sweep_direction * (pi / 2)
                )
                self.pd_theta.setPoint(self.target_theta)
                # robot.node.get_logger().info("Now switching to first turn state")
                self.state = "TURN_1"

        elif self.state == "TURN_1":
            # robot.node.get_logger().info("First turn state")
            error = self.normalize_angle(self.target_theta - robot.pose.theta)
            if abs(error) > 0.05:
                msg.angular.z = max(
                    min(self.pd_theta.update(robot.pose.theta), 0.3), -0.2
                )
                msg.linear.x = 0.0
                robot.vel_pub.publish(msg)

            else:
                msg.angular.z = 0.0
                robot.vel_pub.publish(msg)
                self.start_x = robot.pose.x
                self.start_y = robot.pose.y
                self.pd_x.setPoint(robot.pose.x + self.lane_width)
                # robot.node.get_logger().info(f"Now switching to shift state")
                self.state = "SHIFT"

        elif self.state == "SHIFT":
            # robot.node.get_logger().info("Shift state")
            dx = robot.pose.x - self.start_x
            dy = robot.pose.y - self.start_y
            dist = np.sqrt(dx**2 + dy**2)
            # robot.node.get_logger().info(f"error is {self.lane_width - dist}")
            error = self.lane_width - dist
            if abs(error) > 0.05:
                linear_v = self.pd_x.update(dist)
                if abs(linear_v) > self.forward_speed:
                    linear_v = self.forward_speed * np.sign(linear_v)

                msg.linear.x = abs(linear_v)
                msg.angular.z = 0.0
                robot.vel_pub.publish(msg)

            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                robot.vel_pub.publish(msg)
                self.target_theta = -1 * self.normalize_angle(
                    robot.pose.theta + self.sweep_direction * (pi / 2)
                )
                self.pd_theta.setPoint(self.target_theta)
                # robot.node.get_logger().info("Now switching to second turn state")

                self.state = "TURN_2"

        elif self.state == "TURN_2":
            # robot.node.get_logger().info("Second turn state")

            error = self.normalize_angle(self.target_theta - robot.pose.theta)
            if abs(error) > 0.05:
                msg.angular.z = max(
                    min(self.pd_theta.update(robot.pose.theta), 0.3), -0.2
                )
                msg.linear.x = 0.0
                robot.vel_pub.publish(msg)
            else:
                msg.angular.z = 0.0
                robot.vel_pub.publish(msg)
                self.sweep_direction *= -1
                self.state = "FORWARD"
                # robot.node.get_logger().info(
                #     f"Completed one sweep, switching back to forward"
                # )
        else:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            robot.vel_pub.publish(msg)
            robot.node.get_logger().info(f"Idle state")
