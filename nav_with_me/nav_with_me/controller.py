from math import pi
import numpy as np


def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


def handleControl(robot, msg):
    front_dist = np.mean(robot.front_readings)
    robot.pd_x.setPoint(front_dist)
    if front_dist < robot.wall_threshold:
        msg.linear.x = 0.0
        robot.vel_pub.publish(msg)
    if robot.state == "FORWARD":
        # robot.node.get_logger().info("Forward state")

        if front_dist > robot.wall_threshold:
            linear_v = robot.pd_x.update(robot.wall_threshold)
            if abs(linear_v) > robot.forward_speed:
                linear_v = robot.forward_speed * np.sign(linear_v)
            msg.linear.x = abs(linear_v)
            msg.angular.z = 0.0
            robot.vel_pub.publish(msg)

        else:
            msg.linear.x = 0.0
            robot.vel_pub.publish(msg)

            robot.target_theta = normalize_angle(
                robot.pose.theta + robot.sweep_direction * (pi / 2)
            )
            robot.pd_theta.setPoint(robot.target_theta)
            # robot.node.get_logger().info("Now switching to first turn state")
            robot.state = "TURN_1"

    elif robot.state == "TURN_1":
        # robot.node.get_logger().info("First turn state")
        error = normalize_angle(robot.target_theta - robot.pose.theta)
        if abs(error) > 0.05:
            msg.angular.z = max(min(robot.pd_theta.update(robot.pose.theta), 0.3), -0.2)
            msg.linear.x = 0.0
            robot.vel_pub.publish(msg)

        else:
            msg.angular.z = 0.0
            robot.vel_pub.publish(msg)
            robot.start_x = robot.pose.x
            robot.start_y = robot.pose.y
            robot.pd_x.setPoint(robot.pose.x + robot.lane_width)
            # robot.node.get_logger().info(f"Now switching to shift state")
            robot.state = "SHIFT"

    elif robot.state == "SHIFT":
        # robot.node.get_logger().info("Shift state")
        dx = robot.pose.x - robot.start_x
        dy = robot.pose.y - robot.start_y
        dist = np.sqrt(dx**2 + dy**2)

        error = robot.lane_width - dist
        if abs(error) > 0.05:
            linear_v = robot.pd_x.update(dist)
            if abs(linear_v) > robot.forward_speed:
                linear_v = robot.forward_speed * np.sign(linear_v)

            msg.linear.x = abs(linear_v)
            msg.angular.z = 0.0
            robot.vel_pub.publish(msg)

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            robot.vel_pub.publish(msg)
            robot.target_theta = -1 * normalize_angle(
                robot.pose.theta + robot.sweep_direction * (pi / 2)
            )
            robot.pd_theta.setPoint(robot.target_theta)
            # robot.node.get_logger().info("Now switching to second turn state")

            robot.state = "TURN_2"

    elif robot.state == "TURN_2":
        # robot.node.get_logger().info("Second turn state")

        error = normalize_angle(robot.target_theta - robot.pose.theta)
        if abs(error) > 0.05:
            msg.angular.z = max(min(robot.pd_theta.update(robot.pose.theta), 0.3), -0.2)
            msg.linear.x = 0.0
            robot.vel_pub.publish(msg)
        else:
            msg.angular.z = 0.0
            robot.vel_pub.publish(msg)
            robot.sweep_direction *= -1
            robot.state = "FORWARD"
            # robot.node.get_logger().info(
            #     f"Completed one sweep, switching back to forward"
            # )
    else:
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        robot.vel_pub.publish(msg)
        robot.node.get_logger().info(f"Idle state")
