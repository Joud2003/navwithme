from math import pi, atan2
import numpy as np
import threading
import rclpy
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan


class PIDController:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point
        self.previous_error = 0

    def update(self, current_value):
        # Calculate the new error value = desired - real
        error = self.set_point - current_value
        # choosing P value = 1
        P_term = self.Kp * error
        # Choosing D value to be between 0 and 1
        D_term = self.Kd * (error - self.previous_error)
        # Updating error
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D


class Turtlebot3:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("turtlebot3_move_square")
        self.node.get_logger().info("Pass Ctrl + C to terminate")
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.rate = self.node.create_rate(10)

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
        self.pd = PIDController(1.0, 0.1, 0.0)
        self.i = 0
        self.front_readings = [4.0]

    def run(self):
        msg = Twist()
        while rclpy.ok():
            x_diff = 0.0
            y_diff = 0.4

            if np.mean(self.front_readings) <= 0.3:
                msg.linear.x = 0.0
                theta_desired = atan2((y_diff), (x_diff))
                if theta_desired - self.pose.theta > pi:
                    theta_desired -= 2 * pi
                elif theta_desired - self.pose.theta < -pi:
                    theta_desired += 2 * pi
                self.pd.setPoint(theta_desired)

                new_theta = self.pd.update(self.pose.theta)
                msg.linear.x = 0.2
                msg.angular.z = new_theta
                self.vel_pub.publish(msg)
            else:
                x = self.pd.update(np.mean(self.front_readings))
                msg.linear.x = abs(x)
                self.vel_pub.publish(msg)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
        self.trajectory.append([self.pose.x, self.pose.y])
        self.node.get_logger().info(
            "odom: x="
            + str(self.pose.x)
            + "y="
            + str(self.pose.y)
            + "; theta="
            + str(self.pose.theta)
        )

    def odom_callback(self, msg):
        print(msg.pose.pose)
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

    def scan_callback(self, msg):
        self.front = msg.ranges[0]
        self.front_readings.append(self.front)
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
