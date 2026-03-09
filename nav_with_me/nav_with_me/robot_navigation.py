import os
import numpy as np
import threading
from .controller import handleControl
import rclpy
from tf_transformations import euler_from_quaternion
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
        self.pd_theta = PIDController(1.0, 0.5, 0.0)
        self.front_readings = [4.0]
        self.state = "FORWARD"
        self.sweep_direction = -1  # 1 = left, -1 = right
        self.lane_width = 1.0  # lateral shift distance
        self.forward_speed = 0.3
        self.turn_speed = 0.3
        self.wall_threshold = 0.5

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.target_theta = 0.0
        self.trajectory = list()

    def run(self):
        msg = Twist()
        while rclpy.ok():
            handleControl(self, msg)

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
        self.pose.y = msg.pose.pose.position.y
        self.trajectory.append([self.pose.x, self.pose.y])

    def scan_callback(self, msg):
        samples_20_deg = int(0.349 / msg.angle_increment)
        front_left = msg.ranges[0:samples_20_deg]
        front_right = msg.ranges[-samples_20_deg:]

        self.front_readings.append(min(front_left + front_right))
        if len(self.front_readings) > 5:
            self.front_readings.pop(0)


def main(args=None):
    turtlebot = None
    data = []
    folder = "robot_data"
    os.makedirs(folder, exist_ok=True)

    try:
        turtlebot = Turtlebot3()
        turtlebot.i = 0
        turtlebot.node.get_logger().info("Main is called, node is created")
        turtlebot.run()  # blocking

    except KeyboardInterrupt:
        if turtlebot is not None:
            data.append(turtlebot.trajectory)  # append trajectory now
            np.savetxt(
                os.path.join(folder, "trajectory.csv"),
                np.vstack(data),
                delimiter=",",
            )
            print(f"Trajectory saved to {folder}/trajectory.csv")

    finally:
        if turtlebot is not None:
            turtlebot.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
