import os
from matplotlib import pyplot as plt
import numpy as np
import threading
from .controller import handleControl
import rclpy
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from .utils import PIDController
from tf2_ros import Buffer, TransformListener


class Turtlebot3:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("turtlebot3_move_square")
        self.node.get_logger().info("Pass Ctrl + C to terminate")
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.rate = self.node.create_rate(1)
        self.timer = self.node.create_timer(0.1, self.update_pose)  # 10 Hz

        t = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        t.start()
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.lidar_sub = self.node.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10
        )
        self.map_data = None
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
        self.resolution = 0.0
        self.trajectory = list()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def run(self):
        msg = Twist()
        while rclpy.ok():
            handleControl(self, msg)
            self.rate.sleep()

    def update_pose(self):
        if not self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time()):
            self.node.get_logger().warn("Transform not available yet")
            return

        transform = self.tf_buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time()  # target frame  # robot frame
        )
        self.pose.x = transform.transform.translation.x
        self.pose.y = transform.transform.translation.y
        self.pose.theta = euler_from_quaternion(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        )[2]
        self.node.get_logger().info(
            f"Pose from slam is x: {self.pose.x}, y: {self.pose.y}, theta: {self.pose.theta}"
        )
        self.trajectory.append([self.pose.x, self.pose.y])

    def scan_callback(self, msg):
        samples_20_deg = int(0.349 / msg.angle_increment)
        front_left = msg.ranges[0:samples_20_deg]
        front_right = msg.ranges[-samples_20_deg:]

        self.front_readings.append(min(front_left + front_right))
        if len(self.front_readings) > 5:
            self.front_readings.pop(0)

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution

        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

        self.node.get_logger().info(
            f"Map received {self.map_width} x {self.map_height}"
        )
        transform = self.tf_buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time()  # target frame  # robot frame
        )


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
        np.savez(
            os.path.join(folder, "occupancy_grid_map.npz"),
            grid=turtlebot.map_data,
            resolution=turtlebot.resolution,
            origin_x=turtlebot.origin_x,
            origin_y=turtlebot.origin_y,
        )
        rclpy.shutdown()


if __name__ == "__main__":
    main()
