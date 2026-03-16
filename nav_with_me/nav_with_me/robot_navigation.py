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
        self.trajectory = list()
        plt.ion()
        self.fig, self.ax = plt.subplots()

    def run(self):
        msg = Twist()
        while rclpy.ok():
            handleControl(self, msg)
            self.plot_map()   
            self.rate.sleep()

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

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution

        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.map_data = np.array(msg.data).reshape(
            (self.map_height, self.map_width)
        )

        self.node.get_logger().info(
            f"Map received {self.map_width} x {self.map_height}"
        )

    def plot_map(self):

        if self.map_data is None:
            return

        grid = self.map_data.copy()

        grid_plot = np.zeros_like(grid)

        grid_plot[grid == -1] = 127
        grid_plot[grid == 0] = 255
        grid_plot[grid == 100] = 0

        self.ax.clear()

        self.ax.imshow(
            grid_plot,
            cmap="gray",
            origin="lower"
        )

        x = int((self.pose.x - self.origin_x) / self.resolution)
        y = int((self.pose.y - self.origin_y) / self.resolution)

        self.ax.scatter(x, y, c="red", s=40)

        self.ax.set_title("Occupancy Grid Map")

        plt.pause(0.001)


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
            grid = np.array(turtlebot.map_data).reshape(
                (turtlebot.map_height, turtlebot.map_width)
            )
            plt.clf()

            plt.imshow(grid, cmap="gray", origin="lower")

            plt.title("Occupancy Grid Map")

            plt.pause(0.01)

    finally:
        if turtlebot is not None:
            turtlebot.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
