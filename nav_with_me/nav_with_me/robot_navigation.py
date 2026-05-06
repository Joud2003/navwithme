import os
import numpy as np
import cv2
import threading
from ultralytics import YOLO
from .object_detection import ObjectDetection
from .motion_controller import MotionController
import rclpy
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from tf2_ros import Buffer, TransformListener
from .img_object_detection import ImageProcessor
import queue


class Turtlebot3:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("turtlebot3_move_square")
        self.node.get_logger().info("Pass Ctrl + C to terminate")
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.rate = self.node.create_rate(1)
        self.timer = self.node.create_timer(0.1, self.update_pose)  # 10 Hz
        self.timer_2 = self.node.create_timer(0.1, self.log_row)
        self.controller = MotionController(self)
        self.object_detection = ObjectDetection(self)
        self.image_processor = ImageProcessor(self)
        t = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        t.start()
        self.lidar_sub = self.node.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.img_sub = self.node.create_subscription(
            Image, "camera/image_raw", self.image_callback, 10
        )
        self.pose = Pose2D()
        self.ground_truth_pose = None
        self.estimated_pose = None
        self.trajectory = list()
        self.ground_truth_trajectory = list()
        self.map_data = None
        self.readings = {"front": [4.0], "right": [4.0], "left": [4.0]}
        self.object_is_detected = False
        self.resolution = 0.0
        self.trajectory = list()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.img_height = None
        self.img_width = None
        self.img_encoding = None
        self.detected_objects = []  # Store detected objects with timestamps and poses
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.image_queue = queue.Queue(maxsize=50)

        # YOLO
        self.yolo_running = True
        self.yolo_model = None
        yolo_thread = threading.Thread(target=self._yolo_worker, daemon=True)
        yolo_thread.start()

    def run(self):
        msg = Twist()
        while rclpy.ok():
            self.controller.handleControl(self, msg)
            self.rate.sleep()

    def _yolo_worker(self):
        self.node.get_logger().info("Starting YOLO worker thread")

        try:
            self.yolo_model = YOLO("yolov8n.pt")
            self.node.get_logger().info("YOLO model loaded successfully")
        except Exception as e:
            self.node.get_logger().error(f"Failed to load YOLO model: {e}")
            return

        while self.yolo_running:
            if not self.image_queue.qsize() > 0:
                continue

            image_data = self.image_queue.get_nowait()

            detections = self.image_processor.yolo_detection(image_data["image"])

        timestamp = image_data["timestamp"]
        if detections:
            self.detected_objects.extend(
                [(obj, conf, timestamp) for obj, conf in detections]
            )
        self.image_queue.task_done()

    def odom_callback(self, msg):
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            quaternion = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
            _, _, yaw = euler_from_quaternion(quaternion)
            theta = yaw
            self.ground_truth_pose = [x, y, theta]
            # self.node.get_logger().info(f"Ground truth pose: x={x} y={y} theta={theta}")
        except AttributeError as e:
            self.node.get_logger().warn(f"Unexpected /pose message format: {e}")

    def update_pose(self):
        if not self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time()):
            # self.node.get_logger().warn(f"Transform not available yet")
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
        # self.node.get_logger().info(
        #     f"Pose updated: x={(self.pose.x)} y={(self.pose.y)} theta={(self.pose.theta)}"
        # )
        self.estimated_pose = [self.pose.x, self.pose.y, self.pose.theta]

    def scan_callback(self, msg):
        front_idx = self.object_detection.get_idx(msg, 0)
        right_idx = self.object_detection.get_idx(msg, 3 * np.pi / 2)
        left_idx = self.object_detection.get_idx(msg, np.pi / 2)
        r_front = self.object_detection.get_range(msg, front_idx)
        r_right = self.object_detection.get_range(msg, right_idx)
        r_left = self.object_detection.get_range(msg, left_idx)
        self.readings["right"].append(r_right)
        self.readings["left"].append(r_left)
        self.readings["front"].append(r_front)
        if len(self.readings["front"]) > 5:
            self.readings["front"].pop(0)
        if len(self.readings["left"]) > 5:
            self.readings["left"].pop(0)
        if len(self.readings["right"]) > 5:
            self.readings["right"].pop(0)

        self.object_is_detected = self.object_detection.detect_object(msg)

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution

        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        # self.node.get_logger().info(
        #     f"Map received {self.map_width} x {self.map_height}"
        # )

    def log_row(self):
        if self.estimated_pose is not None and self.ground_truth_pose is not None:
            row = self.estimated_pose + self.ground_truth_pose
            self.trajectory.append(row[:3])  # traj_x, traj_y, traj_theta
            self.ground_truth_trajectory.append(row[3:])  # gt_x, gt_y, gt_theta

    def image_callback(self, msg):
        if self.object_is_detected:
            image_data = self.image_processor.process_image(msg)
            if image_data is not None:
                try:
                    self.image_queue.put_nowait(image_data)
                except queue.Full:
                    pass
                self.img_height = msg.height
                self.img_width = msg.width
                self.img_encoding = msg.encoding


def main(args=None):
    turtlebot = None
    data = []
    folder = "robot_data"
    os.makedirs(folder, exist_ok=True)

    try:
        print("[DEBUG] Creating Turtlebot3 instance...", flush=True)
        turtlebot = Turtlebot3()
        print("[DEBUG] Turtlebot3 created successfully", flush=True)
        turtlebot.i = 0
        turtlebot.node.get_logger().info("Main is called, node is created")
        print("[DEBUG] About to run robot control loop...", flush=True)
        turtlebot.run()  # blocking

    except KeyboardInterrupt:
        if turtlebot is not None:
            print(len(turtlebot.trajectory))
            print(len(turtlebot.ground_truth_trajectory))
            # Create rows for each time step, zipping trajectory and ground truth
            data = []
            for traj_point, gt_point in zip(
                turtlebot.trajectory, turtlebot.ground_truth_trajectory
            ):
                row = (
                    traj_point + gt_point
                )  # [traj_x, traj_y, traj_theta, gt_x, gt_y, gt_theta]
                data.append(row)
            # Save as CSV with multiple rows
            np.savetxt(
                os.path.join(folder, "trajectories.csv"),
                data,
                delimiter=",",
                header="traj_x,traj_y,traj_theta,gt_x,gt_y,gt_theta",
                comments="",  # No # for header
            )
            print(f"Trajectories saved to {folder}/trajectories.csv")
            object_poses = turtlebot.object_detection.get_objects()
            print("The saved poses are: ", object_poses)

            # Save all captured images with metadata and YOLO predictions
            if turtlebot.image_queue:
                images_folder = os.path.join(folder, "images")
                os.makedirs(images_folder, exist_ok=True)
                for idx, image_data in enumerate(turtlebot.image_queue.queue):
                    img_filename = f"image_{idx:04d}.png"
                    img_path = os.path.join(images_folder, img_filename)
                    cv2.imwrite(
                        img_path, cv2.cvtColor(image_data["image"], cv2.COLOR_RGB2BGR)
                    )
                print(f"Captured images saved to {images_folder}/")

    finally:
        if turtlebot is not None and turtlebot.map_data is not None:
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
