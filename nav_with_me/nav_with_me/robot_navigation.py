import os
import numpy as np
import threading
import queue
from ultralytics import YOLO
from .object_detection import ObjectDetection
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from .img_object_detection import ImageProcessor
from .nav2_goal_manager import Nav2GoalManager


class Turtlebot3:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("turtlebot3_move_square")
        self.node.declare_parameter("use_fake_detections", False)
        self.use_fake_detections = self.node.get_parameter("use_fake_detections").value
        self.node.get_logger().info("Pass Ctrl + C to terminate")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.rate = self.node.create_rate(1)
        self.timer = self.node.create_timer(0.1, self.update_pose)  # 10 Hz
        self.timer_2 = self.node.create_timer(0.1, self.log_row)
        self.object_detection = ObjectDetection(self)
        self.image_processor = ImageProcessor(self)

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
        self.exploration_completed_sub = self.node.create_subscription(
            Bool,
            "/exploration_completed",
            self.exploration_completed_callback,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.perception_ready_pub = self.node.create_publisher(
            Bool,
            "/perception_ready",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
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
        self.img_height = None
        self.img_width = None
        self.img_encoding = None
        self.detected_objects = []  # Store detected objects with timestamps and poses
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.image_queue = queue.Queue(maxsize=50)
        self.interval = 0.1
        self.way_points = []
        self.current_index = 0
        self.starting_position = None
        self.exploration_complete = False
        self.re_explore = False
        self.home_goal_sent = False
        self.final_home_goal_sent = False
        self.waypoints_initialized = False
        self.mission_complete = False
        self.nav_goal_manager = Nav2GoalManager(
            self.node, self.navigation_result_callback
        )
        self.yolo_ready = threading.Event()
        self.yolo_load_failed = False
        self.yolo_wait_logged = False

        t = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        t.start()
        # YOLO
        self.yolo_running = True
        self.yolo_model = None
        yolo_thread = threading.Thread(target=self._yolo_worker, daemon=True)
        yolo_thread.start()

    def run(self):
        while rclpy.ok():
            if not self.yolo_ready.is_set():
                if self.yolo_load_failed and not self.yolo_wait_logged:
                    self.node.get_logger().error(
                        "YOLO failed to load; navigation will not start"
                    )
                    self.yolo_wait_logged = True
                elif not self.yolo_wait_logged:
                    self.node.get_logger().info(
                        "Waiting for YOLO model before starting navigation"
                    )
                    self.yolo_wait_logged = True
            elif not self.exploration_complete:
                self.node.get_logger().info("Exploration in progress")
            elif not self.home_goal_sent:
                self.node.get_logger().info("Navigating home...")
                self.nav_home()
            elif self.re_explore and not self.mission_complete:
                self.navigate_to_detected_objects()
            elif self.mission_complete and not self.final_home_goal_sent:
                self.node.get_logger().info("Mission complete; returning home")
                self.final_home_goal_sent = self.nav_home()
            self.rate.sleep()

    def exploration_completed_callback(self, msg):
        if msg.data == True:
            self.node.get_logger().info(
                "C++ motion controller reports exploration complete"
            )
            self.exploration_complete = True

    def _yolo_worker(self):
        self.node.get_logger().info("Starting perception worker thread")

        if self.use_fake_detections:
            self.node.get_logger().warning(
                "Fake detection mode enabled; skipping YOLO model load"
            )
        else:
            try:
                self.yolo_model = YOLO("yolov8n.pt")
                self.node.get_logger().info("YOLO model loaded successfully")
            except Exception as e:
                self.node.get_logger().error(f"Failed to load YOLO model: {e}")
                self.yolo_load_failed = True
                return
        self.yolo_ready.set()
        ready_msg = Bool()
        ready_msg.data = True
        self.perception_ready_pub.publish(ready_msg)
        self.node.get_logger().info(
            "Perception is ready; exploration and navigation are enabled"
        )

        while self.yolo_running:
            if not self.image_queue.qsize() > 0:
                continue

            image_data = self.image_queue.get_nowait()

            detections = self.image_processor.yolo_detection(image_data["image"])

            if detections:
                if len(self.detected_objects) == 0:
                    self.detected_objects = detections
                else:
                    self.detected_objects = self.object_detection.filter_objects(
                        self.detected_objects, detections
                    )
            self.node.get_logger().info(f"Detected objects: {self.detected_objects}")
            self.interval += 0.1
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
        self.starting_position = (
            self.estimated_pose.copy()
            if self.starting_position is None
            else self.starting_position
        )

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
        self.node.get_logger().info(f"Objects received {self.detected_objects}")

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

    def create_waypoints(self):
        def make_pose(x, y):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            return pose

        object_waypoints = []
        for object in self.detected_objects:
            x, y = object["pose"][:2]
            object_waypoints.append(make_pose(x, y))
        return object_waypoints

    def navigate_to_detected_objects(self):
        if not self.waypoints_initialized:
            self.node.get_logger().info("Starting detected-object waypoint navigation")
            self.way_points = self.create_waypoints()
            self.current_index = 0
            self.waypoints_initialized = True

        if self.nav_goal_manager.busy:
            return

        if self.current_index >= len(self.way_points):
            self.node.get_logger().info("All detected-object waypoints reached")
            self.mission_complete = True
            return

        self.nav_goal_manager.send_goal(self.way_points[self.current_index], "object")

    def navigation_result_callback(self, goal_type, succeeded):
        if goal_type == "home":
            if succeeded:
                self.node.get_logger().info("Successfully returned home")
                self.re_explore = True
            else:
                self.node.get_logger().error(
                    "Return-home goal failed; object navigation will not start"
                )
            return

        if not succeeded:
            self.node.get_logger().warn(
                f"Object waypoint {self.current_index} failed; moving to next"
            )
        self.current_index += 1

    def nav_home(self):
        if self.starting_position is None:
            self.node.get_logger().warn(
                "Waiting for the starting pose before returning home"
            )
            return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.starting_position[0]
        pose.pose.position.y = self.starting_position[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        if self.nav_goal_manager.send_goal(pose, "home"):
            self.home_goal_sent = True
            return True
        return False


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
            print(f"Deteted objects are: {turtlebot.detected_objects}")

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
