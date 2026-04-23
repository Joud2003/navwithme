import numpy as np


class ObjectDetection:
    def __init__(self, robot):
        self.poses = []
        self.robot = robot

    def detect_front_object(self, msg):
        front_dist = np.mean(self.robot.front_readings)
        if front_dist < 0.5:
            self.poses.append((self.robot.pose.x, self.robot.pose.y))
            self.robot.node.get_logger().info(
                f"Object detected at ({self.robot.pose.x:.2f}, {self.robot.pose.y:.2f})"
            )
