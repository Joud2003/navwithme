import numpy as np


class ObjectDetection:
    def __init__(self, robot):
        self.poses = []
        self.robot = robot
        self.window = 5

    def detect_front_object(self, msg):
        front_dist = np.mean(self.robot.front_readings)
        if front_dist < 0.5:
            self.poses.append((self.robot.pose.x, self.robot.pose.y))
            self.robot.node.get_logger().info(
                f"Object detected at ({self.robot.pose.x:.2f}, {self.robot.pose.y:.2f})"
            )

    def get_idx(self, msg, angle):
        return int((angle - msg.angle_min) / msg.angle_increment)

    def get_range(self, msg, idx):
        start = max(0, idx - self.window)
        end = min(len(msg.ranges), idx + self.window + 1)

        slice_ranges = msg.ranges[start:end]
        valid_ranges = [r for r in slice_ranges if not np.isinf(r) and not np.isnan(r)]
        return min(valid_ranges) if valid_ranges else float("inf")
