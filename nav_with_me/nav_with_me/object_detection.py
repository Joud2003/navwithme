import numpy as np


class ObjectDetection:
    def __init__(self, robot):
        self.objects_poses = []
        self.robot = robot
        self.window = 5

    def detect_object(self, msg):
        front_dist = np.mean(self.robot.readings["front"])
        left_dist = np.mean(self.robot.readings["left"])
        right_dist = np.mean(self.robot.readings["right"])

        if front_dist < 0.5:
            self.log_and_append(self.robot.pose.x, self.robot.pose.y, "in front")
        if left_dist < 0.5:
            self.log_and_append(self.robot.pose.x, self.robot.pose.y, "on the left")
        if right_dist < 0.5:
            self.log_and_append(self.robot.pose.x, self.robot.pose.y, "on the right")

    def get_idx(self, msg, angle):
        return int((angle - msg.angle_min) / msg.angle_increment)

    def get_range(self, msg, idx):
        start = max(0, idx - self.window)
        end = min(len(msg.ranges), idx + self.window + 1)

        slice_ranges = msg.ranges[start:end]
        valid_ranges = [r for r in slice_ranges if not np.isinf(r) and not np.isnan(r)]
        return min(valid_ranges) if valid_ranges else float("inf")

    def log_and_append(self, x, y, position):
        self.robot.node.get_logger().warn(
            f"Object detected at {position} at ({x:.2f}, {y:.2f})"
        )
        self.objects_poses.append((x, y))

    def get_objects(self):
        return self.objects_poses
