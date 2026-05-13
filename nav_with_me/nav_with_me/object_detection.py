import numpy as np
from torch import det

DIST_THRESHOLD = 0.3  # meters


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
            return True
        if left_dist < 0.5:
            self.log_and_append(self.robot.pose.x, self.robot.pose.y, "on the left")
            return True
        if right_dist < 0.5:
            self.log_and_append(self.robot.pose.x, self.robot.pose.y, "on the right")
            return True
        return False

    def get_idx(self, msg, angle):
        return int((angle - msg.angle_min) / msg.angle_increment)

    def get_range(self, msg, idx):
        start = max(0, idx - self.window)
        end = min(len(msg.ranges), idx + self.window + 1)

        slice_ranges = msg.ranges[start:end]
        valid_ranges = [r for r in slice_ranges if not np.isinf(r) and not np.isnan(r)]
        return min(valid_ranges) if valid_ranges else float("inf")

    def log_and_append(self, x, y, position):
        # self.robot.node.get_logger().warn(
        #     f"Object detected at {position} at ({x:.2f}, {y:.2f})"
        # )
        self.objects_poses.append((x, y))

    def get_objects(self):
        return self.objects_poses

    def is_same_object(self, obj1, obj2):
        if obj1["class"] != obj2["class"]:
            return False

        p1 = np.array(obj1["pose"][:2])
        p2 = np.array(obj2["pose"][:2])
        return np.linalg.norm(p1 - p2) < DIST_THRESHOLD

    def filter_objects(self, old_detections, new_detections):
        filtered = [
            obj.copy() for obj in old_detections
        ]  # Start with copies of old detections
        for det in new_detections:
            matched = False
            for obj in filtered:
                if self.is_same_object(obj, det):
                    obj["pose"] = [
                        (obj["pose"][i] + det["pose"][i]) / 2 for i in range(3)
                    ]
                    obj["count"] += 1
                    obj["conf"] = max(obj["conf"], det["conf"])
                    matched = True
                    break
            if not matched:
                det_copy = det.copy()
                det_copy["count"] = 1
                filtered.append(det_copy)
        return filtered
