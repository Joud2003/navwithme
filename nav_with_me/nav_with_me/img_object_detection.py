from datetime import datetime
import numpy as np
from cv_bridge import CvBridge
from .constants import HARMFUL_OBJECTS, CONFIDENCE_THRESHOLD


class ImageProcessor:
    def __init__(self, robot):
        self.robot = robot
        self.bridge = CvBridge()

    def process_image(self, msg):
        self.robot.node.get_logger().info("Image received while object is detected")
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Store timestamp and current pose with the image
            timestamp = datetime.now().isoformat()
            image_data = {"timestamp": timestamp, "image": cv_image}
            return image_data
        except Exception as e:
            self.robot.node.get_logger().error(f"Failed to process image: {e}")
            return None

    def yolo_detection(self, cv_image):
        if self.robot.use_fake_detections:
            detections = self.fake_detections()
            self.robot.node.get_logger().info(f"Fake detections: {detections}")
            return detections

        results = self.robot.yolo_model(cv_image, verbose=False)
        detections = results[0].boxes.data.cpu().numpy()
        detected_objects = []
        if self.robot.estimated_pose is None:
            return detected_objects

        for detection in detections:
            if len(detection) < 6:
                continue

            confidence = float(detection[4])
            class_id = int(detection[5])
            class_name = self.robot.yolo_model.names[class_id]
            if confidence <= CONFIDENCE_THRESHOLD or class_name not in HARMFUL_OBJECTS:
                continue

            self.robot.node.get_logger().warning(
                f"Harmful object detected: {class_name} "
                f"with confidence {confidence:.2f}"
            )
            detected_objects.append(
                {
                    "class": class_name,
                    "pose": list(self.robot.estimated_pose),
                    "conf": confidence,
                    "count": 1,
                }
            )

        self.robot.node.get_logger().info(
            f"YOLO harmful detections: {detected_objects}"
        )
        return detected_objects

    def fake_detections(self):

        if self.robot.estimated_pose is None:
            return []

        x, y, theta = self.robot.estimated_pose

        noise_x = np.random.normal(0, 0.03)
        noise_y = np.random.normal(0, 0.03)
        noise_conf = np.random.normal(0, 0.1)

        obj = {
            "class": "knife",
            "pose": [x + 0.02 + noise_x, y + 0.01 + noise_y, theta],
            "conf": 0.9 - noise_conf,
            "count": 1,
        }

        return [obj]
