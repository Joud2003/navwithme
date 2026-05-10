from datetime import datetime
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
        results = self.robot.yolo_model(cv_image, verbose=False)
        detections = results[0].boxes.data.cpu().numpy()
        detected_objects = []
        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            if conf > CONFIDENCE_THRESHOLD:
                class_name = self.robot.yolo_model.names[int(cls_id)]
                if class_name in HARMFUL_OBJECTS:
                    self.robot.node.get_logger().warn(
                        f"Harmful object detected: {class_name} with confidence {conf:.2f}"
                    )
                    new_object = {
                        "class": class_name,
                        "pose": self.robot.estimated_pose,
                        "conf": conf,
                        "count": 1,
                    }
                    detected_objects.append(new_object)

        return detected_objects

    def fake_detections(self, interval=0.1):
        if self.robot.estimated_pose is None:
            return []

        x, y, theta = self.robot.estimated_pose
        array_of_objects = []
        for i in range(0, 20, 1):
            new_object = {
                "class": "knife",
                "pose": [x + 0.2 + (i * interval), y + 0.1 + (i * interval), theta],
                "conf": 0.9 - (i * interval * 0.1),
                "count": 1,
            }
            array_of_objects.append(new_object)
        return array_of_objects
