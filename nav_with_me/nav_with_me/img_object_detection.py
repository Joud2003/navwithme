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
                detected_objects.append((class_name, conf))
                if class_name in HARMFUL_OBJECTS:
                    self.robot.node.get_logger().warn(
                        f"Harmful object detected: {class_name} with confidence {conf:.2f}"
                    )
        return detected_objects
