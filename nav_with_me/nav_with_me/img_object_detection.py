from datetime import datetime
from cv_bridge import CvBridge


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
