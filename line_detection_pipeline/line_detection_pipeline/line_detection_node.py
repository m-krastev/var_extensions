import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import sys
print(sys.version_info)
from .constants import IMAGE_TOPIC, OUTPUT_TOPIC

class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.process_image_callback,
            10)
        self.publisher = self.create_publisher(Image, OUTPUT_TOPIC, 10)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.logger.info("Line Detection Node started.")

    def process_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Step 1: Preprocessing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        bright = cv2.convertScaleAbs(gray, alpha=1.5, beta=0)  # adjust brightness if needed
        blurred = cv2.GaussianBlur(bright, (5, 5), 0)

        # Step 2: Edge Detection using Canny
        edges = cv2.Canny(blurred, 50, 150)

        # Step 3: Line Detection with Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=10)
        
        # Step 4: Filter and Draw Detected Lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Convert processed image back to ROS Image message
        output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher.publish(output_msg)
        self.get_logger().info("Processed and published line-detected image.")
        
        # Display the processed image in a non-blocking OpenCV window
        cv2.imshow('Processed Image', cv_image)
        
        # Non-blocking wait for 1 ms to update the window
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
