import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from .constants import IMAGE_TOPIC, OUTPUT_TOPIC

print(sys.version_info)


class LineDetectionNode(Node):
    def __init__(self):
        super().__init__("line_detection_node")
        self.subscription = self.create_subscription(
            Image, IMAGE_TOPIC, self.process_image_callback, 10
        )
        self.publisher = self.create_publisher(Image, OUTPUT_TOPIC, 10)
        self.direction_publisher = self.create_publisher(Twist, "line_direction", 10)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.logger.info("Line Detection Node started.")

    def process_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Step 1: Preprocessing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        bright = cv2.convertScaleAbs(
            gray, alpha=1.5, beta=0
        )  # adjust brightness if needed
        blurred = cv2.GaussianBlur(bright, (5, 5), 0)

        # Step 2: Edge Detection using Canny
        edges = cv2.Canny(blurred, 150, 200)

        # Step 3: Line Detection with Hough Transform
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=150, minLineLength=225, maxLineGap=20
        )

        # Step 4: Filter and Draw Detected Lines
        if lines is not None:
            # Find the brightest line that is closest to the vertical center
            brightest_line = None
            max_brightness = -1
            image_center_x = cv_image.shape[1] // 2

            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                line_brightness = (bright[y1, x1] + bright[y2, x2]) / 2
                line_center_x = (x1 + x2) / 2

                if line_brightness > max_brightness and abs(line_center_x - image_center_x) < 50:
                    max_brightness = line_brightness
                    brightest_line = line[0]

            if brightest_line is not None:
                x1, y1, x2, y2 = brightest_line
                # Draw the brightest line in red
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 5)

                # Calculate the direction of the line
                direction = Twist()
                direction.linear.x = float(x2 - x1)
                direction.angular.z = float(y2 - y1)
                self.direction_publisher.publish(direction)

        # Convert processed image back to ROS Image message
        output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher.publish(output_msg)
        self.get_logger().info("Processed and published line-detected image.")

        # Display the processed image in a non-blocking OpenCV window
        cv2.imshow("Processed Image", cv_image)

        # Non-blocking wait for 1 ms to update the window
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
