import sys
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import os
from .constants import (LINE_DIRECTION_TOPIC, RAE_RIGHT_IMAGE_RAW_TOPIC, OUTPUT_TOPIC,
                        RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC)

print(sys.version_info)


class LineDetectionNode(Node):
    def __init__(self):
        super().__init__("line_detection_node")
        self.subscription = self.create_subscription(
            CompressedImage, RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC, self.process_image_callback, 10
        )
        self.publisher = self.create_publisher(Image, OUTPUT_TOPIC, 10)
        self.direction_publisher = self.create_publisher(Twist, LINE_DIRECTION_TOPIC, 10)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.logger.info("Line Detection Node started.")
        print(os.getcwd())
        self.dist = np.loadtxt("dist")
        self.mtx = np.loadtxt("mtx")

    def process_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = self.undistort(img)


        # Preprocessing
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        bright = cv2.convertScaleAbs(
            gray, alpha=1.5, beta=0)

        # Brigthness Threshold 
        brightness_threshold = 235
        _, mask = cv2.threshold(img, brightness_threshold, 255, cv2.THRESH_BINARY)
        

        # Assuming masked_image is the image you want to process
        # Convert the image to grayscale if it is not already
        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

        # Ensure the image is in CV_8UC1 format
        mask = cv2.convertScaleAbs(mask)

        masked_image = cv2.bitwise_and(img, img, mask=mask)

        # Find contours
        masked_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=len, reverse=True)
        
        # Fit line to top 5 biggest contours
        top_n=5
        lines_info = []
        xs = []
        y1 = 0
        for i, contour in enumerate(contours[:top_n]):
            points = contour.reshape(-1, 2)
            [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
            slope = vy / vx if vx != 0 else float('inf')
            a = -vy
            b = vx
            c = vy * x0 - vx * y0
            errors =  np.abs(a * points[:, 0] + b * points[:, 1] + c) / np.sqrt(a**2 + b**2) 
            x1 = x0 + (y1-y0)/slope
            xs.append(abs(x1-img.shape[1]/2))
            lines_info.append((slope,  np.mean(errors), contour, (vx, vy, x0, y0)))

        # Find most vertical line and draw it red
        if len(lines_info) != 0:
            min_dist_index = np.argmin(xs)
            min_slope_contour = lines_info[min_dist_index][2]
            slope = lines_info[min_dist_index][0]

            cv2.drawContours(img, [min_slope_contour], -1, (255, 0, 0), 2)

            # Change direction to make the vertical line more vertical
            direction = Twist()
            direction.angular.z = float(abs(slope) / slope)
            self.direction_publisher.publish(direction)


        # Convert processed image back to ROS Image message
        output_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher.publish(output_msg)
        self.get_logger().info("Processed and published line-detected image.")

        # First row: original image and grayscale image
        # Second row: brightened image and mask
        first = np.hstack((img, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)))
        second = np.hstack((cv2.cvtColor(bright, cv2.COLOR_GRAY2BGR), cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        debug = np.vstack((first, second))
        debug = cv2.resize(debug, (0, 0), fx=0.6, fy=0.6)
        # Display the processed image in a non-blocking OpenCV window
        cv2.imshow("Processed Image", debug)

        # Non-blocking wait for 5 ms to update the window
        cv2.waitKey(5)

    def undistort(self,img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        return dst

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
