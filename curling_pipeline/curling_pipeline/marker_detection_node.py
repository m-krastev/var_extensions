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


class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__("line_detection_node")
        self.subscription = self.create_subscription(
            CompressedImage, RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC, self.process_image_callback, 10
        )
        self.publisher = self.create_publisher(Image, OUTPUT_TOPIC, 10)
        self.direction_publisher = self.create_publisher(Twist, LINE_DIRECTION_TOPIC, 10)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.logger.info("Marker Detection Node started.")
        print(os.getcwd())
        self.dist = np.loadtxt("dist")
        self.mtx = np.loadtxt("mtx")

    def process_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = self.undistort(img)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
        else:
            print("No AprilTags detected.")

        # Find most vertical line and draw it red

            #self.direction_publisher.publish(direction)


        # Convert processed image back to ROS Image message
        #output_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        #self.publisher.publish(output_msg)
        #self.get_logger().info("Processed and published line-detected image.")

        # First row: original image and grayscale image
        # Second row: brightened image and mask
        # Display the processed image in a non-blocking OpenCV window
        cv2.imshow("Processed Image", image)

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
    node = MarkerDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
