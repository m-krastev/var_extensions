
import argparse
import os
import sys
from datetime import datetime

import cv2
import apriltag
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, CompressedImage, Image

from .constants import BATTERY_STATUS_TOPIC, LCD_TOPIC, RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC, RAE_RIGHT_IMAGE_RAW_TOPIC, highprofile, lowprofile

print(sys.version_info)


class MarkerDetectionNode(Node):
    def __init__(self, show_battery=False):
        super().__init__("marker_detection_node")

        self.subscription = self.create_subscription(
            CompressedImage, RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC, self.process_image_callback, 10
        )
        self.direction_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Battery status and charge
        #self.show_battery = show_battery
        #self.charge, self.status = 0, "U"
        #self.lcd_publisher = self.create_publisher(Image, LCD_TOPIC, 10)
        #self.battery_reader = self.create_subscription(BatteryState, BATTERY_STATUS_TOPIC, self.battery_callback, 10)

        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.logger.info("Marker Detection Node started.")

        print(os.getcwd())
        self.dist = np.loadtxt("dist")
        self.mtx = np.loadtxt("mtx")


    def process_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Save the original image to the filesystem
        cv2.imwrite(f"debug/image-{datetime.fromtimestamp(msg.header.stamp.sec)}.png", img)
        self.logger.info("Received image from camera.")

        img = self.undistort(img)

        # Detect AprilTags
        results = self.detect_marker(img)
        
        # Visualize the detection results
        self.visualise_detection(results, img)

        # Display the processed image in a non-blocking OpenCV window
        cv2.imshow("Processed Image", img)

        # Non-blocking wait for 5 ms to update the window
        #cv2.waitKey(5)
    
    def undistort(self,img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        return dst
       
    def detect_marker(self, image):
        results = None
        # Preprocessing
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        print("[INFO] detecting AprilTags...")
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))

        if not results:
            self.logger.warning("No AprilTags detected in the current frame.")

        return results

    def visualise_detection(self, results, image):
        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(tagFamily))
        # show the output image after AprilTag detection
        cv2.imshow("Image", image)
        cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    
    """    
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=False,
	help="path to input image containing AprilTag")
    args = vars(ap.parse_args())
    """
    node = MarkerDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()