import argparse
import os
import sys
from datetime import datetime

import cv2
import cv2.aruco as aruco

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
    def __init__(self, marker_id, show_battery=False):
        super().__init__("marker_detection_node")

        self.marker_id = marker_id
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
        self.logger.info(f"Marker Detection Node started with target marker ID: {self.marker_id}")

        self.dist = np.loadtxt("dist")
        self.mtx = np.loadtxt("mtx")

    def process_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Save the original image to the filesystem
            if not os.path.exists('debug'):
                os.makedirs('debug')
            cv2.imwrite(f"debug/image-{datetime.fromtimestamp(msg.header.stamp.sec)}.png", img)
            self.logger.info("Received image from camera.")

            img = self.undistort(img)

            # Detect ArUco markers
            corners, ids = self.detect_marker(img)
            
            # Visualize the detection results
            self.visualise_detection(corners, ids, img)

            # Display the processed image in a non-blocking OpenCV window
            cv2.imshow("Processed Image", img)
            cv2.waitKey(1)  # Non-blocking wait for 1 ms
        except Exception as e:
            self.logger.error(f"Error in process_image_callback: {e}")

    
    def undistort(self,img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        return dst
       
    def detect_marker(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        detector_params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, detector_params)

        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Convert ids to a Python list for filtering
            ids_list = ids.flatten().tolist()
            # Filter markers based on the specified ID
            valid_corners = [corners[i] for i in range(len(ids_list)) if ids_list[i] == self.marker_id]
            valid_ids = [ids_list[i] for i in range(len(ids_list)) if ids_list[i] == self.marker_id]

            if valid_ids:
                print(f"[INFO] Detected specified ArUco marker ID: {self.marker_id}")
                self.logger.info(f"Detected specified ArUco marker ID: {self.marker_id}")
                # Convert valid_ids back to a NumPy array
                valid_ids = np.array(valid_ids, dtype=np.int32).reshape(-1, 1)
            else:
                print("[INFO] Specified ArUco marker not detected.")
                self.logger.warning("Specified ArUco marker not detected.")
                valid_corners = []
                valid_ids = None
        else:
            print("[INFO] No ArUco markers detected.")
            self.logger.warning("No ArUco markers detected.")
            valid_corners = []
            valid_ids = None

        return valid_corners, valid_ids



    def visualise_detection(self, corners, ids, image):
        if ids is not None and len(ids) > 0:
            # Draw the detected markers on the image
            aruco.drawDetectedMarkers(image, corners, ids)
            # Iterate through each detected marker
            for i in range(len(ids)):
                # Get the corners of the marker
                corner = corners[i][0]
                ptA = (int(corner[0][0]), int(corner[0][1]))
                ptB = (int(corner[1][0]), int(corner[1][1]))
                ptC = (int(corner[2][0]), int(corner[2][1]))
                ptD = (int(corner[3][0]), int(corner[3][1]))

                # Draw the bounding box of the marker
                cv2.line(image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(image, ptD, ptA, (0, 255, 0), 2)

                # Draw the center of the marker
                cX = int((ptA[0] + ptC[0]) / 2)
                cY = int((ptA[1] + ptC[1]) / 2)
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

                # Display the marker ID
                marker_id = ids[i][0]
                cv2.putText(image, f"ID: {marker_id}", (ptA[0], ptA[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print(f"[INFO] Detected ArUco marker ID: {marker_id}")
        else:
            self.logger.warning("No markers to visualize.")


        # Show the image with detected markers
        #cv2.imshow("Image", image)
        #cv2.waitKey(1)  # Non-blocking wait for 1 ms


def main(args=None):
    rclpy.init(args=args)

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Marker Detection Node")
    parser.add_argument("--marker-id", type=int, required=True, help="ID of the ArUco marker to detect")
    parsed_args = parser.parse_args()

    # Create the node with the specified marker ID
    node = MarkerDetectionNode(marker_id=parsed_args.marker_id)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()