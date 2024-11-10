import argparse
import sys
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, BatteryState
import os
from .constants import (LCD_TOPIC, LINE_DIRECTION_TOPIC, RAE_RIGHT_IMAGE_RAW_TOPIC, OUTPUT_TOPIC,
                        RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC, BATTERY_STATUS_TOPIC, highprofile, lowprofile)

from PIL import ImageDraw, ImageFont, Image as PILImage

print(sys.version_info)


class LineDetectionNode(Node):
    def __init__(self, use_canny=False, show_battery=False):
        super().__init__("line_detection_node")
        self.use_canny = use_canny

        self.subscription = self.create_subscription(
            CompressedImage, RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC, self.process_image_callback, 10
        )

        self.battery_reader = self.create_subscription(BatteryState, BATTERY_STATUS_TOPIC, self.battery_callback, 10)
        self.charge = 0
        self.status = "U"

        self.direction_publisher = self.create_publisher(Twist, LINE_DIRECTION_TOPIC, 10)
        
        self.lcd_publisher = None
        if show_battery:
            self.lcd_publisher = self.create_publisher(Image, LCD_TOPIC, 1)

        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.logger.info("Line Detection Node started.")
        print(os.getcwd())
        self.dist = np.loadtxt("dist")
        self.mtx = np.loadtxt("mtx")

    def create_text_image(self, text, width=160, height=80):
        """
        Creates a black and white image with the given text centered on it.
        The font size is automatically determined based on the image size.
        """
        image = PILImage.new("L", (width, height), color="black")
        draw = ImageDraw.Draw(image)

        # Start with a large font size and decrease until the text fits
        font_size = 50
        while font_size > 0:
            font = ImageFont.load_default(font_size)
            bbox = draw.textbbox((0, 0), text, font=font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            if text_width <= width and text_height <= height:
                break
            font_size -= 1

        position = ((width - text_width) // 2, (height - text_height) // 2)
        draw.text(position, text, fill="white", font=font)
        return image

    def battery_callback(self, msg):
        self.charge = msg.capacity
        match msg.power_supply_status:
            case BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
                self.status = "D" # Discharging
            case BatteryState.POWER_SUPPLY_STATUS_CHARGING:
                self.status = "C" # Charging
            case BatteryState.POWER_SUPPLY_STATUS_FULL:
                self.status = "F" # Full
            case _:
                self.status = "U" # Unknown
        if self.lcd_publisher is not None:
            image = self.create_text_image(f"{self.status}/{self.charge:.0f}%")
            image = cv2.cvtColor(np.asarray(image, dtype=np.uint8), cv2.COLOR_GRAY2BGR)
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.lcd_publisher.publish(msg)

    def process_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = self.undistort(img)

        # Detect the line in the image
        if self.use_canny:
            debug, line, direction = self.detect_line_canny(img)
        else:
            debug, line, direction = self.detect_line_ours(img)
        
        if direction is not None:
            self.direction_publisher.publish(direction)
    
        self.get_logger().info("Processed and published line-detected image.")
        # Write the battery status and charge on the image
        battery_status = f"{self.status}/{self.charge:.0f}%"
        cv2.putText(debug, battery_status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # debug = cv2.resize(debug, (0, 0), fx=0.6, fy=0.6)
        
        # Display the processed image in a non-blocking OpenCV window
        cv2.imshow(f"Processed Image ({battery_status})", debug)

        # Non-blocking wait for 5 ms to update the window
        cv2.waitKey(5)

    def undistort(self,img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        return dst
    
    
    def detect_line_ours(self, image):
        # Preprocessing
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        bright = cv2.convertScaleAbs(gray, alpha=1.5, beta=0)

        # Brigthness Threshold 
        brightness_threshold = 235
        _, mask = cv2.threshold(image, brightness_threshold, 255, cv2.THRESH_BINARY)
        
        # Convert the image to grayscale if it is not already
        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

        # Ensure the image is in CV_8UC1 format
        mask = cv2.convertScaleAbs(mask)
        masked_image = cv2.bitwise_and(image, image, mask=mask)

        # Find contours
        masked_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=len, reverse=True)
        
        # Fit line to top 10 biggest contours and filtering those with fitting error >2
        top_n=10
        error_threshold = 2
        lines_info = []
        y1=0
        for i, contour in enumerate(contours[:top_n]):
            points = contour.reshape(-1, 2)
            [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
            slope = vy / vx if vx != 0 else float('inf')
            a = -vy
            b = vx
            c = vy * x0 - vx * y0
            errors =  np.abs(a * points[:, 0] + b * points[:, 1] + c) / np.sqrt(a**2 + b**2) 
            error = np.mean(errors)
            x1 = x0 + (y1-y0)/slope
            if error <2:
                lines_info.append((slope,  error, contour, (vx, vy, x0, y0), abs(x1-image.shape[1]/2)))

        # Find most vertical line and draw it red
        if len(lines_info) != 0:
            min_dist_line = min(lines_info, key=lambda x: min(x[4]))
            _, _, contour, (vx, vy, x0, y0), _ = min_dist_line
            x1 = int(x0 - 1000 * vx)
            y1 = int(y0 - 1000 * vy)
            x2 = int(x0 + 1000 * vx)
            y2 = int(y0 + 1000 * vy)

            cv2.drawContours(image, [contour], -1, (255, 0, 255), 1)
            cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 1)

            min_dist_index = np.argmin(xs)
            min_slope_contour = lines_info[min_dist_index][2]
            slope = lines_info[min_dist_index][0]

            cv2.drawContours(image, [min_slope_contour], -1, (255, 0, 0), 2)

            # Change direction to make the vertical line more vertical
            direction = Twist()
            direction.angular.z = float(abs(slope) / slope)

        # First row: original image and grayscale image
        # Second row: brightened image and mask
        first = np.hstack((image, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)))
        second = np.hstack((cv2.cvtColor(bright, cv2.COLOR_GRAY2BGR), cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        return np.vstack((first, second)), min_slope_contour, direction
    
    def detect_line_canny(self, image):
        # Step 1: Preprocessing
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
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
            image_center_x = image.shape[1] // 2

            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                line_brightness = (bright[y1, x1] + bright[y2, x2]) / 2
                line_center_x = (x1 + x2) / 2

                if line_brightness > max_brightness and abs(line_center_x - image_center_x) < 50:
                    max_brightness = line_brightness
                    brightest_line = line[0]

            if brightest_line is not None:
                x1, y1, x2, y2 = brightest_line
                # Draw the brightest line in red
                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 5)
                direction = Twist()
                direction.angular.z = (x1 + x2) / 2 - image_center_x

        first_row = np.hstack((image, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)))
        second_row = np.hstack((cv2.cvtColor(bright, cv2.COLOR_GRAY2BGR), cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)))
        return np.vstack((first_row, second_row)), None, None


def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_canny", action="store_true", help="Use Canny edge detection")
    parser.add_argument("--show_battery", action="store_true", help="Show battery status on the LCD")
    args, _ = parser.parse_known_args()

    node = LineDetectionNode(**vars(args))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()