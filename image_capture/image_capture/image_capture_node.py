import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageCaptureNode(Node):
    def __init__(self, camera_side, key_press_mode):
        super().__init__('image_capture_node')

        self.key_press_mode = key_press_mode

        #Access right or left camera depending on user input
        if camera_side == 'left':
            self.image_topic = '/rae/left/image_raw'
        elif camera_side == 'right':
            self.image_topic = '/rae/right/image_raw'
        else:
            raise ValueError("Choose the 'left' or 'right' camera. Current inptu is invalid.")

        #Folder name for output images
        self.start_time = datetime.now().strftime('%Y-%m-%d_%H-%M')
        self.image_folder = f'images_{self.start_time}_{self.image_topic.lstrip("/").replace("/", "_")}'
        os.makedirs(self.image_folder, exist_ok=True)

        #ros2 subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,  #Front right camera
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.cv_image = None
        self.get_logger().info("Image Capture Node started. Please wait for up to 10 seconds for images to appear")

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') #convert ros2 image to OpenCV

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def save_image(self):
        # Save image to a file
        if self.cv_image is not None:
            try:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = os.path.join(self.image_folder, f'image_{timestamp}.jpg')
                cv2.imwrite(filename, self.cv_image)
                self.get_logger().info(f"Saved image to {filename}")
            except Exception as e:
                self.get_logger().error(f"Error saving image: {e}")
        else:
            self.get_logger().warning("No image available to save.")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.key_press_mode == 'y':
                self.get_logger().info("Press 's' to save an image.")
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    self.save_image()
            else:
                self.save_image()
                time.sleep(1)


def main(args=None):
    parser = argparse.ArgumentParser(description = "Choose RAE camera from whicht o capture images")
    parser.add_argument('--camera', choices = ['left', 'right'], default='right',
                        help="Choose front camera side: 'left' or 'right' (default)")
    parser.add_argument('--key_press', choices=['y', 'n'], default = 'y', help= "'y' to only take photos upon key press - 'n' for automatically (1 per second).")
    parser.add_argument('--video', choices=['y','n'], default='n', help="'y' to record video instead of photos (default n)")

    parsed_args, unknown_args = parser.parse_known_args()
    

    rclpy.init(args=unknown_args)
    node = ImageCaptureNode(camera_side=parsed_args.camera, key_press_mode=parsed_args.key_press)
    #rclpy.spin(node)
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
