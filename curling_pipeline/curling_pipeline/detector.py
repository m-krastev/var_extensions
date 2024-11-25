from io import BytesIO
import json
import sys
from .astar import PathFinder
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

import os
from .constants import (LINE_DIRECTION_TOPIC, RAE_RIGHT_IMAGE_RAW_TOPIC, OUTPUT_TOPIC,
                        RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC)
import matplotlib.pyplot as plt
from matplotlib import patches

print(sys.version_info)

# Dimensions in mm
A = 9000  # Field length
B = 6000  # Field width
C = 50  # Line width
D = 100  # Penalty mark size
E = 600  # Goal area length
F = 2200  # Goal area width
G = 1650  # Penalty area length
H = 4000  # Penalty area width
I = 1300  # Penalty mark distance
J = 1500  # Center circle diameter
K = 2700  # Border strip width

# CONSTANTS
dictionaries = [
            cv2.aruco.DICT_7X7_100, 
            cv2.aruco.DICT_APRILTAG_36h11,
            cv2.aruco.DICT_ARUCO_MIP_36h12,
            ]

parameters =  cv2.aruco.DetectorParameters()

# Unique marker keys with (x, x, z (in mm), length_of_square (in m))
markers = {
    # window
    # "36H11_
    "79": (-4500, 1060 + 3000 + 50, 510, 0.08, "window"),
    # "36H11_
    "26": (-4500 + 450, 130 + 1060 + 3000 + 50, 1080, 0.235, "window"),
    # "36H12_
    "67": (-4500 + 450 + 530 + 80, 130 + 1060 + 3000 + 50, 1000, 0.2, "window"),
    # "36H11_
    "69": (-1800, 620 + 3000 + 50, 500, 0.08, "window"),
    # "36H11_
    "18": (0, 130 + 1060 + 3000 + 50, 1050, 0.28, "window"),
    # "36H12_
    "47": (840, 130 + 1060 + 3000 + 50, 1050, 0.2, "window"),
    # "36H11_
    "9": (840 + 850, 1060 + 3000 + 50, 500, 0.08, "window"),
    # "36H11_
    "66": (840 + 850 + 1150, 130 + 1060 + 3000 + 50, 1100, 0.235, "window"),


    # tables
    # "36H11_
    "39": (4500, 1100, 500, 0.08, "tables"),
    # "36H11_
    "46": (4550 + 820, 0, 1680, 0.235, "tables"),  # the one up
    # "36H12_
    "57": (4550 + 820, 0, 1000, 0.2, "tables"),  # the lower one
    # "36H11_
    "49": (4500, -1100, 500, 0.08, "tables"),


    # wall
    # "36H11_
    "16": (2850, -2000 - 3005, 1070, 0.235, "wall"),
    # "36H11_
    "19": (2850 - 1050, -2000 - 3005, 510, 0.08, "wall"),
    # "7x7_
    "37": (2850 - 1050 - 850, -2000 - 3005, 1000, 0.2, "wall"),
    # "36H11_
    "29": (2850 - 1050 - 850 - 1050, -2000 - 3005, 500, 0.08, "wall"),  # one on the bottom
    # "36H11_
    "8": (2850 - 1050 - 850 - 1050, -2000 - 3005, 1060, 0.275, "wall"),  # one on top
    # "36H11_
    "59": (-2800 + 1050, -120 - 3005, 510, 0.08, "wall"),  # column one
    # "7x7_
    "27": (-2800, -2000 - 3005, 1000, 0.2, "wall"),  #
    # "36H11_
    "36": (-2800 - 900, -2000 - 3005, 1080, 0.235, "wall"),


    # ducks
    # "7x7_
    "7": (-4600 - 50 - 930, 0, 1680, 0.2, "ducks"),  # lower one in column
    # "36H11_
    "56": (-4600 - 50 - 930, 0, 1000, 0.235, "ducks"),  # top one in column
    # "36H11_
    "89": (-4500, -1100, 450, 0.08, "ducks"),
    # "36H11_
    "99": (-4500, 1100, 500, 0.08, "ducks"),  # window
}


duck_locations = (
    (G, (B + H) / 2),
    (G, (B - H) / 2),
    (E, (B + F) / 2),
    (E, (B - F) / 2),
    (I, B / 2),
)


class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__("line_detection_node")
        self.subscription = self.create_subscription(
            CompressedImage, RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC, self.process_image_callback, 10
        )
        self.publisher = self.create_publisher(String, OUTPUT_TOPIC, 10)
        self.direction_publisher = self.create_publisher(Twist, LINE_DIRECTION_TOPIC, 10)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.logger.info("Marker Detection Node started.")
        print(os.getcwd())
        self.dist = np.loadtxt("calibration/right_camera/800/dist")
        self.mtx = np.loadtxt("calibration/right_camera/800/mtx")

        
    # Functions
    @staticmethod
    def twopointstriangulation( x_A, y_A, z_A, d_A, x_B, y_B, z_B, d_B):
        A = -2 * (x_A - x_B)
        B = -2 * (y_A - y_B)
        C = d_A**2 - z_A**2 - d_B**2 + z_B**2 - x_A**2 + x_B**2 - y_A**2 + y_B**2

        if B == 0:        
            x1 = C / A
            x2 = C/A

            a = 1
            b = -2 * y_B
            c = (x1**2 - 2*x1 * x_B + x_B**2) + y_B**2 + z_B**2 - d_B**2
            root = b**2 - 4*a*c
            if root >=0:
                y1 = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)
                y2 = (-b - np.sqrt(b**2 - 4*a*c)) / (2*a)
            else:
                # print("Coudln't find match")
                return None

        else:
            a = 1 + A**2/B**2
            b = -2*(x_A + A*C/B**2 - y_A * A/B)
            c = x_A**2 + z_A**2 -d_A**2 - 2*y_A*C/B + (C/B)**2 + y_A**2

            x1 = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)
            x2 = (-b - np.sqrt(b**2 - 4*a*c)) / (2*a)

            if d_B**2 - z_B**2 - (x1-x_B)**2 >= 0 and d_B**2 - z_B**2 - (x2-x_B)**2 >0:
                y1 = y_B + np.sqrt(d_B**2 - z_B**2 - (x1-x_B)**2)
                y2 = y_B + np.sqrt(d_B**2 - z_B**2 - (x2-x_B)**2)
            else:
                # print("Coudln't find match because they don't intersect")
                return None

        return (x1, y1), (x2, y2)



    def compute_triangulation(self, markers_in_img):
        pairs = []
        for i in range(len(markers_in_img)):
            for j in range(i + 1, len(markers_in_img)):
                pairs.append((i, j))

        points = []
        
        for (idx_A, idx_B) in pairs:
            print("Pair",markers_in_img[idx_A][3], markers_in_img[idx_B][3])
            x_A, y_A, z_A = markers_in_img[idx_A][1][:3]
            x_B, y_B, z_B = markers_in_img[idx_B][1][:3]
            d_A = markers_in_img[idx_A][2]
            d_B = markers_in_img[idx_B][2]
            out = self.twopointstriangulation(x_A, y_A, z_A, d_A, x_B, y_B, z_B, d_B)
            if out != None:
                (x1, y1), (x2, y2) = out       
                # print("Point",markers_in_img[idx_A][3],":", x_A, y_A, z_A, d_A)
                # print("Point",markers_in_img[idx_B][3],":", x_B, y_B, z_B, d_B)
                # print("x1,y1:",x1,y1,"x2,y2:",x2,y2)
                p1_fails = abs(y1) > 3000 or abs(x1) > 4500
                p2_fails = abs(y2) > 3000 or abs(x2) > 4500
                if p1_fails and not p2_fails:
                    # print("Chosen", x2,y2)
                    points.append([x2,y2,0])
                elif p2_fails and not p1_fails:
                    # print("Chosen", x1,y1)
                    points.append([x1,y1,0])
                elif p1_fails and p2_fails:
                    print("Discarted pair because out of field")
                else: #both are okay
                    if markers_in_img[idx_A][1][4] == "wall" or markers_in_img[idx_A][1][4] == "window":
                        if abs(y1) > abs(y2):
                            points.append([x1,y1,0])
                        else:
                            points.append([x2,y2,0])
                    else:
                        if abs(x1) > abs(x2):
                            points.append([x1,y1,0])
                        else:
                            points.append([x2,y2,0]) 
            # print("")
            
        if len(points)!=0:
            return np.array(points).mean(axis=0)
        else:
            return []

    def location_for_one_marker(self, x_A, y_A, z_A, x_robot_respect_to_marker, y_robot_respect_to_marker, one_dist):
        a = 1
        if y_robot_respect_to_marker == None: # We look for y_robot
            b = -2*y_A
            c = y_A**2 + (x_robot_respect_to_marker)**2 + (z_A-0)**2 - one_dist**2
            cond = 3000
        elif x_robot_respect_to_marker == None: # We look for x_robot
            b = -2*x_A        
            c = x_A**2 + (y_robot_respect_to_marker)**2 + (z_A-0)**2 - one_dist**2
            cond = 4500
        root = b**2 - 4*a*C

        if root < 0:
            # print("No intersection found")
            return None
        
        y1 = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)
        y2 = (-b - np.sqrt(b**2 - 4*a*c)) / (2*a)
        # print("y1:",y1,"y2",y2, "cond",cond)
        if abs(y1) > cond:
            # print("Chosen",y2)
            return y2
        elif abs(y2) > cond:
            # print("Chosen",y1)
            return y1
        else:
            # print(y1,y2)
            # print("Everything is out of the field")
            return None



    def detect_and_locate(self, image):
        markers_in_img = []
        ids = []
        markers_2D_coords = []
        for dictionar in dictionaries:
            dictionary = cv2.aruco.getPredefinedDictionary(dictionar)
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            markerCorners_dict, markerIds_dict, _ = detector.detectMarkers(image)
            if markerCorners_dict:
                for markerCorner, markerId in zip(markerCorners_dict, markerIds_dict):
                    if any(np.array_equal(markerCorner, mar[0]) for mar in markers_in_img):
                        continue
                    else:
                        if str(markerId[0]) in markers.keys():
                            markerLength = markers[str(markerId[0])][3]
                            # print("MarkerId:", markerId[0], ", Marker coordinates + length:", markers[str(markerId[0])]) #markerLength)
                            coords = markerCorner[0]
                            for coord in coords:
                                markers_2D_coords.append(coord)
                            ids.append(markerId[0])
                            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, markerLength, self.mtx, self.dist) #in meters bc markerLength is in meter
                            distance = np.linalg.norm(tvecs)*1000
                            # print("Distance:", distance)
                            marker_global_coordinates = markers[str(markerId[0])][:3]
                            # print("Marker global coord:", marker_global_coordinates)
                            markers_in_img.append([markerCorner, markers[str(markerId[0])], distance, markerId[0], tvecs]) #Pixel coordinates, 3D coordinates, ID, distance
                            
                            print("")
        if len(markers_in_img)>=2:
            # print("")
            # print("--------------")
            # print("")
            out = self.compute_triangulation(markers_in_img)
            # print("out:",out)
            if len(out) != 0:
                x, y, z = out
                return (x, y, z)
            else:
                return (None, None, None)
        
        elif len(markers_in_img) == 1:
            # print("")
            # print("--------------")
            # print("")
            # print("Only one marker!")
            _, one_markerdict, one_dist, _, one_tvecs = markers_in_img[0]
            if one_markerdict[4] == "wall" or one_markerdict[4] == "window":
                x_robot_respect_to_marker = one_tvecs[0][0][0]
                x_A, y_A, z_A = one_markerdict[:3]
                x_robot = x_A - x_robot_respect_to_marker
                # print(x_robot)
                y_robot = self.location_for_one_marker(x_A, y_A, z_A, x_robot_respect_to_marker, None, one_dist)
                if y_robot is not None:
                    # print("No inter")
                    return (None, None, None)
                
            elif one_markerdict[4] == "ducks" or one_markerdict[4] == "tables":
                y_robot_respect_to_marker = one_tvecs[0][0][1]
                x_A, y_A, z_A = one_markerdict[:3]
                y_robot = y_A - y_robot_respect_to_marker
                # print(y_robot)
                x_robot = self.location_for_one_marker(x_A, y_A, z_A, None, y_robot_respect_to_marker, one_dist)
                if x_robot is not None:
                    # print("No inter")
                    return (None, None, None)
            return (x_robot, y_robot, 0)

        
        else:
            print("No markers found")
            return (None, None, None)
        

        

    def process_image_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        debug = image.copy()
        # image = self.undistort(image)   
        # self.get_logger().info(f"Image received at {self.get_clock().now()}")
        blurred = cv2.GaussianBlur(image, (3, 3), 5)
        image = cv2.addWeighted(image, 1.5, blurred, -0.5, 0)
        
        debug = np.concatenate((debug, image), axis=1)

        (x,y,z) = self.detect_and_locate(image)
        self.get_logger().info(f"Robot located at: {x}, {y}, {z}")
        fig, ax = visualize_field(x, y)
        
                
        def select_duck(duck_locations, idx):
            return duck_locations[idx], duck_locations[:idx] + duck_locations[idx + 1 :]

        if x is not None:
            start = (x + A/2,y + B/2)

            # Select the first duck
            goal, obstacles = select_duck(duck_locations, 0)

            # If the robot is close to the duck, don't move or find paths
            if np.linalg.norm(np.array(start) - np.array(goal)) < 100:
                self.get_logger().info("Robot is close to the duck. Not moving.")
            else:
                pathfinder = PathFinder(40, 20, A, B)
                path_coordinates, debug_dict = pathfinder.find_path(start, goal, obstacles, distance_metric="euclidian", debug=True)
                self.get_logger().debug(f"Path coordinates: {debug_dict}")
                message = String()
                message.data = json.dumps(path_coordinates)
                self.publisher.publish(message)
                
                ax.scatter(*list(zip(*path_coordinates)), C / 2, "k", zorder=3)
        
        # Step 2: Save the plot to an in-memory buffer
        buf = BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)  # Rewind the buffer to the beginning

        # Step 3: Use OpenCV to decode the image from the buffer
        # Read the image as bytes and decode it using cv2
        img_bytes = np.frombuffer(buf.read(), dtype=np.uint8)  # Convert buffer to byte array
        img_array = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)  # Decode the byte array into an image (BGR format)


        # Resize the image to match the original image size (with padding)
        img_array = cv2.resize(img_array, (debug.shape[1], debug.shape[0]))
        
        # Concatenate the original image and the plot
        debug = np.concatenate((debug, img_array), axis=0)
        debug = cv2.resize(debug, (800, 600))

        # Optionally, display the image using OpenCV
        cv2.imshow('Matplotlib Plot', debug)
        cv2.waitKey(500)
        # plt.show(block=False)
        # plt.pause(0.5)
    
#  def undistort(self, image):

        

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()




### Utility functions


def visualize_field(xrobot, yrobot):
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.patch.set_facecolor("green")  # Set the background color to green

    # Draw the outer field
    outer_field = patches.Rectangle(
        (0, 0), A, B, linewidth=C / 10, edgecolor="white", facecolor="none"
    )
    ax.add_patch(outer_field)

    # Goal areas
    goal_area_left = patches.Rectangle(
        (0, (B - F) / 2), E, F, linewidth=C / 10, edgecolor="white", facecolor="none"
    )
    goal_area_right = patches.Rectangle(
        (A - E, (B - F) / 2),
        E,
        F,
        linewidth=C / 10,
        edgecolor="white",
        facecolor="none",
    )
    ax.add_patch(goal_area_left)
    ax.add_patch(goal_area_right)

    # Penalty areas
    penalty_area_left = patches.Rectangle(
        (0, (B - H) / 2), G, H, linewidth=C / 10, edgecolor="white", facecolor="none"
    )
    penalty_area_right = patches.Rectangle(
        (A - G, (B - H) / 2),
        G,
        H,
        linewidth=C / 10,
        edgecolor="white",
        facecolor="none",
    )
    ax.add_patch(penalty_area_left)
    ax.add_patch(penalty_area_right)

    # Penalty marks
    ax.plot(I, B / 2, "wo", markersize=D / 10)  # Left penalty mark
    ax.plot(A - I, B / 2, "wo", markersize=D / 10)  # Right penalty mark

    # Center circle
    center_circle = patches.Circle(
        (A / 2, B / 2), J / 2, linewidth=C / 10, edgecolor="white", facecolor="none"
    )
    ax.add_patch(center_circle)
    ax.plot(A / 2, B / 2, "wo", markersize=D / 10)  # Center mark

    # Halfway line
    ax.plot([A / 2, A / 2], [0, B], "white", linewidth=C / 10)

    # Plot the ducks
    # For some reason hides the center point
    ax.scatter(*list(zip(*duck_locations)), D, "yellow", zorder=2.5)

    # Robot position
    if xrobot is not None:
        ax.plot(int(xrobot) +A/2,int(yrobot) +B/2, "ro", zorder=5)
    else:
        print("Robot coudln't be located")

    # Plot the markers on the field
    for marker, (x, y, z, _, _) in markers.items():
        # Convert (x, y) to pixel coordinates
        x  # *= 1000
        y  # *=1000
        x_img = int((x + A / 2))
        y_img = int((y + B / 2))

        # Draw a blue circle for each marker
        ax.plot(x_img, y_img, "bo", markersize=5)
        ax.text(x_img, y_img-400, marker)

    # Set the limits and aspecs
    ax.set_xlim(-K, A + K)
    ax.set_ylim(-K, B + K)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")  # Turn off axe
    return fig, ax