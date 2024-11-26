import json
import time
from .constants import CMD_VEL_TOPIC, OUTPUT_TOPIC
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class LineDetectionListener(Node):
    def __init__(self):
        super().__init__("marker_detection_listener")

        # Create a subscription to the topic where the publisher is publishing
        self.subscription = self.create_subscription(
            String,  # Message type
            OUTPUT_TOPIC,  # Receives info from output topic
            self.listener_callback, # When receives info, calls listener_callback
            1,  # Queue size (adjustable)
        )
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10) #Sends velocity commands 
        self.get_logger().info("Listener node has been initialized")

    def listener_callback(self, msg):
        # This function is called whenever a message is received on the subscribed topic
        # self.get_logger().info(f"Received message: {msg.data}")
        # self.publisher.publish(Twist())

        # Parse the message and do something with it
        msg = json.loads(msg.data) # Converts json-> python
        self.get_logger().info(f"Received message: {msg}")
        if msg == "stop": # If message is stop, the program finishes
            exit()
        self.follow_coordinates(msg) # Otherwise, it calls follow_coordinates

    def follow_coordinates(self, coordinates: list[tuple[float, float]]):
        last_change = 0  # 0 for x, 1 for y; in which direction the robot moved last
        for i in range(1, len(coordinates)):
            curr_x, curr_y = coordinates[i]
            prev_x, prev_y = coordinates[i - 1]
            # Check which one changed last
            if last_change == 0: # Last thing u did was move in x (straight)
                if curr_x == prev_x: # If now you dont need to move in x
                    if curr_y > prev_y: # You need to increase your Y coordinate 
                        # up
                        last_change = 1 # You say last was a change in y for the next 
                        direction = "right"
                    else:
                        # down
                        last_change = -1 # You say last was a change in y for the next 
                        direction = "left"
                else: # If you need to change your x coordinate is bc you need to go straight
                    direction = "forward"

            elif last_change == -1: # Last thing u did was turn left
                if curr_y == prev_y: # If you need to keep moving to the goal
                    last_change = 0 # You say last was a change in x for the next 
                    if curr_x > prev_x: # To go back to straight, you turn right 
                        # right
                        direction = "right" #You are looking to +Y, so to go to +X need to turn right
                    else:
                        # left
                        direction = "left"
                else: # You need to keep advancing in -Y
                    direction = "forward"

            else: # last change = 1 bc u turned right 
                if curr_y == prev_y: # If you need to keep moving to the goal
                    last_change = 0 # You say last was a change in x for the next 
                    if curr_x > prev_x: # To go back to straight, you turn right 
                        # right
                        direction = "left" #You are looking to +Y, so to go to +X need to turn left
                    else:
                        # left
                        direction = "right"
                else: # You need to keep advancing in +Y
                    direction = "forward"

            twist = Twist()
            twist.linear.x = 1.0
            match direction:
                case "forward":
                    pass
                case "right":
                    twist.angular.z = -0.6
                case "left":
                    twist.angular.z = 0.6
            self.publisher.publish(twist)
            time.sleep(0.5)
            self.get_logger().info(f"Move {direction} to {curr_x}, {curr_y}")
        self.get_logger().info("Reached the end of the path")


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2

    listener = LineDetectionListener()  # Create the listener object

    coordinates = [
        (4500.0, 3000.0),
        (4275.0, 3000.0),
        (4050.0, 3000.0),
        (3825.0, 3000.0),
        (3600.0, 3000.0),
        (3375.0, 3000.0),
        (3150.0, 3000.0),
        (2925.0, 3000.0),
        (2925.0, 3300.0000000000005),
        (2700.0, 3300.0000000000005),
        (2700.0, 3600.0),
        (2475.0, 3600.0),
        (2475.0, 3900.0),
        (2250.0, 3900.0),
        (2025.0, 3900.0),
        (2025.0, 4200.0),
        (2025.0, 4500.0),
        (1800.0, 4500.0),
        (1800.0, 4800.0),
        (1575.0, 4800.0),
        (1400, 5000),
        (1200, 5000),
        (1000, 5000),
        (800, 5000),
        (600, 5000),
        (400, 5000),
        (200, 5000),
        (0, 5000),
    ]
    # listener.follow_coordinates(coordinates)

    # Spin the node so it can keep listening for incoming messages
    rclpy.spin(listener)

    # Cleanup and shutdown when node is done
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
