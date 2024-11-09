import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal

from line_detection_pipeline.constants import CMD_VEL_TOPIC, LINE_DIRECTION_TOPIC


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower_node")
        self.subscription = self.create_subscription(
            Twist, LINE_DIRECTION_TOPIC, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.get_logger().info("Line Follower Node started.")

    def listener_callback(self, msg):
        # Process the direction message and send velocity commands
        velocity = Twist()
        velocity.linear.x = 0.25  # Move forward at a constant speed
        velocity.angular.z = - msg.angular.z * 0.3  # Adjust the turning rate based on the line direction
        self.publisher.publish(velocity)
        self.get_logger().info(
            f"Published velocity: linear.x={velocity.linear.x}, angular.z={velocity.angular.z}"
        )

    def stop_robot(self):
        # Stop the robot by publishing zero velocity
        self.get_logger().info("Stopping the robot.")
        stop_velocity = Twist()
        stop_velocity.linear.x = 0.0
        stop_velocity.angular.z = 0.0
        self.publisher.publish(stop_velocity)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C detected. Stopping the robot.")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
