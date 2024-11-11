import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from line_detection_pipeline.constants import (
    CMD_VEL_TOPIC,
    LINE_DIRECTION_TOPIC,
    highprofile,
)

RHALFPI = 2 / np.pi


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
        turn = msg.angular.z  # slope
        
        # ArcTan function to find the angle of the line (-pi/2 to pi/2)
        # Divide by the slope so that the robot only turns when the line is not straight
        # Multiple by 3 to smooth the function
        # Divide by π/2 to convert to rad/s
        # Multiply by 0.3 to limit the turn speed
        # Negative sign to move away from side lines (or lines with low slope) since we want to stay in the middle.
        velocity.angular.z = (
            - 0.3 * np.arctan(3 / turn) * RHALFPI
        )
        
        self.publisher.publish(velocity)
        self.get_logger().info(
            f"Speed: \t{velocity.linear.x:.2f}m/s \t Turn: \t{velocity.angular.z:.4f} rad/s"
        )

    def stop_robot(self):
        # Stop the robot by publishing zero velocity
        self.publisher.publish(Twist())


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
