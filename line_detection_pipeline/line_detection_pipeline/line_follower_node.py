import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower_node")
        self.subscription = self.create_subscription(
            Twist, "line_direction", self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.get_logger().info("Line Follower Node started.")

    def listener_callback(self, msg):
        # Process the direction message and send velocity commands
        velocity = Twist()
        velocity.linear.x = 0.5  # Move forward at a constant speed
        velocity.angular.z = (
            -msg.angular.z * 0.01
        )  # Adjust the turning rate based on the line direction
        self.publisher.publish(velocity)
        self.get_logger().info(
            f"Published velocity: linear.x={velocity.linear.x}, angular.z={velocity.angular.z}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
