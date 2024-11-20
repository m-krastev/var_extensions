from .constants import CMD_VEL_TOPIC, OUTPUT_TOPIC
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Adjust the message type based on your publisher's message type
from geometry_msgs.msg import Twist

class LineDetectionListener(Node):
    def __init__(self):
        super().__init__('marker_detection_listener')
        
        # Create a subscription to the topic where the publisher is publishing
        self.subscription = self.create_subscription(
            String,  # Replace with the appropriate message type
            OUTPUT_TOPIC,  # Replace with the actual topic name
            self.listener_callback,
            10  # Queue size (adjustable)
        )
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        coordinates = [(4500.0, 3000.0), (4275.0, 3000.0), (4050.0, 3000.0), (3825.0, 3000.0), (3600.0, 3000.0), (3375.0, 3000.0), (3150.0, 3000.0), (2925.0, 3000.0), (2925.0, 3300.0000000000005), (2700.0, 3300.0000000000005), (2700.0, 3600.0), (2475.0, 3600.0), (2475.0, 3900.0), (2250.0, 3900.0), (2025.0, 3900.0), (2025.0, 4200.0), (2025.0, 4500.0), (1800.0, 4500.0), (1800.0, 4800.0), (1575.0, 4800.0)]
        self.follow_coordinates(coordinates)
        
        
    def follow_coordinates(self, coordinates: list[tuple[float, float]]):
        last_change = 0 # 0 for x, 1 for y; in which direction the robot moved last
        for i in range(1, len(coordinates)):
            curr_x, curr_y = coordinates[i]
            prev_x, prev_y = coordinates[i - 1]
            # Check which one changed last
            if last_change == 0:
                if curr_x == prev_x:
                    last_change = 1
                    if curr_y > prev_y:
                        # up
                        direction = "right"
                    else:
                        # down
                        direction = "left"
                else:
                    direction = "forward"
            else:
                if curr_y == prev_y:
                    last_change = 0
                    if curr_x > prev_x:
                        # right
                        direction = "right"
                    else:
                        # left
                        direction = "left"
                else:
                    direction = "forward"
            twist = Twist()
            twist.linear.x = 1.0
            match direction:
                case "forward":
                    pass
                case "right":
                    twist.angular.z = -0.3
                case "left":
                    twist.angular.z = 0.3
            self.publisher.publish(twist)
            self.get_logger().info(f'Move {direction} to {curr_x}, {curr_y}')

    def listener_callback(self, msg):
        # This function is called whenever a message is received on the subscribed topic
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2

    listener = LineDetectionListener()  # Create the listener object

    # Spin the node so it can keep listening for incoming messages
    rclpy.spin(listener)

    # Cleanup and shutdown when node is done
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
