from .constants import OUTPUT_TOPIC
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Adjust the message type based on your publisher's message type

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
        self.subscription  # prevent unused variable warning

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
