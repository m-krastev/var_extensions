from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_capture',
            executable='image_capture_node',
            name='image_capture_node',
            output='screen'
        )
    ])