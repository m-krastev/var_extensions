from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_detection_pipeline',
            executable='line_detection_node',
            name='line_detection_node',
            output='screen'
        )
    ])