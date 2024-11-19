from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='curling_pipeline',
            executable='marker_detection_node',
            name='marker_detection_node',
            output='screen'
        )
    ])