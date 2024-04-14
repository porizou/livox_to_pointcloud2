from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_to_pointcloud2',
            executable='livox_to_pointcloud2_node',
            name='livox_to_pointcloud2_node'
        )
    ])
