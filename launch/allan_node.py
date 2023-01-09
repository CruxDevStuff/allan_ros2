import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('allan_ros2'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='allan_ros2',
            executable='allan_node',
            parameters=[config]
        )
    ])