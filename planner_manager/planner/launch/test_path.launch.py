import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    test_path_node = Node(
        package='planner',
        executable='test_path_publisher',
        name='test_path_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        test_path_node
    ])