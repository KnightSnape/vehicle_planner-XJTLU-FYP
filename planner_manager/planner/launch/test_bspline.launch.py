import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planner',
            executable='traj_server',
            name='traj_server',
            output='screen',
        ),
        
        Node(
            package='planner',
            executable='test_bspline',
            name='test_bspline',
            output='screen',
        )
    ])