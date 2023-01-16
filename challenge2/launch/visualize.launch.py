import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='gnome-terminal -x'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/bot/ros2_ws/uvlarm-SOX/challenge2/config/slam_config.rviz']
        ),

        Node(
            package='tbot_pytools',
            executable='multiplexer',
            prefix='gnome-terminal -x'
        ),

    ])