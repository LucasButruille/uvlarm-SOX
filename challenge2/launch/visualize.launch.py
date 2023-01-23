from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

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

    ])