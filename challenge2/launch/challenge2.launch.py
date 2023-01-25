from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    tbot_start_path = get_package_share_directory('tbot_start')
    tbot_start_launch_dir = os.path.join(tbot_start_path, 'launch')
    nav_path = '/home/bot/ros2_ws/uvlarm-SOX/challenge2'

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav_path, '/launch/mon_navigation_launch.py']),            
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([tbot_start_launch_dir, '/minimal.launch.py'])
        ),

        Node(
            package='tuto_move',
            executable='scan_echo',
        ),

        Node(
            package='tuto_move',
            executable='reactive_move',
        ),

        Node(
            package='tuto_vision',
            executable='camera_image',
        ),

        Node(
            package='tbot_pytools',
            executable='multiplexer',
            prefix='gnome-terminal -x'
        ),

    ])