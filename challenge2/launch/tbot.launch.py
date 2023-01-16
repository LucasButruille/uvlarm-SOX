import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    tbot_start_path = get_package_share_directory('tbot_start')
    tbot_start_launch_dir = os.path.join(tbot_start_path, 'launch')

    return LaunchDescription([
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
    ])