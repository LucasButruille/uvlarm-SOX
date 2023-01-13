import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    tbot_sim_path = get_package_share_directory('tbot_sim')
    launch_file_dir = os.path.join(tbot_sim_path, 'launch','includes')

    return LaunchDescription([
        
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='gnome-terminal -x'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/challenge.launch.py']),
            launch_arguments={'world': 'challenge-1'}.items(),
            ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/bot/ros2_ws/uvlarm-SOX/tuto_sim/config/configRviz2.rviz']
        ),

        Node(
            package='tbot_pytools',
            executable='multiplexer',
            prefix='gnome-terminal -x'
        )

    ])
