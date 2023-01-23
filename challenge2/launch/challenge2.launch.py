from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_path = get_package_share_directory('challenge2')    
    tbot_start_path = get_package_share_directory('tbot_start')
    tbot_start_launch_dir = os.path.join(tbot_start_path, 'launch')
    nav_path = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav_path, '/launch/navigation_launch.py']),            
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([config_path, '/mon_sync_launch.py']),
            launch_arguments = {'use_sim_time' : 'False'}.items(),
            
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

    ])