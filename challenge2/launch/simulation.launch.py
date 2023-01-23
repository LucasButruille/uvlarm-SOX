import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    config_path = get_package_share_directory('challenge2')    
    tbot_sim_path = get_package_share_directory('tbot_sim')
    launch_file_dir = os.path.join(tbot_sim_path, 'launch','includes')
    nav_path = get_package_share_directory('nav2_bringup')


    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav_path, '/launch/navigation_launch.py']),            
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/challenge.launch.py']),
            launch_arguments={'world': 'challenge-2'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([config_path, '/mon_sync_launch.py']),
            launch_arguments = {'use_sim_time' : 'False'}.items(),   
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/bot/ros2_ws/uvlarm-SOX/challenge2/config/slam_config.rviz']
        ),

        Node(
            package='tuto_move',
            executable='scan_echo',
        ),

        Node(
            package='tuto_move',
            executable='reactive_move_simu',
        ),
        
        # Node(
        #     package='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     prefix='gnome-terminal -x',
        #     arguments=['/cmd_vel:=/multi/cmd_teleop'] 
        # ),

        # Node(
        #     package='tbot_pytools',
        #     executable='multiplexer',
        #     prefix='gnome-terminal -x'
        # ),

    ])