from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='tuto_vision',
            executable='camera_image',
        ),
        
        Node(
            package='tuto_vision',
            executable='vision',
        ),

    ])