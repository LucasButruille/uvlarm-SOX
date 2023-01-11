from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='gnome-terminal -x'
        ),
        
        Node(
            package='tbot_start',
            executable='start_base',
        ),

        Node(
            package='ros1_bridge',
            executable='dynamic_bridge',
        ),

        Node(
            package='tbot_pytools',
            executable='multiplexer',
            prefix='gnome-terminal -x'
        ),

        Node(
            package='urg_node',
            executable='urg_node_driver',
            arguments = ['-p', 'serial_port:=/dev/ttyACM0']
        ),
    ])
