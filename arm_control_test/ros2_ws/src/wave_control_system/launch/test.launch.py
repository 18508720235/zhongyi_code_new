from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wave_control_system',
            executable='wave_controller',
            name='wave_controller',
            output='screen'
        )
    ])
