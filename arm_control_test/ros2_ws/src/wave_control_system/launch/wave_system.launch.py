from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('wave_control_system'),
        'config'
    )
    
    params_file = os.path.join(config_dir, 'wave_params.yaml')
    
    return LaunchDescription([
        # 声明launch参数
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation motor driver'
        ),
        
        # 挥手控制器节点
        Node(
            package='wave_control_system',
            executable='wave_controller',
            name='wave_controller',
            output='screen',
            parameters=[params_file],
            respawn=True,
            respawn_delay=2,
        ),
        
        # 电机驱动模拟节点（根据参数决定是否启动）
        Node(
            package='wave_control_system',
            executable='motor_driver_sim',
            name='motor_driver_sim',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),
        
        # 日志信息
        LogInfo(
            msg=['Wave Control System launched with sim=', 
                 LaunchConfiguration('use_sim')]
        ),
    ])
