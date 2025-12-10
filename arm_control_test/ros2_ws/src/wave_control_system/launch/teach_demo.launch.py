#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成示教功能演示的launch文件"""

    # 获取包路径
    wave_control_system_dir = get_package_share_directory('wave_control_system')

    return LaunchDescription([
        # 示教管理器节点
        Node(
            package='wave_control_system',
            executable='teach_manager',
            name='teach_manager',
            output='screen',
            parameters=[{
                'record_frequency': 20.0,
                'trajectory_dir': os.path.join(wave_control_system_dir, '..', '..', 'trajectories')
            }]
        ),

        # 电机驱动模拟器
        Node(
            package='wave_control_system',
            executable='motor_driver_sim',
            name='motor_driver_sim',
            output='screen'
        ),

        # 延迟启动示例轨迹生成
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'wave_control_system', 'wave_example'],
                    name='generate_wave_examples',
                    output='screen'
                )
            ]
        ),

        # 延迟启动测试
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'wave_control_system', 'test_teach_function'],
                    name='teach_function_test',
                    output='screen'
                )
            ]
        ),

        # 可选：启动GUI界面
        # TimerAction(
        #     period=8.0,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=['wave_gui'],
        #             name='wave_gui',
        #             output='screen'
        #         )
        #     ]
        # ),
    ])