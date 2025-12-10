#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """生成真实机械臂示教功能的launch文件"""

    return LaunchDescription([
        # 真实示教管理器节点
        Node(
            package='wave_control_system',
            executable='teach_manager_real',
            name='teach_manager_real',
            output='screen',
            parameters=[{
                'joint_state_topic': '/joint_states_pub',
                'num_joints': 5
            }]
        )
    ])
