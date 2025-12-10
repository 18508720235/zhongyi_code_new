#!/usr/bin/env python3
"""
挥手示例轨迹生成器
用于生成示教功能的示例轨迹
"""

import json
import math
import os
from datetime import datetime


def generate_wave_trajectory():
    """生成挥手动作的轨迹数据"""

    # 轨迹参数
    duration = 3.0  # 总时长(秒)
    frequency = 2.0  # 挥手频率(Hz)
    amplitude = 45.0  # 挥手幅度(度)
    sample_rate = 20  # 采样频率(Hz)

    # 关节名称
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    # 生成轨迹帧
    frames = []
    num_samples = int(duration * sample_rate)

    for i in range(num_samples):
        t = i / sample_rate  # 当前时间

        # 生成挥手动作的关节角度
        # 假设挥手主要涉及前3个关节

        # joint_1: 主要的挥手关节 (肩关节)
        angle1 = amplitude * math.sin(2 * math.pi * frequency * t)

        # joint_2: 辅助关节 (肘关节)
        angle2 = amplitude * 0.3 * math.sin(2 * math.pi * frequency * t + math.pi/4)

        # joint_3: 细微调整 (腕关节)
        angle3 = amplitude * 0.1 * math.sin(2 * math.pi * frequency * t + math.pi/2)

        # joint_4-6: 保持固定或微小动作
        angle4 = amplitude * 0.05 * math.sin(2 * math.pi * frequency * t * 0.5)
        angle5 = amplitude * 0.03 * math.cos(2 * math.pi * frequency * t * 0.3)
        angle6 = 0.0  # 固定

        # 转换为弧度
        positions = [
            angle1 * math.pi / 180.0,
            angle2 * math.pi / 180.0,
            angle3 * math.pi / 180.0,
            angle4 * math.pi / 180.0,
            angle5 * math.pi / 180.0,
            angle6
        ]

        frame = {
            'timestamp': t,
            'positions': positions
        }
        frames.append(frame)

    # 创建轨迹数据
    trajectory_data = {
        'id': 'wave_example',
        'description': 'Wave motion example trajectory',
        'joint_names': joint_names,
        'record_frequency': sample_rate,
        'record_time': datetime.now().isoformat(),
        'parameters': {
            'duration': duration,
            'frequency': frequency,
            'amplitude': amplitude,
            'sample_rate': sample_rate
        },
        'frames': frames
    }

    return trajectory_data


def generate_gesture_trajectories():
    """生成多种手势的示例轨迹"""

    trajectories = {}

    # 1. 挥手轨迹
    trajectories['wave_example'] = generate_wave_trajectory()

    # 2. 点头轨迹
    nod_trajectory = {
        'id': 'nod_example',
        'description': 'Nodding motion example',
        'joint_names': ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        'record_frequency': 15,
        'record_time': datetime.now().isoformat(),
        'frames': []
    }

    duration = 2.0
    sample_rate = 15
    num_samples = int(duration * sample_rate)

    for i in range(num_samples):
        t = i / sample_rate
        # 点头动作主要在joint_1
        angle = 15 * math.sin(math.pi * t)  # 单周期点头
        positions = [
            angle * math.pi / 180.0,  # joint_1
            0.0,  # joint_2
            0.0,  # joint_3
            0.0,  # joint_4
            0.0,  # joint_5
            0.0   # joint_6
        ]

        nod_trajectory['frames'].append({
            'timestamp': t,
            'positions': positions
        })

    trajectories['nod_example'] = nod_trajectory

    # 3. 画圆轨迹
    circle_trajectory = {
        'id': 'circle_example',
        'description': 'Circular motion example',
        'joint_names': ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        'record_frequency': 20,
        'record_time': datetime.now().isoformat(),
        'frames': []
    }

    duration = 4.0
    sample_rate = 20
    num_samples = int(duration * sample_rate)
    radius = 30  # 半径(度)

    for i in range(num_samples):
        t = i / sample_rate
        angle = 2 * math.pi * t / duration  # 完成一圈

        # 在joint_1和joint_2平面画圆
        pos1 = radius * math.cos(angle)
        pos2 = radius * math.sin(angle)

        positions = [
            pos1 * math.pi / 180.0,  # joint_1
            pos2 * math.pi / 180.0,  # joint_2
            0.0,  # joint_3
            0.0,  # joint_4
            0.0,  # joint_5
            0.0   # joint_6
        ]

        circle_trajectory['frames'].append({
            'timestamp': t,
            'positions': positions
        })

    trajectories['circle_example'] = circle_trajectory

    return trajectories


def save_trajectories(trajectories, output_dir="trajectories"):
    """保存轨迹到文件"""

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    saved_files = []

    for name, trajectory in trajectories.items():
        filename = os.path.join(output_dir, f"{name}.json")

        try:
            with open(filename, 'w') as f:
                json.dump(trajectory, f, indent=2)
            saved_files.append(filename)
            print(f"Saved trajectory: {filename}")
        except Exception as e:
            print(f"Failed to save {filename}: {e}")

    return saved_files


def print_trajectory_info(trajectory):
    """打印轨迹信息"""
    print(f"\n=== Trajectory Info ===")
    print(f"ID: {trajectory['id']}")
    print(f"Description: {trajectory['description']}")
    print(f"Joint Names: {trajectory['joint_names']}")
    print(f"Record Frequency: {trajectory['record_frequency']} Hz")
    print(f"Total Frames: {len(trajectory['frames'])}")

    if 'parameters' in trajectory:
        print(f"Parameters: {trajectory['parameters']}")

    # 显示前几帧和后几帧
    frames = trajectory['frames']
    print(f"\nFirst frame (t={frames[0]['timestamp']:.3f}s):")
    print(f"  Positions: {[f'{p:.4f}' for p in frames[0]['positions']]}")

    print(f"\nLast frame (t={frames[-1]['timestamp']:.3f}s):")
    print(f"  Positions: {[f'{p:.4f}' for p in frames[-1]['positions']]}")


if __name__ == "__main__":
    print("Generating example trajectories for teach pendant...")

    # 生成多种手势轨迹
    trajectories = generate_gesture_trajectories()

    # 保存轨迹文件
    saved_files = save_trajectories(trajectories)

    # 显示轨迹信息
    for name, trajectory in trajectories.items():
        print_trajectory_info(trajectory)

    print(f"\n=== Summary ===")
    print(f"Generated {len(trajectories)} example trajectories")
    print(f"Saved files: {saved_files}")
    print("\nYou can now use these trajectories with the teach_control service:")
    print("  - ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command: \"play_trajectory\", trajectory_id: \"wave_example\"}'")