#!/usr/bin/env python3
"""
生成正弦波轨迹 - 用于测试回放功能
生成5个关节的正弦波运动轨迹，保存为JSON文件
"""

import json
import math
import os
from datetime import datetime


def generate_sine_trajectory(
    duration=10.0,      # 总时长（秒）
    frequency=50.0,     # 采样频率（Hz）
    amplitude=30.0,     # 幅度（度）
    period=5.0,         # 正弦波周期（秒）
    num_joints=5        # 关节数量
):
    """
    生成正弦波轨迹
    
    参数:
        duration: 轨迹总时长（秒）
        frequency: 采样频率（Hz）
        amplitude: 正弦波幅度（度，会转换为弧度）
        period: 正弦波周期（秒）
        num_joints: 关节数量
    """
    
    # 关节名称（匹配真机）
    joint_names = ['ti5_1_0', 'ti5_1_1', 'ti5_2_0', 'ti5_2_1', 'encos']
    
    # 计算总帧数
    num_frames = int(duration * frequency)
    dt = 1.0 / frequency
    
    # 生成轨迹数据
    frames = []
    start_time = 1765268120.0  # 模拟起始时间戳
    
    for i in range(num_frames):
        t = i * dt
        timestamp = start_time + t
        
        # 为每个关节生成不同相位的正弦波
        positions = []
        velocities = []
        
        for j in range(num_joints):
            # 相位偏移：每个关节相位不同，使运动更自然
            phase = j * (2 * math.pi / num_joints)
            
            # 不同幅度：让各关节运动幅度略有不同
            joint_amplitude = amplitude * (1.0 - j * 0.1)
            
            # 位置：正弦波（转换为弧度）
            angle_deg = joint_amplitude * math.sin(2 * math.pi * t / period + phase)
            angle_rad = math.radians(angle_deg)
            positions.append(angle_rad)
            
            # 速度：位置的导数
            velocity = joint_amplitude * (2 * math.pi / period) * math.cos(2 * math.pi * t / period + phase)
            velocity_rad = math.radians(velocity)
            velocities.append(velocity_rad)
        
        frames.append({
            'timestamp': timestamp,
            'positions': positions,
            'velocities': velocities,
            'efforts': [0.0] * num_joints
        })
    
    # 构造完整的轨迹数据
    trajectory_data = {
        'id': 'sine_wave_test',
        'description': f'正弦波测试轨迹 - 周期{period}s, 幅度{amplitude}度',
        'joint_names': joint_names[:num_joints],
        'num_joints': num_joints,
        'record_frequency': frequency,
        'record_time': datetime.now().isoformat(),
        'data_source': 'generated_sine_wave',
        'frames': frames
    }
    
    return trajectory_data


def main():
    """主函数"""
    print("=== 正弦波轨迹生成器 ===\n")
    
    # 用户输入参数
    print("请输入轨迹参数（直接按Enter使用默认值）：\n")
    
    duration_input = input("轨迹时长（秒，默认10）: ").strip()
    duration = float(duration_input) if duration_input else 10.0
    
    amplitude_input = input("摆动幅度（度，默认30）: ").strip()
    amplitude = float(amplitude_input) if amplitude_input else 30.0
    
    period_input = input("正弦波周期（秒，默认5）: ").strip()
    period = float(period_input) if period_input else 5.0
    
    # 生成轨迹
    print(f"\n生成轨迹中...")
    print(f"  时长: {duration}秒")
    print(f"  幅度: {amplitude}度")
    print(f"  周期: {period}秒")
    print(f"  频率: 50Hz")
    
    trajectory = generate_sine_trajectory(
        duration=duration,
        amplitude=amplitude,
        period=period
    )
    
    # 保存到文件
    trajectories_dir = "trajectories"
    if not os.path.exists(trajectories_dir):
        os.makedirs(trajectories_dir)
    
    filename = f"{trajectories_dir}/sine_wave_test.json"
    
    with open(filename, 'w') as f:
        json.dump(trajectory, f, indent=2)
    
    print(f"\n✓ 轨迹已保存: {filename}")
    print(f"  总帧数: {len(trajectory['frames'])}")
    print(f"  关节数: {trajectory['num_joints']}")
    print(f"\n可以使用以下命令回放:")
    print(f"  ros2 run wave_control_system test_playback")
    print(f"  然后输入轨迹名称: sine_wave_test")


if __name__ == '__main__':
    main()
