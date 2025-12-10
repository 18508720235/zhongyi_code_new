#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from wave_control_msgs.msg import JointTrajectory, TeachRecord, MotorCommand, WaveStatus
from wave_control_msgs.srv import TeachControl
import json
import threading
import time
import os
from datetime import datetime
import math
import numpy as np


class TeachManager(Node):
    """示教功能管理器"""

    def __init__(self):
        super().__init__('teach_manager')

        # 状态变量
        self.recording_state = "idle"  # idle, recording, paused, completed
        self.playback_state = "idle"   # idle, playing, paused
        self.current_trajectory = None
        self.current_record_id = None
        self.trajectory_data = []
        self.record_start_time = None
        self.playback_start_time = None
        self.record_frequency = 10.0  # Hz
        self.play_speed = 1.0
        self.loop_playback = False

        # 存储目录
        self.trajectory_dir = "trajectories"
        if not os.path.exists(self.trajectory_dir):
            os.makedirs(self.trajectory_dir)

        # 关节名称配置 (假设6个关节对应wave_control)
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

        # 服务
        self.teach_service = self.create_service(
            TeachControl,
            'teach_control',
            self.handle_teach_control
        )

        # 发布者
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10
        )

        self.record_status_pub = self.create_publisher(
            TeachRecord,
            'teach_record_status',
            10
        )

        # 订阅者 - 订阅电机反馈获取关节状态
        self.joint_state_sub = self.create_subscription(
            WaveStatus,
            'wave_status',  # 使用现有的状态话题获取关节信息
            self.joint_state_callback,
            10
        )

        self.motor_cmd_pub = self.create_publisher(
            MotorCommand,
            'motor_commands',  # 复用现有的电机命令话题
            10
        )

        # 定时器
        self.record_timer = None
        self.playback_timer = None

        # 线程锁
        self.lock = threading.Lock()

        self.get_logger().info('Teach Manager Node initialized')

    def create_joint_trajectory_msg(self, trajectory_data):
        """创建JointTrajectory消息的辅助函数"""
        if not trajectory_data:
            return None

        # 提取时间戳和位置数据
        timestamps = []
        positions = []

        for frame in trajectory_data:
            timestamps.append(frame['timestamp'])
            positions.extend(frame['positions'])  # 扁平化位置数据

        # 创建消息
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        traj_msg.positions = positions
        traj_msg.num_joints = len(self.joint_names)
        traj_msg.num_points = len(trajectory_data)
        traj_msg.timestamps = timestamps
        traj_msg.frame_id = "base_link"
        traj_msg.description = "Teach trajectory"

        return traj_msg

    def joint_state_callback(self, msg):
        """处理关节状态回调"""
        if self.recording_state == "recording":
            # 从WaveStatus中提取关节角度信息
            # 这里需要根据实际的消息结构调整
            current_time = time.time()

            # 假设从WaveStatus中可以获取到关节角度
            # 实际实现中可能需要从其他话题获取更详细的关节信息
            joint_positions = self.extract_joint_positions(msg)

            with self.lock:
                self.trajectory_data.append({
                    'timestamp': current_time,
                    'positions': joint_positions
                })

    def extract_joint_positions(self, wave_status_msg):
        """从WaveStatus消息中提取关节位置"""
        # 这里需要根据实际的WaveStatus消息结构来提取关节位置
        # 由于WaveStatus主要包含挥手状态，我们需要扩展它或使用其他话题

        # 临时使用模拟数据，实际应该从传感器话题获取
        if hasattr(wave_status_msg, 'current_amplitude'):
            # 简化处理：使用挥手幅度来模拟关节位置
            amplitude = wave_status_msg.current_amplitude
            # 生成6个关节的角度（简化示例）
            base_angle = amplitude * math.pi / 180.0
            positions = [
                base_angle,  # joint_1
                base_angle * 0.8,  # joint_2
                base_angle * 0.6,  # joint_3
                base_angle * 0.4,  # joint_4
                base_angle * 0.2,  # joint_5
                0.0   # joint_6
            ]
        else:
            positions = [0.0] * 6

        return positions

    def handle_teach_control(self, request, response):
        """处理示教控制服务请求"""
        command = request.command

        try:
            if command == "start_record":
                response = self.start_recording(request, response)
            elif command == "stop_record":
                response = self.stop_recording(response)
            elif command == "pause_record":
                response = self.pause_recording(response)
            elif command == "resume_record":
                response = self.resume_recording(response)
            elif command == "play_trajectory":
                response = self.play_trajectory(request, response)
            elif command == "save_trajectory":
                response = self.save_trajectory(request, response)
            elif command == "load_trajectory":
                response = self.load_trajectory(request, response)
            else:
                response.success = False
                response.message = f"Unknown command: {command}"

        except Exception as e:
            response.success = False
            response.message = f"Error processing command: {str(e)}"
            self.get_logger().error(f"Teach control error: {str(e)}")

        # 设置当前状态
        response.current_state = f"record:{self.recording_state}, playback:{self.playback_state}"
        response.frame_count = len(self.trajectory_data)
        response.progress = self.get_progress()

        return response

    def start_recording(self, request, response):
        """开始记录轨迹"""
        if self.recording_state == "recording":
            response.success = False
            response.message = "Already recording"
            return response

        with self.lock:
            self.trajectory_data = []
            self.recording_state = "recording"
            self.current_record_id = f"record_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            self.record_start_time = time.time()
            self.record_frequency = max(1.0, min(100.0, request.record_frequency))  # 限制频率范围

        # 创建记录定时器
        if self.record_timer:
            self.record_timer.cancel()
        self.record_timer = self.create_timer(
            1.0 / self.record_frequency,
            self.record_joint_state
        )

        response.success = True
        response.message = f"Started recording with frequency {self.record_frequency}Hz"

        # 发布记录状态
        self.publish_record_status()

        self.get_logger().info(f"Started recording trajectory: {self.current_record_id}")
        return response

    def stop_recording(self, response):
        """停止记录轨迹"""
        if self.recording_state != "recording" and self.recording_state != "paused":
            response.success = False
            response.message = "Not currently recording"
            return response

        if self.record_timer:
            self.record_timer.cancel()
            self.record_timer = None

        with self.lock:
            self.recording_state = "completed"
            duration = time.time() - self.record_start_time if self.record_start_time else 0.0

        response.success = True
        response.message = f"Stopped recording. Recorded {len(self.trajectory_data)} frames"

        # 发布记录状态
        self.publish_record_status()

        self.get_logger().info(f"Stopped recording. Total frames: {len(self.trajectory_data)}")
        return response

    def pause_recording(self, response):
        """暂停记录"""
        if self.recording_state != "recording":
            response.success = False
            response.message = "Not currently recording"
            return response

        self.recording_state = "paused"
        if self.record_timer:
            self.record_timer.cancel()

        response.success = True
        response.message = "Recording paused"

        # 发布记录状态
        self.publish_record_status()

        return response

    def resume_recording(self, response):
        """恢复记录"""
        if self.recording_state != "paused":
            response.success = False
            response.message = "Recording is not paused"
            return response

        self.recording_state = "recording"

        # 恢复定时器
        self.record_timer = self.create_timer(
            1.0 / self.record_frequency,
            self.record_joint_state
        )

        response.success = True
        response.message = "Recording resumed"

        # 发布记录状态
        self.publish_record_status()

        return response

    def play_trajectory(self, request, response):
        """播放轨迹"""
        if self.playback_state == "playing":
            response.success = False
            response.message = "Already playing trajectory"
            return response

        trajectory_id = request.trajectory_id
        if not trajectory_id and self.current_trajectory:
            # 播放当前录制的轨迹
            trajectory_data = self.trajectory_data
        else:
            # 加载指定的轨迹文件
            trajectory_file = os.path.join(self.trajectory_dir, f"{trajectory_id}.json")
            if not os.path.exists(trajectory_file):
                response.success = False
                response.message = f"Trajectory file not found: {trajectory_file}"
                return response

            with open(trajectory_file, 'r') as f:
                trajectory_file_data = json.load(f)

            # 从文件数据中提取frames
            trajectory_data = trajectory_file_data.get('frames', [])

            if not trajectory_data:
                response.success = False
                response.message = f"No frames found in trajectory file: {trajectory_file}"
                return response

        if not trajectory_data:
            response.success = False
            response.message = "No trajectory data to play"
            return response

        # 设置播放参数
        self.play_speed = max(0.1, min(10.0, request.play_speed))  # 限制播放速度
        self.loop_playback = request.loop_playback

        with self.lock:
            self.playback_state = "playing"
            self.playback_start_time = time.time()
            self.current_trajectory = trajectory_data

        # 创建播放定时器
        if self.playback_timer:
            self.playback_timer.cancel()
        self.playback_timer = self.create_timer(
            0.05,  # 20Hz播放频率
            self.execute_playback
        )

        response.success = True
        response.message = f"Started playing trajectory with speed {self.play_speed}x"

        self.get_logger().info(f"Started playing trajectory with {len(trajectory_data)} frames")
        return response

    def save_trajectory(self, request, response):
        """保存轨迹到文件"""
        if not self.trajectory_data:
            response.success = False
            response.message = "No trajectory data to save"
            return response

        trajectory_id = request.trajectory_id or self.current_record_id
        description = request.description or "Recorded trajectory"

        trajectory_data = {
            'id': trajectory_id,
            'description': description,
            'joint_names': self.joint_names,
            'record_frequency': self.record_frequency,
            'record_time': datetime.now().isoformat(),
            'frames': self.trajectory_data
        }

        file_path = os.path.join(self.trajectory_dir, f"{trajectory_id}.json")

        try:
            with open(file_path, 'w') as f:
                json.dump(trajectory_data, f, indent=2)

            response.success = True
            response.message = f"Trajectory saved to {file_path}"
            self.get_logger().info(f"Trajectory saved: {file_path}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to save trajectory: {str(e)}"
            self.get_logger().error(f"Failed to save trajectory: {str(e)}")

        return response

    def load_trajectory(self, request, response):
        """从文件加载轨迹"""
        trajectory_id = request.trajectory_id
        if not trajectory_id:
            response.success = False
            response.message = "Trajectory ID is required"
            return response

        file_path = os.path.join(self.trajectory_dir, f"{trajectory_id}.json")

        try:
            with open(file_path, 'r') as f:
                trajectory_data = json.load(f)

            with self.lock:
                self.current_trajectory = trajectory_data['frames']
                self.trajectory_data = trajectory_data['frames']

            response.success = True
            response.message = f"Trajectory loaded from {file_path}"
            self.get_logger().info(f"Trajectory loaded: {file_path}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to load trajectory: {str(e)}"
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")

        return response

    def record_joint_state(self):
        """定时记录关节状态"""
        if self.recording_state != "recording":
            return

        # 这里应该从实际的关节状态话题获取数据
        # 暂时使用模拟数据
        current_time = time.time()

        # 生成模拟的关节位置（用于测试）
        t = current_time - self.record_start_time if self.record_start_time else 0
        positions = [
            math.sin(t) * 30 * math.pi / 180,  # joint_1: 正弦波动
            math.cos(t) * 20 * math.pi / 180,  # joint_2: 余弦波动
            math.sin(t * 2) * 15 * math.pi / 180,  # joint_3: 快速正弦
            math.cos(t * 1.5) * 25 * math.pi / 180,  # joint_4
            math.sin(t * 0.5) * 10 * math.pi / 180,  # joint_5: 慢速正弦
            0.0  # joint_6: 固定
        ]

        with self.lock:
            self.trajectory_data.append({
                'timestamp': current_time,
                'positions': positions
            })

        # 发布记录状态
        self.publish_record_status()

    def execute_playback(self):
        """执行轨迹播放"""
        if self.playback_state != "playing" or not self.current_trajectory:
            return

        current_time = time.time()
        elapsed_time = (current_time - self.playback_start_time) * self.play_speed

        # 计算应该播放哪一帧
        if not self.current_trajectory:
            return

        # 获取第一帧的时间戳作为起始时间
        start_time = self.current_trajectory[0]['timestamp']
        trajectory_duration = self.current_trajectory[-1]['timestamp'] - start_time

        if elapsed_time >= trajectory_duration:
            if self.loop_playback:
                # 循环播放
                self.playback_start_time = current_time
                elapsed_time = 0.0
            else:
                # 播放完成
                self.stop_playback()
                return

        # 找到对应时间的帧
        target_time = start_time + elapsed_time

        # 线性插值找到目标关节位置
        target_positions = self.interpolate_trajectory(target_time)

        if target_positions:
            # 发送电机命令
            self.send_motor_commands(target_positions)

    def interpolate_trajectory(self, target_time):
        """在轨迹中插值获取目标时间的位置"""
        if not self.current_trajectory:
            return None

        # 找到前后两个帧
        prev_frame = None
        next_frame = None

        for frame in self.current_trajectory:
            if frame['timestamp'] <= target_time:
                prev_frame = frame
            elif frame['timestamp'] > target_time and next_frame is None:
                next_frame = frame
                break

        if prev_frame is None:
            return self.current_trajectory[0]['positions']
        if next_frame is None:
            return self.current_trajectory[-1]['positions']

        # 线性插值
        t1 = prev_frame['timestamp']
        t2 = next_frame['timestamp']
        alpha = (target_time - t1) / (t2 - t1) if t2 > t1 else 0.0

        prev_pos = prev_frame['positions']
        next_pos = next_frame['positions']

        interpolated = []
        for i in range(len(prev_pos)):
            interpolated.append(prev_pos[i] * (1 - alpha) + next_pos[i] * alpha)

        return interpolated

    def send_motor_commands(self, positions):
        """发送电机命令"""
        motor_cmd = MotorCommand()
        motor_cmd.motor_ids = list(range(1, len(positions) + 1))  # 电机ID从1开始
        motor_cmd.positions = positions
        motor_cmd.velocities = [30.0] * len(positions)  # 默认速度

        self.motor_cmd_pub.publish(motor_cmd)

    def stop_playback(self):
        """停止播放"""
        self.playback_state = "idle"
        if self.playback_timer:
            self.playback_timer.cancel()
            self.playback_timer = None

        # 发送停止命令
        self.send_stop_commands()

        self.get_logger().info("Trajectory playback completed")

    def send_stop_commands(self):
        """发送停止命令"""
        motor_cmd = MotorCommand()
        motor_cmd.motor_ids = [1, 2, 3, 4, 5, 6]  # 所有电机
        motor_cmd.positions = [0.0] * 6  # 回到初始位置
        motor_cmd.velocities = [10.0] * 6  # 慢速回位

        self.motor_cmd_pub.publish(motor_cmd)

    def publish_record_status(self):
        """发布记录状态"""
        status_msg = TeachRecord()
        status_msg.record_id = self.current_record_id or ""
        status_msg.state = self.recording_state
        status_msg.frame_count = len(self.trajectory_data)

        if self.record_start_time:
            status_msg.duration = time.time() - self.record_start_time
        else:
            status_msg.duration = 0.0

        status_msg.frequency = self.record_frequency
        status_msg.description = "Recording trajectory"

        self.record_status_pub.publish(status_msg)

    def get_progress(self):
        """获取当前进度"""
        if self.playback_state == "playing" and self.current_trajectory:
            current_time = time.time()
            elapsed_time = (current_time - self.playback_start_time) * self.play_speed

            # 检查current_trajectory是否是字典（从文件加载的）还是列表（直接记录的）
            if isinstance(self.current_trajectory, dict):
                frames = self.current_trajectory.get('frames', [])
            else:
                frames = self.current_trajectory

            if not frames:
                return 0.0

            start_time = frames[0]['timestamp']
            total_duration = frames[-1]['timestamp'] - start_time

            if total_duration > 0:
                return min(1.0, elapsed_time / total_duration)

        return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = TeachManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()