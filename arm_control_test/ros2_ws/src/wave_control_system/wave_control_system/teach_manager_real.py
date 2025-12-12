#!/usr/bin/env python3
"""
支持真实机械臂的示教管理器
可以记录实际的关节状态而不是模拟数据
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from wave_control_msgs.msg import JointTrajectory, TeachRecord, WaveStatus
from wave_control_msgs.srv import TeachControl
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
import threading
import time
import os
from datetime import datetime
import math


class TeachManagerReal(Node):
    """支持真实机械臂的示教功能管理器"""

    def __init__(self):
        super().__init__('teach_manager_real')

        # 回调组：服务使用独立的组，避免被高频订阅阻塞
        self.service_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = ReentrantCallbackGroup()

        # 参数配置
        self.declare_parameter('joint_state_topic', '/joint_states_pub')
        self.declare_parameter('num_joints', 5)

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

        # 关节名称配置（匹配真机电机控制节点的命名）
        self.num_joints = self.get_parameter('num_joints').value
        # 真机电机的关节名称
        self.joint_names = ['ti5_1_0', 'ti5_1_1', 'ti5_2_0', 'ti5_2_1', 'encos']

        # 最新关节状态缓存
        self.latest_joint_state = None

        # 服务
        self.teach_service = self.create_service(
            TeachControl,
            'teach_control_real',
            self.handle_teach_control,
            callback_group=self.service_callback_group
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

        # 订阅真实关节状态
        joint_state_topic = self.get_parameter('joint_state_topic').value
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10,
            callback_group=self.subscription_callback_group
        )

        # 订阅WaveStatus（作为备用数据源）
        self.wave_status_sub = self.create_subscription(
            WaveStatus,
            'wave_status',
            self.wave_status_callback,
            10,
            callback_group=self.subscription_callback_group
        )

        # 电机命令发布者（使用标准 JointState 消息）
        self.motor_cmd_pub = self.create_publisher(
            JointState,
            'joint_states_sub',  # 真机接收话题
            10
        )

        # 播放控制订阅者（用于停止播放）
        self.playback_control_sub = self.create_subscription(
            String,
            'playback_control',
            self.playback_control_callback,
            10,
            callback_group=self.subscription_callback_group
        )

        # 停止标志
        self.stop_requested = False

        # 定时器
        self.record_timer = None
        self.playback_timer = None

        # 线程锁
        self.lock = threading.Lock()

        self.get_logger().info(f'Teach Manager Real Node initialized')
        self.get_logger().info(f'Listening to joint states on: {joint_state_topic}')

    def joint_state_callback(self, msg):
        """处理真实关节状态回调（带降采样到50Hz）"""
        self.latest_joint_state = msg

        # 动态更新关节数量和名称
        if len(msg.name) != self.num_joints:
            self.num_joints = len(msg.name)
            self.joint_names = msg.name
            self.get_logger().info(f'Updated joint names: {self.joint_names}')

        # 如果正在记录，保存当前关节状态（降采样到50Hz）
        if self.recording_state == "recording":
            current_time = time.time()
            
            # 降采样：只有距离上次记录超过1/50秒才记录
            if not hasattr(self, 'last_record_time'):
                self.last_record_time = 0.0
            
            time_since_last_record = current_time - self.last_record_time
            min_interval = 1.0 / 50.0  # 50Hz = 0.02秒间隔
            
            if time_since_last_record >= min_interval:
                with self.lock:
                    self.trajectory_data.append({
                        'timestamp': current_time,
                        'positions': list(msg.position),
                        'velocities': list(msg.velocity) if msg.velocity else [],
                        'efforts': list(msg.effort) if msg.effort else []
                    })
                self.last_record_time = current_time

    def wave_status_callback(self, msg):
        """处理WaveStatus回调（备用数据源）"""
        # 如果没有真实关节状态数据，使用WaveStatus生成关节角度
        if self.recording_state == "recording" and self.latest_joint_state is None:
            current_time = time.time()

            # 从WaveStatus的挥手信息生成关节角度
            if hasattr(msg, 'current_amplitude') and hasattr(msg, 'wave_part'):
                amplitude = msg.current_amplitude
                wave_part = msg.wave_part

                # 根据挥手部位生成不同的关节角度
                positions = self.generate_joint_angles_from_wave(amplitude, wave_part)

                with self.lock:
                    self.trajectory_data.append({
                        'timestamp': current_time,
                        'positions': positions,
                        'source': 'wave_status_simulation'
                    })

    def generate_joint_angles_from_wave(self, amplitude, wave_part):
        """从WaveStatus生成关节角度（备用方法）"""
        # 简化的关节角度生成逻辑
        base_angle = amplitude * math.pi / 180.0

        if wave_part == "right_arm":
            # 右臂关节角度
            positions = [
                base_angle,      # joint_1: 肩关节
                base_angle * 0.7, # joint_2: 肘关节
                base_angle * 0.5, # joint_3: 腕关节
                base_angle * 0.3, # joint_4
                base_angle * 0.2, # joint_5
                0.0              # joint_6
            ]
        elif wave_part == "left_arm":
            # 左臂关节角度
            positions = [
                0.0,              # joint_1
                base_angle * 0.2, # joint_2
                base_angle * 0.3, # joint_3
                base_angle * 0.5, # joint_4
                base_angle * 0.7, # joint_5
                base_angle        # joint_6
            ]
        else:  # both
            positions = [base_angle * 0.5] * self.num_joints

        return positions

    def playback_control_callback(self, msg):
        """处理播放控制命令"""
        if msg.data == "stop":
            self.stop_requested = True
            self.get_logger().info("Stop requested, will stop after current loop")

    def handle_teach_control(self, request, response):
        """处理示教控制服务请求"""
        command = request.command

        self.get_logger().info(f"[service] recv command={command}")

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

        self.get_logger().info(f"[service] done command={command}, success={response.success}, msg={response.message}")

        return response

    def start_recording(self, request, response):
        """开始记录轨迹"""
        if self.recording_state == "recording":
            response.success = False
            response.message = "Already recording"
            return response

        self.get_logger().info("[service] start_record begin")

        with self.lock:
            self.trajectory_data = []
            self.recording_state = "recording"
            self.current_record_id = f"record_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            self.record_start_time = time.time()
            self.record_frequency = max(1.0, min(100.0, request.record_frequency))

        # 创建记录定时器（实际关节状态是事件驱动的，不需要定时器记录）
        # 这里只需要一个状态发布定时器
        if self.record_timer:
            self.record_timer.cancel()
        self.record_timer = self.create_timer(
            0.1,  # 10Hz发布状态
            self.publish_record_status
        )

        # 检查是否有真实关节状态数据
        if self.latest_joint_state:
            self.get_logger().info(f"Started recording real joint states: {self.current_record_id}")
        else:
            self.get_logger().warn(f"No joint states received yet, using fallback: {self.current_record_id}")

        response.success = True
        response.message = f"Started recording with {len(self.joint_names)} joints"

        # 发布记录状态
        self.publish_record_status()

        self.get_logger().info("[service] start_record end")

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

        data_source = "real joint states" if self.latest_joint_state else "wave status simulation"
        self.get_logger().info(f"Stopped recording. {len(self.trajectory_data)} frames from {data_source}")

        response.success = True
        response.message = f"Stopped recording. Recorded {len(self.trajectory_data)} frames"

        # 发布记录状态
        self.publish_record_status()

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

        # 恢复状态发布定时器
        self.record_timer = self.create_timer(
            0.1,
            self.publish_record_status
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
        self.play_speed = max(0.1, min(10.0, request.play_speed))
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
        response.message = f"Started playing {len(trajectory_data)} frames with speed {self.play_speed}x"

        self.get_logger().info(f"Started playing trajectory with {len(trajectory_data)} frames")
        return response

    def save_trajectory(self, request, response):
        """保存轨迹到文件"""
        if not self.trajectory_data:
            response.success = False
            response.message = "No trajectory data to save"
            return response

        trajectory_id = request.trajectory_id or self.current_record_id
        description = request.description or "Recorded trajectory from real joints"

        # 确定数据来源
        data_source = "real_joint_states" if self.latest_joint_state else "wave_status_simulation"

        trajectory_data = {
            'id': trajectory_id,
            'description': description,
            'joint_names': self.joint_names,
            'num_joints': self.num_joints,
            'record_frequency': self.record_frequency,
            'record_time': datetime.now().isoformat(),
            'data_source': data_source,
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
                self.current_trajectory = trajectory_data.get('frames', [])
                self.trajectory_data = trajectory_data.get('frames', [])

                # 更新关节数据
                if 'joint_names' in trajectory_data:
                    self.joint_names = trajectory_data['joint_names']
                    self.num_joints = len(self.joint_names)

            response.success = True
            response.message = f"Trajectory loaded from {file_path}"
            self.get_logger().info(f"Trajectory loaded: {file_path}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to load trajectory: {str(e)}"
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")

        return response

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
                # 检查停止请求 - 在循环结束时检查
                if self.stop_requested:
                    self.get_logger().info("Stopping playback gracefully after completing loop")
                    self.stop_playback()
                    self.stop_requested = False
                    return
                
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
        """发送电机命令（使用标准 JointState 消息）"""
        motor_cmd = JointState()
        motor_cmd.header.stamp = self.get_clock().now().to_msg()
        motor_cmd.name = self.joint_names
        motor_cmd.position = positions
        motor_cmd.velocity = [0.5] * len(positions)  # 安全速度：0.5 rad/s
        motor_cmd.effort = [0.0] * len(positions)   # 必须提供 effort 字段

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
        """发送停止命令（使用标准 JointState 消息）"""
        motor_cmd = JointState()
        motor_cmd.header.stamp = self.get_clock().now().to_msg()
        motor_cmd.name = self.joint_names
        motor_cmd.position = [0.0] * self.num_joints  # 回到初始位置
        motor_cmd.velocity = [0.2] * self.num_joints  # 慢速回位
        motor_cmd.effort = [0.0] * self.num_joints

        self.motor_cmd_pub.publish(motor_cmd)

    def publish_record_status(self):
        """发布记录状态"""
        if self.recording_state in ["recording", "paused"]:
            status_msg = TeachRecord()
            status_msg.record_id = self.current_record_id or ""
            status_msg.state = self.recording_state
            status_msg.frame_count = len(self.trajectory_data)

            if self.record_start_time:
                status_msg.duration = time.time() - self.record_start_time
            else:
                status_msg.duration = 0.0

            status_msg.frequency = self.record_frequency
            status_msg.description = f"Recording from {'real joints' if self.latest_joint_state else 'simulation'}"

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
    node = TeachManagerReal()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()