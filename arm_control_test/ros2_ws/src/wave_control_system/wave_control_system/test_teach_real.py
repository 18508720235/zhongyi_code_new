#!/usr/bin/env python3
"""
真实机械臂示教功能测试节点
只做录制与保存，不做回放
"""

import rclpy
from rclpy.node import Node
# SingleThreadedExecutor removed - using synchronous call instead
from wave_control_msgs.srv import TeachControl
from wave_control_msgs.msg import TeachRecord, JointTrajectory, MotorCommand
import time
import json
import os


class TeachRealTester(Node):
    """真实示教功能测试器（同步调用，避免线程死锁）"""

    def __init__(self):
        super().__init__('teach_real_tester')

        # 客户端 - 连接到真实示教服务
        self.teach_client = self.create_client(TeachControl, 'teach_control_real')
        while not self.teach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teach_control_real service...')

        # 订阅者（可选，仅用于观察状态）
        self.record_status_sub = self.create_subscription(
            TeachRecord,
            'teach_record_status',
            self.record_status_callback,
            10
        )

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.last_record_status = None
        self.get_logger().info('Teach Real Tester initialized')

    def record_status_callback(self, msg):
        """处理记录状态回调"""
        self.last_record_status = msg
        if msg.frame_count % 20 == 0:
            self.get_logger().info(f'Record Status: {msg.state}, Frames: {msg.frame_count}, Duration: {msg.duration:.2f}s')

    def trajectory_callback(self, msg):
        """处理轨迹回调"""
        self.get_logger().info(f'Received trajectory: {len(msg.positions[0]) if msg.positions else 0} joints, {len(msg.timestamps)} frames')

    def send_teach_command(self, command, **kwargs):
        """发送示教命令，使用call_async()配合rclpy.spin_until_future_complete()"""
        request = TeachControl.Request()
        request.command = command

        # 设置可选参数
        if 'trajectory_id' in kwargs:
            request.trajectory_id = kwargs['trajectory_id']
        if 'description' in kwargs:
            request.description = kwargs['description']
        if 'record_frequency' in kwargs:
            request.record_frequency = kwargs['record_frequency']
        if 'play_speed' in kwargs:
            request.play_speed = kwargs['play_speed']
        if 'loop_playback' in kwargs:
            request.loop_playback = kwargs['loop_playback']

        # 使用call_async()发送请求
        future = self.teach_client.call_async(request)
        
        # 使用rclpy.spin_until_future_complete()等待响应（不创建新executor）
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                return future.result()
            except Exception as e:
                self.get_logger().error(f"Service call exception for command: {command}: {e}")
                return None
        else:
            self.get_logger().error(f"Service call timed out for command: {command}")
            return None

    def run_test(self):
        """运行测试流程（交互式）"""
        self.get_logger().info('=== Starting Real Robot Teach Test (Interactive Mode) ===')

        # 1. 等待用户确认开始
        self.get_logger().info('准备开始示教录制...')
        input('按 Enter 键开始录制...')
        
        # 开始记录
        self.get_logger().info('1. Starting recording...')
        response = self.send_teach_command(
            "start_record",
            description="Real robot interactive recording",
            record_frequency=50.0  # 设置为50Hz
        )

        if response is None:
            self.get_logger().error('Failed to get response from service')
            return

        if not response.success:
            self.get_logger().error(f'Failed to start recording: {response.message}')
            if "Already recording" not in response.message:
                return
        self.get_logger().info(f'✓ 录制已开始: {response.message}')

        # 2. 等待用户操作完成
        self.get_logger().info('')
        self.get_logger().info('正在录制中... 请手动操作机械臂')
        input('完成示教动作后，按 Enter 键停止录制...')
        self.get_logger().info('')

        # 3. 停止记录
        self.get_logger().info('3. Stopping recording...')
        response = self.send_teach_command("stop_record")

        if response is None:
            self.get_logger().error('Failed to get response from service')
            return

        if not response.success:
            self.get_logger().error(f'Failed to stop recording: {response.message}')
            return
        self.get_logger().info(f'Stop recording success: {response.message}')

        # 4. 保存轨迹
        self.get_logger().info('4. Saving trajectory...')
        
        # 让用户输入轨迹名称
        trajectory_name = input('请输入轨迹名称（将自动添加 .json 后缀）: ').strip()
        if not trajectory_name:
            trajectory_name = "trajectory_" + time.strftime("%Y%m%d_%H%M%S")
            self.get_logger().info(f'未输入名称，使用默认名称: {trajectory_name}')
        
        response = self.send_teach_command("save_trajectory", trajectory_id=trajectory_name)

        if response is None:
            self.get_logger().error('Failed to get response from service')
            return

        if not response.success:
            self.get_logger().error(f'Failed to save trajectory: {response.message}')
            return
        self.get_logger().info(f'Save trajectory success: {response.message}')

        self.get_logger().info('=== Test Completed (Recording Only) ===')


def main(args=None):
    rclpy.init(args=args)
    tester = TeachRealTester()

    tester.run_test()

    tester.destroy_node()
    rclpy.shutdown()
