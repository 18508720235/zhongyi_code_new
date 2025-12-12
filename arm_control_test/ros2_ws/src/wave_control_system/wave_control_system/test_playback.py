#!/usr/bin/env python3
"""
轨迹回放测试节点 - 支持单轨迹和序列播放
"""

import rclpy
from rclpy.node import Node
from wave_control_msgs.srv import TeachControl
from std_msgs.msg import String
import os
import time
import threading
import json
import signal


class TrajectoryPlaybackTester(Node):
    """轨迹回放测试器（支持单轨迹和序列播放）"""

    def __init__(self):
        super().__init__('trajectory_playback_tester')

        # 客户端 - 连接到真实示教服务
        self.teach_client = self.create_client(TeachControl, 'teach_control_real')
        while not self.teach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teach_control_real service...')

        # 停止标志
        self.stop_requested = False

        # 设置Ctrl+C信号处理
        signal.signal(signal.SIGINT, self.signal_handler)

        # 停止话题订阅
        self.stop_sub = self.create_subscription(
            String,
            'playback_control',
            self.stop_callback,
            10
        )

        self.get_logger().info('Trajectory Playback Tester initialized')

    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号"""
        if not self.stop_requested:
            self.get_logger().info("\n收到 Ctrl+C，将在完成当前循环后继续...")
            self.stop_requested = True
        else:
            # 第二次Ctrl+C强制退出
            self.get_logger().info("\n强制退出")
            raise KeyboardInterrupt

    def stop_callback(self, msg):
        """停止回调"""
        if msg.data == "stop":
            self.stop_requested = True
            self.get_logger().info("Stop requested")

    def send_teach_command(self, command, **kwargs):
        """发送示教命令"""
        request = TeachControl.Request()
        request.command = command

        # 设置可选参数
        if 'trajectory_id' in kwargs:
            request.trajectory_id = kwargs['trajectory_id']
        if 'play_speed' in kwargs:
            request.play_speed = kwargs['play_speed']
        if 'loop_playback' in kwargs:
            request.loop_playback = kwargs['loop_playback']

        # 使用call_async()发送请求
        future = self.teach_client.call_async(request)
        
        # 等待响应
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

    def play_segment(self, trajectory_id, play_speed=1.0, loop=False):
        """播放单个轨迹段"""
        self.get_logger().info(f"Playing: {trajectory_id}")
        
        response = self.send_teach_command(
            "play_trajectory",
            trajectory_id=trajectory_id,
            play_speed=play_speed,
            loop_playback=loop
        )

        if response is None or not response.success:
            self.get_logger().error(f"Failed to play: {trajectory_id}")
            return False

        return True

    def get_trajectory_duration(self, trajectory_id):
        """从轨迹文件获取时长"""
        try:
            traj_file = f"trajectories/{trajectory_id}.json"
            if not os.path.exists(traj_file):
                self.get_logger().warn(f"轨迹文件不存在: {traj_file}，使用默认3秒")
                return 3.0
            
            with open(traj_file, 'r') as f:
                data = json.load(f)
            
            frames = data.get('frames', [])
            if len(frames) < 2:
                return 3.0
            
            # 计算时长：最后一帧时间 - 第一帧时间
            duration = frames[-1]['timestamp'] - frames[0]['timestamp']
            return max(duration, 1.0)  # 至少1秒
            
        except Exception as e:
            self.get_logger().warn(f"读取轨迹时长失败: {e}，使用默认3秒")
            return 3.0

    def run_single_playback(self):
        """单轨迹播放"""
        # 列出可用的轨迹文件
        trajectories_dir = "trajectories"
        if os.path.exists(trajectories_dir):
            files = [f.replace('.json', '') for f in os.listdir(trajectories_dir) if f.endswith('.json')]
            if files:
                self.get_logger().info('可用的轨迹文件:')
                for i, f in enumerate(files, 1):
                    self.get_logger().info(f'  {i}. {f}')
            else:
                self.get_logger().warn('未找到轨迹文件')
                return
        else:
            self.get_logger().warn('trajectories 目录不存在')
            return

        # 用户输入
        trajectory_name = input('\n请输入要回放的轨迹名称: ').strip()
        if not trajectory_name:
            self.get_logger().error('未输入轨迹名称')
            return

        speed_input = input('请输入回放速度倍数 (默认1.0): ').strip()
        try:
            play_speed = float(speed_input) if speed_input else 1.0
        except ValueError:
            play_speed = 1.0

        loop_input = input('是否循环播放? (y/n, 默认n): ').strip().lower()
        loop_playback = (loop_input == 'y')

        # 播放
        self.get_logger().info(f'\n开始回放: {trajectory_name} (速度={play_speed}x, 循环={loop_playback})')
        
        response = self.send_teach_command(
            "play_trajectory",
            trajectory_id=trajectory_name,
            play_speed=play_speed,
            loop_playback=loop_playback
        )

        if response and response.success:
            self.get_logger().info(f'✓ {response.message}')
            if loop_playback:
                self.get_logger().info('\n循环播放中... 发送stop停止或按Enter退出')
            input('按 Enter 键退出...')
        else:
            self.get_logger().error('播放失败')

    def run_sequence_playback(self):
        """序列播放（3段）"""
        self.get_logger().info('\n=== 序列播放配置 ===')
        
        seg1 = input("第一段轨迹名称（抬起，默认 raise_arm）: ").strip() or "raise_arm"
        seg2 = input("第二段轨迹名称（摆手，默认 wave_motion）: ").strip() or "wave_motion"
        seg3 = input("第三段轨迹名称（回零，默认 return_zero）: ").strip() or "return_zero"
        
        loop_input = input("是否循环第二段？(y/n, 默认 y): ").strip().lower()
        loop_middle = (loop_input != 'n')

        self.get_logger().info(f'\n配置:')
        self.get_logger().info(f'  段1: {seg1}')
        self.get_logger().info(f'  段2: {seg2} (循环={loop_middle})')
        self.get_logger().info(f'  段3: {seg3}')
        input('\n按 Enter 开始...')

        # 第一段：抬起
        self.get_logger().info('\n[1/3] 播放抬起段...')
        if not self.play_segment(seg1, play_speed=1.0, loop=False):
            return
        time.sleep(2.0)  # 等待完成

        if self.stop_requested:
            self.get_logger().info("收到停止信号，跳转到第三段")
            self.run_segment_3(seg3)
            return

        # 第二段：摆手  
        self.get_logger().info('\n[2/3] 播放摆手段...')
        
        if loop_middle:
            # 循环播放
            self.play_segment(seg2, play_speed=1.0, loop=True)
            self.get_logger().info('循环播放中... 按Enter、Ctrl+C或发送stop继续到第三段')
            
            # 创建线程监听Enter键
            enter_pressed = threading.Event()
            def wait_for_enter():
                input()
                enter_pressed.set()
                self.stop_requested = True
            
            enter_thread = threading.Thread(target=wait_for_enter, daemon=True)
            enter_thread.start()
            
            # 等待停止信号（Enter或ROS话题）
            while not self.stop_requested:
                rclpy.spin_once(self, timeout_sec=0.5)
            
            # 发送停止命令到teach_manager
            self.get_logger().info('发送停止信号...')
            stop_pub = self.create_publisher(String, 'playback_control', 10)
            time.sleep(0.1)  # 等待发布器初始化
            
            stop_msg = String()
            stop_msg.data = "stop"
            stop_pub.publish(stop_msg)
            
            # 获取第二段轨迹的实际时长
            seg2_duration = self.get_trajectory_duration(seg2)
            self.get_logger().info(f'等待当前循环完成（约{seg2_duration:.1f}秒）...')
            time.sleep(seg2_duration + 0.5)  # 轨迹时长 + 0.5秒缓冲
        else:
            # 只播放一次
            self.play_segment(seg2, play_speed=1.0, loop=False)
            time.sleep(2.0)

        # 第三段：回零
        self.run_segment_3(seg3)
        self.get_logger().info('\n=== 序列播放完成 ===')

    def run_segment_3(self, seg3_name):
        """播放第三段（回零）"""
        self.get_logger().info('\n[3/3] 播放回零段...')
        self.play_segment(seg3_name, play_speed=1.0, loop=False)
        time.sleep(2.0)

    def run_playback(self):
        """主入口：选择播放模式"""
        self.get_logger().info('=== 轨迹播放系统 ===\n')
        
        mode = input('选择播放模式:\n  1 - 单轨迹播放\n  2 - 序列播放（3段）\n请选择 (1/2, 默认1): ').strip()
        
        if mode == '2':
            self.run_sequence_playback()
        else:
            self.run_single_playback()


def main(args=None):
    rclpy.init(args=args)
    tester = TrajectoryPlaybackTester()

    try:
        tester.run_playback()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
