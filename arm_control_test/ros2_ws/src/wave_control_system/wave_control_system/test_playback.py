#!/usr/bin/env python3
"""
轨迹回放测试节点 - 加载并回放已保存的轨迹
"""

import rclpy
from rclpy.node import Node
from wave_control_msgs.srv import TeachControl
import os


class TrajectoryPlaybackTester(Node):
    """轨迹回放测试器"""

    def __init__(self):
        super().__init__('trajectory_playback_tester')

        # 客户端 - 连接到真实示教服务
        self.teach_client = self.create_client(TeachControl, 'teach_control_real')
        while not self.teach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teach_control_real service...')

        self.get_logger().info('Trajectory Playback Tester initialized')

    def send_teach_command(self, command, **kwargs):
        """发送示教命令，使用call_async()配合rclpy.spin_until_future_complete()"""
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

    def run_playback(self):
        """运行回放流程（交互式）"""
        self.get_logger().info('=== Trajectory Playback Test ===')

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

        # 用户输入轨迹名称
        trajectory_name = input('\n请输入要回放的轨迹名称（不含.json后缀）: ').strip()
        if not trajectory_name:
            self.get_logger().error('未输入轨迹名称')
            return

        # 用户输入回放速度
        speed_input = input('请输入回放速度倍数 (默认1.0): ').strip()
        try:
            play_speed = float(speed_input) if speed_input else 1.0
        except ValueError:
            self.get_logger().warn('无效的速度值，使用默认1.0')
            play_speed = 1.0

        # 是否循环播放
        loop_input = input('是否循环播放? (y/n, 默认n): ').strip().lower()
        loop_playback = (loop_input == 'y')

        # 开始播放
        self.get_logger().info(f'\n开始回放轨迹: {trajectory_name}')
        self.get_logger().info(f'回放速度: {play_speed}x')
        self.get_logger().info(f'循环播放: {"是" if loop_playback else "否"}')
        
        response = self.send_teach_command(
            "play_trajectory",
            trajectory_id=trajectory_name,
            play_speed=play_speed,
            loop_playback=loop_playback
        )

        if response is None:
            self.get_logger().error('Failed to get response from service')
            return

        if not response.success:
            self.get_logger().error(f'Failed to play trajectory: {response.message}')
            return

        self.get_logger().info(f'✓ 回放已开始: {response.message}')
        
        if not loop_playback:
            self.get_logger().info('\n回放进行中...')
            input('回放完成后按 Enter 键退出...')
        else:
            self.get_logger().info('\n循环回放中...')
            input('按 Enter 键停止循环回放并退出...')

        self.get_logger().info('=== Playback Test Completed ===')


def main(args=None):
    rclpy.init(args=args)
    tester = TrajectoryPlaybackTester()

    tester.run_playback()

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
