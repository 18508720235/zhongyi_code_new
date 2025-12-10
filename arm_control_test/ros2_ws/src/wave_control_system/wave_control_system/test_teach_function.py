#!/usr/bin/env python3
"""
示教功能测试节点
用于测试记录和回放功能
"""

import rclpy
from rclpy.node import Node
from wave_control_msgs.srv import TeachControl
from wave_control_msgs.msg import TeachRecord, JointTrajectory, MotorCommand
import time
import threading
import json
import os


class TeachFunctionTester(Node):
    """示教功能测试器"""

    def __init__(self):
        super().__init__('teach_function_tester')

        # 客户端
        self.teach_client = self.create_client(TeachControl, 'teach_control')
        while not self.teach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teach_control service...')

        # 订阅者
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

        self.motor_cmd_sub = self.create_subscription(
            MotorCommand,
            'motor_commands',
            self.motor_cmd_callback,
            10
        )

        # 状态跟踪
        self.last_record_status = None
        self.test_results = []

        self.get_logger().info('Teach Function Tester initialized')

    def record_status_callback(self, msg):
        """处理记录状态回调"""
        self.last_record_status = msg
        self.get_logger().info(f'Record Status: {msg.state}, Frames: {msg.frame_count}, Duration: {msg.duration:.2f}s')

    def trajectory_callback(self, msg):
        """处理轨迹回调"""
        self.get_logger().info(f'Received trajectory: {len(msg.positions[0]) if msg.positions else 0} joints, {len(msg.timestamps)} frames')

    def motor_cmd_callback(self, msg):
        """处理电机命令回调"""
        if len(msg.positions) > 0:
            self.get_logger().debug(f'Motor command: IDs={msg.motor_ids}, Positions={[f"{p:.3f}" for p in msg.positions[:3]]}...')

    def send_teach_command(self, command, **kwargs):
        """发送示教命令"""
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

        future = self.teach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        return response

    def test_basic_recording(self):
        """测试基本记录功能"""
        self.get_logger().info('=== Testing Basic Recording ===')

        # 开始记录
        response = self.send_teach_command(
            "start_record",
            description="Basic recording test",
            record_frequency=10.0
        )

        if not response.success:
            self.get_logger().error(f'Failed to start recording: {response.message}')
            return False

        self.get_logger().info(f'Started recording: {response.message}')

        # 记录5秒
        self.get_logger().info('Recording for 5 seconds...')
        time.sleep(5)

        # 停止记录
        response = self.send_teach_command("stop_record")
        if not response.success:
            self.get_logger().error(f'Failed to stop recording: {response.message}')
            return False

        self.get_logger().info(f'Stopped recording: {response.message}, Frames: {response.frame_count}')

        # 保存轨迹
        response = self.send_teach_command(
            "save_trajectory",
            trajectory_id="test_recording",
            description="Test recording from basic test"
        )

        if response.success:
            self.get_logger().info(f'Saved trajectory: {response.message}')
        else:
            self.get_logger().error(f'Failed to save trajectory: {response.message}')

        return True

    def test_playback(self):
        """测试回放功能"""
        self.get_logger().info('=== Testing Playback ===')

        # 先尝试播放刚才录制的轨迹
        response = self.send_teach_command(
            "play_trajectory",
            trajectory_id="test_recording",
            play_speed=1.0,
            loop_playback=False
        )

        if not response.success:
            self.get_logger().error(f'Failed to start playback: {response.message}')

            # 尝试播放示例轨迹
            self.get_logger().info('Trying to play wave_example trajectory...')
            response = self.send_teach_command(
                "play_trajectory",
                trajectory_id="wave_example",
                play_speed=1.0,
                loop_playback=False
            )

            if not response.success:
                self.get_logger().error(f'Failed to play wave_example: {response.message}')
                return False

        self.get_logger().info(f'Started playback: {response.message}')

        # 观察回放10秒
        self.get_logger().info('Observing playback for 10 seconds...')
        start_time = time.time()

        while time.time() - start_time < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        return True

    def test_pause_resume(self):
        """测试暂停/恢复功能"""
        self.get_logger().info('=== Testing Pause/Resume ===')

        # 开始记录
        response = self.send_teach_command(
            "start_record",
            description="Pause/resume test",
            record_frequency=5.0
        )

        if not response.success:
            self.get_logger().error(f'Failed to start recording: {response.message}')
            return False

        # 记录2秒
        self.get_logger().info('Recording for 2 seconds...')
        time.sleep(2)

        # 暂停记录
        response = self.send_teach_command("pause_record")
        if response.success:
            self.get_logger().info(f'Paused recording: {response.message}')
        else:
            self.get_logger().error(f'Failed to pause recording: {response.message}')

        # 等待2秒（不记录）
        self.get_logger().info('Paused for 2 seconds...')
        time.sleep(2)

        # 恢复记录
        response = self.send_teach_command("resume_record")
        if response.success:
            self.get_logger().info(f'Resumed recording: {response.message}')
        else:
            self.get_logger().error(f'Failed to resume recording: {response.message}')

        # 再记录2秒
        self.get_logger().info('Recording for 2 more seconds...')
        time.sleep(2)

        # 停止记录
        response = self.send_teach_command("stop_record")
        if response.success:
            self.get_logger().info(f'Stopped recording: {response.message}')
        else:
            self.get_logger().error(f'Failed to stop recording: {response.message}')

        return True

    def test_loop_playback(self):
        """测试循环回放功能"""
        self.get_logger().info('=== Testing Loop Playback ===')

        response = self.send_teach_command(
            "play_trajectory",
            trajectory_id="wave_example",
            play_speed=1.5,
            loop_playback=True
        )

        if not response.success:
            self.get_logger().error(f'Failed to start loop playback: {response.message}')
            return False

        self.get_logger().info(f'Started loop playback: {response.message}')

        # 观察8秒循环回放
        self.get_logger().info('Observing loop playback for 8 seconds...')
        time.sleep(8)

        return True

    def test_trajectory_management(self):
        """测试轨迹管理功能"""
        self.get_logger().info('=== Testing Trajectory Management ===')

        # 加载示例轨迹
        response = self.send_teach_command(
            "load_trajectory",
            trajectory_id="circle_example"
        )

        if response.success:
            self.get_logger().info(f'Loaded circle_example: {response.message}')
        else:
            self.get_logger().error(f'Failed to load circle_example: {response.message}')

        # 检查轨迹文件是否存在
        trajectory_files = []
        trajectory_dir = "trajectories"
        if os.path.exists(trajectory_dir):
            trajectory_files = [f for f in os.listdir(trajectory_dir) if f.endswith('.json')]

        self.get_logger().info(f'Found trajectory files: {trajectory_files}')

        return True

    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('Starting Teach Function Tests...')

        tests = [
            ("Basic Recording", self.test_basic_recording),
            ("Playback", self.test_playback),
            ("Pause/Resume", self.test_pause_resume),
            ("Loop Playback", self.test_loop_playback),
            ("Trajectory Management", self.test_trajectory_management)
        ]

        results = {}

        for test_name, test_func in tests:
            self.get_logger().info(f'\n--- Running {test_name} Test ---')

            try:
                start_time = time.time()
                success = test_func()
                duration = time.time() - start_time

                results[test_name] = {
                    'success': success,
                    'duration': duration,
                    'message': 'PASSED' if success else 'FAILED'
                }

                status = 'PASS' if success else 'FAIL'
                self.get_logger().info(f'{test_name}: {status} (took {duration:.2f}s)')

            except Exception as e:
                results[test_name] = {
                    'success': False,
                    'duration': 0,
                    'message': f'Exception: {str(e)}'
                }
                self.get_logger().error(f'{test_name}: EXCEPTION - {str(e)}')

            # 测试间隔
            time.sleep(2)

        # 打印测试结果摘要
        self.print_test_summary(results)

        return results

    def print_test_summary(self, results):
        """打印测试结果摘要"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('TEACH FUNCTION TEST SUMMARY')
        self.get_logger().info('='*50)

        total_tests = len(results)
        passed_tests = sum(1 for r in results.values() if r['success'])

        for test_name, result in results.items():
            status = 'PASS' if result['success'] else 'FAIL'
            duration = result['duration']
            message = result['message']
            self.get_logger().info(f'{test_name}: {status} ({duration:.2f}s) - {message}')

        self.get_logger().info('-'*50)
        self.get_logger().info(f'Total: {passed_tests}/{total_tests} tests passed')

        if passed_tests == total_tests:
            self.get_logger().info('🎉 ALL TESTS PASSED!')
        else:
            self.get_logger().warning('⚠️  Some tests failed')


def main(args=None):
    rclpy.init(args=args)

    try:
        # 首先生成示例轨迹
        from wave_example import generate_gesture_trajectories, save_trajectories

        print("Generating example trajectories...")
        trajectories = generate_gesture_trajectories()
        save_trajectories(trajectories)
        print("Example trajectories generated.")

        # 创建测试器并运行测试
        tester = TeachFunctionTester()
        results = tester.run_all_tests()

    except KeyboardInterrupt:
        print('\nTest interrupted by user')
    except Exception as e:
        print(f'Test failed with exception: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()