#!/usr/bin/env python3
"""
模拟关节状态发布器 - 用于虚拟机测试
发布模拟的关节状态数据到 /joint_states_pub
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class MockJointPublisher(Node):
    """模拟关节状态发布器"""

    def __init__(self):
        super().__init__('mock_joint_publisher')
        
        # 发布器
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states_pub',
            10
        )
        
        # 定时器：20Hz 发布频率
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        # 关节名称
        self.joint_names = ['joint1_ti5', 'joint2_ti5', 'joint3_ti5', 'joint4_ti5', 'joint5_encos']
        
        # 时间计数器
        self.start_time = time.time()
        
        self.get_logger().info('Mock Joint Publisher started')
        self.get_logger().info(f'Publishing to /joint_states_pub at 20Hz')

    def publish_joint_states(self):
        """发布模拟的关节状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # 生成模拟的正弦波关节位置
        t = time.time() - self.start_time
        
        msg.position = [
            0.5 * math.sin(t * 0.5),           # joint1: 慢速摆动
            0.3 * math.sin(t * 0.8 + 1.0),     # joint2
            0.4 * math.sin(t * 0.6 + 2.0),     # joint3
            0.2 * math.sin(t * 1.0 + 3.0),     # joint4
            0.3 * math.sin(t * 0.7 + 4.0),     # joint5
        ]
        
        # 速度：位置的导数
        msg.velocity = [
            0.5 * 0.5 * math.cos(t * 0.5),
            0.3 * 0.8 * math.cos(t * 0.8 + 1.0),
            0.4 * 0.6 * math.cos(t * 0.6 + 2.0),
            0.2 * 1.0 * math.cos(t * 1.0 + 3.0),
            0.3 * 0.7 * math.cos(t * 0.7 + 4.0),
        ]
        
        # 力矩：设为0
        msg.effort = [0.0] * 5
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockJointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
