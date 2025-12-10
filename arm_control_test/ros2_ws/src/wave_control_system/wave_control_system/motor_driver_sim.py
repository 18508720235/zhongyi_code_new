#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from wave_control_msgs.msg import MotorCommand, WaveStatus

class MotorDriverSim(Node):
    def __init__(self):
        super().__init__('motor_driver_sim')
        
        # 订阅电机命令
        self.create_subscription(
            MotorCommand,
            'motor_commands',
            self.motor_command_callback,
            10
        )
        
        # 发布传感器反馈
        self.feedback_pub = self.create_publisher(
            WaveStatus,
            'sensor_feedback',
            10
        )
        
        self.get_logger().info('Motor Driver Simulator initialized')
        
    def motor_command_callback(self, msg):
        """处理电机命令"""
        self.get_logger().info(
            f'Received motor command: IDs={msg.motor_ids}, '
            f'Positions={msg.positions}, Velocities={msg.velocities}'
        )
        
        # 模拟执行并发送反馈
        feedback = WaveStatus()
        
        # 根据位置判断动作
        if msg.positions[0] > 0:
            feedback.current_action = "raising"
        elif msg.positions[0] == 0:
            feedback.current_action = "lowering"
        else:
            feedback.current_action = "stopped"
            
        feedback.current_amplitude = abs(msg.positions[0]) if msg.positions else 0.0
        feedback.current_frequency = 1.0  # 模拟值
        
        # 模拟一些延迟
        self.create_timer(0.1, lambda: self.feedback_pub.publish(feedback))

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
