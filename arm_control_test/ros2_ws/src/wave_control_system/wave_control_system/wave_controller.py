#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from wave_control_msgs.msg import WaveCommand, MotorCommand, WaveStatus
from wave_control_msgs.srv import WaveControl
import json
from datetime import datetime
import math

class WaveController(Node):
    def __init__(self):
        super().__init__('wave_controller')
        
        # 参数初始化
        self.declare_parameter('default_amplitude', 45.0)
        self.declare_parameter('default_frequency', 1.0)
        self.declare_parameter('max_amplitude', 90.0)
        self.declare_parameter('min_amplitude', 10.0)
        
        # 状态变量
        self.is_waving = False
        self.current_wave_part = "right_arm"
        self.current_amplitude = self.get_parameter('default_amplitude').value
        self.current_frequency = self.get_parameter('default_frequency').value
        
        # 服务
        self.wave_service = self.create_service(
            WaveControl, 
            'wave_control', 
            self.handle_wave_control
        )
        
        # 发布者
        self.motor_cmd_pub = self.create_publisher(
            MotorCommand, 
            'motor_commands', 
            10
        )
        
        self.status_pub = self.create_publisher(
            WaveStatus,
            'wave_status',
            10
        )
        
        # 订阅者
        self.create_subscription(
            WaveStatus,
            'sensor_feedback',
            self.sensor_feedback_callback,
            10
        )
        
        # 定时器用于挥手动作
        self.wave_timer = None
        self.wave_phase = 0  # 0: raising, 1: lowering
        
        self.get_logger().info('Wave Controller Node initialized')
        
    def validate_parameters(self, command):
        """参数合法性校验"""
        max_amp = self.get_parameter('max_amplitude').value
        min_amp = self.get_parameter('min_amplitude').value
        
        if command.amplitude < min_amp or command.amplitude > max_amp:
            return False, f"Amplitude must be between {min_amp} and {max_amp}"
            
        if command.frequency < 0.1 or command.frequency > 5.0:
            return False, "Frequency must be between 0.1 and 5.0 Hz"
            
        if command.wave_part not in ["left_arm", "right_arm", "both"]:
            return False, "Invalid wave part"
            
        return True, "Parameters valid"
        
    def handle_wave_control(self, request, response):
        """处理挥手控制服务请求"""
        command = request.command
        
        # 参数校验
        is_valid, msg = self.validate_parameters(command)
        
        if not is_valid:
            response.success = False
            response.message = msg
            return response
            
        # 根据操作类型处理
        if command.operation_type == "wave_hello":
            response = self.start_waving(command, response)
        elif command.operation_type == "change_state":
            response = self.change_wave_state(command, response)
        elif command.operation_type == "stop":
            response = self.stop_waving(response)
        else:
            response.success = False
            response.message = "Unknown operation type"
            
        # 设置当前状态
        response.current_status = self.get_current_status()
        
        return response
        
    def start_waving(self, command, response):
        """开始挥手"""
        if self.is_waving:
            response.success = False
            response.message = "Already waving"
            return response
            
        self.current_wave_part = command.wave_part
        self.current_amplitude = command.amplitude
        self.current_frequency = command.frequency
        self.is_waving = True
        
        # 创建定时器执行挥手动作
        period = 1.0 / (2.0 * self.current_frequency)  # 半个周期
        self.wave_timer = self.create_timer(period, self.execute_wave_motion)
        
        response.success = True
        response.message = "Started waving"
        
        # 记录日志
        self.log_wave_action("START", command)
        
        return response
        
    def change_wave_state(self, command, response):
        """改变挥手状态"""
        if not self.is_waving:
            response.success = False
            response.message = "Not currently waving"
            return response
            
        # 更新参数
        self.current_wave_part = command.wave_part
        self.current_amplitude = command.amplitude
        self.current_frequency = command.frequency
        
        # 更新定时器频率
        if self.wave_timer:
            self.wave_timer.cancel()
            period = 1.0 / (2.0 * self.current_frequency)
            self.wave_timer = self.create_timer(period, self.execute_wave_motion)
            
        response.success = True
        response.message = "Wave state changed"
        
        # 记录日志
        self.log_wave_action("CHANGE", command)
        
        return response
        
    def stop_waving(self, response):
        """停止挥手"""
        if not self.is_waving:
            response.success = False
            response.message = "Not currently waving"
            return response
            
        self.is_waving = False
        
        if self.wave_timer:
            self.wave_timer.cancel()
            self.wave_timer = None
            
        # 发送停止命令到电机
        self.send_stop_command()
        
        response.success = True
        response.message = "Stopped waving"
        
        # 记录日志
        self.log_wave_action("STOP", None)
        
        return response
        
    def execute_wave_motion(self):
        """执行挥手动作"""
        if not self.is_waving:
            return
            
        motor_cmd = MotorCommand()
        
        # 根据挥手部位设置电机ID
        if self.current_wave_part == "left_arm":
            motor_cmd.motor_ids = [1, 2, 3]  # 左臂电机ID
        elif self.current_wave_part == "right_arm":
            motor_cmd.motor_ids = [4, 5, 6]  # 右臂电机ID
        else:  # both
            motor_cmd.motor_ids = [1, 2, 3, 4, 5, 6]
            
        # 计算目标位置
        if self.wave_phase == 0:  # raising
            target_position = self.current_amplitude
            self.wave_phase = 1
            action = "raising"
        else:  # lowering
            target_position = 0.0
            self.wave_phase = 0
            action = "lowering"
            
        motor_cmd.positions = [target_position] * len(motor_cmd.motor_ids)
        motor_cmd.velocities = [30.0] * len(motor_cmd.motor_ids)  # 默认速度
        
        # 发布电机命令
        self.motor_cmd_pub.publish(motor_cmd)
        
        # 发布状态
        status = WaveStatus()
        status.current_action = action
        status.current_frequency = self.current_frequency
        status.current_amplitude = self.current_amplitude
        status.wave_part = self.current_wave_part
        self.status_pub.publish(status)
        
    def send_stop_command(self):
        """发送停止命令"""
        motor_cmd = MotorCommand()
        motor_cmd.motor_ids = [1, 2, 3, 4, 5, 6]  # 所有电机
        motor_cmd.positions = [0.0] * 6  # 回到初始位置
        motor_cmd.velocities = [10.0] * 6  # 慢速回位
        self.motor_cmd_pub.publish(motor_cmd)
        
    def sensor_feedback_callback(self, msg):
        """处理传感器反馈"""
        # 根据反馈调整参数
        if self.is_waving:
            # 这里可以添加动态调整逻辑
            self.get_logger().debug(f'Sensor feedback: {msg.current_action}')
            
    def get_current_status(self):
        """获取当前状态"""
        status = WaveStatus()
        status.current_action = "waving" if self.is_waving else "stopped"
        status.current_frequency = self.current_frequency
        status.current_amplitude = self.current_amplitude
        status.wave_part = self.current_wave_part
        return status
        
    def log_wave_action(self, action_type, command):
        """记录挥手动作日志"""
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "action_type": action_type,
            "robot_id": command.robot_id if command else "N/A",
            "wave_part": self.current_wave_part,
            "amplitude": self.current_amplitude,
            "frequency": self.current_frequency
        }
        
        # 这里可以写入文件或发送到日志服务
        self.get_logger().info(f'Wave log: {json.dumps(log_entry)}')

def main(args=None):
    rclpy.init(args=args)
    node = WaveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
