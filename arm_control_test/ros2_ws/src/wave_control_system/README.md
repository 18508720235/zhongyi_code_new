# Wave Control System

机器人挥手控制系统，基于ROS2实现。

## 功能特性
- 支持左臂、右臂、双臂挥手
- 可调节挥手幅度和频率
- 实时状态反馈
- 参数合法性校验
- 操作日志记录

## 安装依赖
`
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
pip3 install PyQt5  # 如果需要GUI
`

## 编译
`
cd ~/ros2_ws
colcon build --packages-select wave_control_msgs wave_control_system
source install/setup.bash
`

## 运行
`
# 启动系统
ros2 launch wave_control_system wave_system.launch.py

# 运行客户端
ros2 run wave_control_system wave_client

# 运行GUI（可选）
ros2 run wave_control_system wave_gui
`

## 服务接口
- /wave_control - 挥手控制服务

## 话题接口
- /motor_commands - 电机控制命令
- /wave_status - 挥手状态
- /sensor_feedback - 传感器反馈
