# 示教功能使用指南

## 概述

本项目为wave_control_system添加了完整的示教功能，支持记录和回放机械臂轨迹。示教功能包括：

- **轨迹记录**：实时记录机械臂关节运动轨迹
- **轨迹回放**：精确回放录制的轨迹
- **轨迹管理**：保存、加载和管理多个轨迹文件
- **示例轨迹**：预置挥手、点头、画圆等示例动作

## 功能特性

### 1. 示教记录功能
- 支持可调节的记录频率（1-100 Hz）
- 实时显示记录状态和进度
- 支持暂停/恢复记录
- 自动生成记录ID和时间戳

### 2. 示教回放功能
- 支持可调节的播放速度（0.1x-10x）
- 支持循环播放模式
- 线性插值保证轨迹平滑
- 实时电机命令发布

### 3. 轨迹管理
- JSON格式存储轨迹数据
- 支持轨迹描述和元数据
- 轨迹文件的保存和加载
- 示例轨迹自动生成

## 安装和构建

```bash
# 进入ROS2工作空间
cd ~/arm_control2_test/ros2_ws

# 构建项目
colcon build --packages-select wave_control_msgs wave_control_system

# 刷新环境
source install/setup.bash
```

## 使用方法

### 1. 启动示教系统

```bash
# 启动完整的演示系统
ros2 launch wave_control_system teach_demo.launch.py

# 或者只启动示教管理器
ros2 run wave_control_system teach_manager
```

### 2. 生成示例轨迹

```bash
# 生成挥手、点头、画圆等示例轨迹
ros2 run wave_control_system wave_example
```

### 3. 运行功能测试

```bash
# 运行完整的示教功能测试
ros2 run wave_control_system test_teach_function
```

### 4. 手动控制示教功能

#### 开始记录轨迹
```bash
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "start_record", description: "手动记录测试", record_frequency: 20.0}'
```

#### 停止记录
```bash
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "stop_record"}'
```

#### 保存轨迹
```bash
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "save_trajectory", trajectory_id: "my_wave", description: "我的挥手动作"}'
```

#### 播放轨迹
```bash
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "play_trajectory", trajectory_id: "wave_example", play_speed: 1.0, loop_playback: false}'
```

## 话题和服务

### 话题

- `/teach_control` (service) - 示教控制服务
- `/teach_record_status` (TeachRecord) - 记录状态发布
- `/joint_trajectory` (JointTrajectory) - 轨迹数据发布
- `/motor_commands` (MotorCommand) - 电机命令发布

### 服务

**TeachControl服务参数：**

- `command`: 操作命令
  - `"start_record"` - 开始记录
  - `"stop_record"` - 停止记录
  - `"pause_record"` - 暂停记录
  - `"resume_record"` - 恢复记录
  - `"play_trajectory"` - 播放轨迹
  - `"save_trajectory"` - 保存轨迹
  - `"load_trajectory"` - 加载轨迹

- `trajectory_id`: 轨迹标识符
- `description`: 轨迹描述
- `record_frequency`: 记录频率（Hz）
- `play_speed`: 播放速度倍数
- `loop_playback`: 是否循环播放

## 文件结构

```
wave_control_system/
├── wave_control_system/
│   ├── teach_manager.py          # 示教功能主节点
│   ├── wave_example.py           # 示例轨迹生成器
│   ├── test_teach_function.py    # 功能测试节点
│   └── ...
├── launch/
│   └── teach_demo.launch.py      # 演示启动文件
└── trajectories/                 # 轨迹文件存储目录
    ├── wave_example.json
    ├── nod_example.json
    └── circle_example.json
```

## 轨迹文件格式

轨迹文件以JSON格式存储：

```json
{
  "id": "wave_example",
  "description": "Wave motion example",
  "joint_names": ["joint_1", "joint_2", ...],
  "record_frequency": 20,
  "record_time": "2024-01-01T12:00:00",
  "frames": [
    {
      "timestamp": 0.0,
      "positions": [0.1, 0.2, 0.3, ...]
    },
    ...
  ]
}
```

## 示例场景

### 1. 挥手示教

```bash
# 1. 开始记录挥手动作
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "start_record", description: "记录挥手动作"}'

# 2. 手动移动机械臂进行挥手（等待5-10秒）

# 3. 停止记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command: "stop_record"}'

# 4. 保存轨迹
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "save_trajectory", trajectory_id: "my_wave", description: "自定义挥手"}'

# 5. 回放挥手动作
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "play_trajectory", trajectory_id: "my_wave", loop_playback: true}'
```

### 2. 复合动作记录

```bash
# 记录一个复杂的动作序列
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "start_record", description: "复杂动作序列", record_frequency: 30.0}'

# 执行动作序列（例如：挥手 → 点头 → 画圈）

# 暂停记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command: "pause_record"}'

# 恢复记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command: "resume_record"}'

# 停止并保存
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command: "stop_record"}'
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{command: "save_trajectory", trajectory_id: "complex_motion", description: "复杂动作序列"}'
```

## 故障排除

### 常见问题

1. **服务调用失败**
   - 确保teach_manager节点正在运行
   - 检查ROS2环境是否正确加载

2. **轨迹文件无法加载**
   - 检查文件路径是否正确
   - 确认文件格式为有效的JSON

3. **回放没有动作**
   - 检查motor_driver_sim是否运行
   - 确认电机命令话题是否正确连接

4. **记录没有数据**
   - 检查关节状态话题是否正常
   - 确认记录频率设置合理

### 调试命令

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看服务列表
ros2 service list

# 监控记录状态
ros2 topic echo /teach_record_status

# 监控电机命令
ros2 topic echo /motor_commands
```

## 扩展开发

### 添加新的关节配置

在`teach_manager.py`中修改`joint_names`列表：

```python
self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
```

### 自定义轨迹生成

参考`wave_example.py`中的示例，创建新的轨迹生成函数：

```python
def generate_custom_trajectory():
    # 自定义轨迹逻辑
    return trajectory_data
```

### 集成实际的传感器数据

修改`joint_state_callback`函数来处理实际的传感器数据：

```python
def joint_state_callback(self, msg):
    # 处理实际的关节状态消息
    joint_positions = extract_from_actual_sensor(msg)
    # ...
```

## 性能建议

1. **记录频率**：根据动作复杂度选择合适的记录频率（10-50 Hz）
2. **轨迹存储**：长时间记录会产生大量数据，定期清理旧轨迹
3. **回放速度**：避免过快的回放速度导致电机响应不过来
4. **内存使用**：长时间记录时注意内存占用，考虑分段记录

## 许可证

MIT License - 详见项目根目录LICENSE文件