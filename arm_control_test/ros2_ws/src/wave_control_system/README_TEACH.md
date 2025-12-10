# 机械臂示教-回放系统使用指南

## 📋 概述

本项目为wave_control_system添加了完整的示教功能，支持记录机械臂动作轨迹并精确回放。系统基于ROS2框架，可以记录任意复杂的机械臂动作并重复执行。

## ✨ 主要功能

- ✅ **实时轨迹记录**：支持1-100Hz可调节记录频率
- ✅ **精确轨迹回放**：支持0.1x-10x播放速度，循环播放
- ✅ **暂停/恢复记录**：灵活控制记录过程
- ✅ **轨迹文件管理**：JSON格式存储，支持多个轨迹文件
- ✅ **状态监控**：实时显示记录和回放状态
- ✅ **系统集成**：与现有wave_control系统无缝集成

## 🚀 快速开始

### 1. 环境准备

```bash
# 进入ROS2工作空间
cd ~/Desktop/arm_control2_test/ros2_ws

# 构建项目
colcon build --packages-select wave_control_msgs wave_control_system

# 刷新环境
source install/setup.bash
```

### 2. 启动系统

```bash
# 方法1：一键启动演示（推荐）
ros2 launch wave_control_system teach_demo.launch.py

# 方法2：手动启动各个组件
# 终端1：示教管理器
ros2 run wave_control_system teach_manager

# 终端2：电机驱动模拟器
ros2 run wave_control_system motor_driver_sim


## 📖 完整示教-回放流程

### 示教阶段 - 记录机械臂动作

#### 1. 开始记录轨迹

```bash
# 基本记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "start_record", "description": "我的挥手动作", "record_frequency": 20.0}'

# 完整参数示例
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "start_record",
    "description": "复杂挥手动作训练",
    "record_frequency": 30.0
  }'
```

**参数说明：**
- `command`: `"start_record"` - 开始记录
- `description`: 轨迹描述信息
- `record_frequency`: 记录频率（Hz），建议10-50Hz

#### 2. 暂停/恢复记录（可选）

```bash
# 暂停记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "pause_record"}'

# 恢复记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "resume_record"}'
```

#### 3. 停止记录

```bash
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "stop_record"}'
```

### 保存阶段 - 存储轨迹文件

```bash
# 保存记录的轨迹
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "save_trajectory",
    "trajectory_id": "my_wave_001",
    "description": "第一次挥手练习"
  }'
```

**参数说明：**
- `trajectory_id`: 轨迹唯一标识符
- `description`: 轨迹详细描述

**保存位置：** `trajectories/{trajectory_id}.json`

### 回放阶段 - 执行录制的动作

#### 1. 基本回放

```bash
# 正常速度回放
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "play_trajectory",
    "trajectory_id": "my_wave_001",
    "play_speed": 1.0,
    "loop_playback": false
  }'
```

#### 2. 高级回放选项

```bash
# 快速回放（2倍速）
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "play_trajectory",
    "trajectory_id": "my_wave_001",
    "play_speed": 2.0,
    "loop_playback": false
  }'

# 慢速回放（0.5倍速）
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "play_trajectory",
    "trajectory_id": "my_wave_001",
    "play_speed": 0.5,
    "loop_playback": false
  }'

# 循环回放
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "play_trajectory",
    "trajectory_id": "my_wave_001",
    "play_speed": 1.0,
    "loop_playback": true
  }'
```

#### 3. 回放预置示例轨迹

```bash
# 回放挥手示例
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "play_trajectory",
    "trajectory_id": "wave_example",
    "play_speed": 1.5
  }'

# 回放点头示例
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "play_trajectory",
    "trajectory_id": "nod_example",
    "play_speed": 1.0
  }'

# 回放画圆示例
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "play_trajectory",
    "trajectory_id": "circle_example",
    "play_speed": 0.8
  }'
```

## 🔍 监控和调试

### 1. 监控记录状态

```bash
# 实时查看记录状态
ros2 topic echo /teach_record_status
```

**输出示例：**
```
record_id: "record_20241112_123045"
state: "recording"
frame_count: 125
duration: 6.25
frequency: 20.0
description: "我的挥手动作"
```

### 2. 监控电机命令

```bash
# 查看回放时的电机动作
ros2 topic echo /motor_commands
```

### 3. 检查系统状态

```bash
# 查看节点状态
ros2 node list | grep -E "(teach|motor)"

# 查看服务列表
ros2 service list | grep teach

# 查看话题列表
ros2 topic list | grep -E "(teach|motor)"
```

## 📁 轨迹文件管理

### 1. 查看轨迹文件

```bash
# 列出所有轨迹文件
ls -la trajectories/

# 查看轨迹文件内容
cat trajectories/my_wave_001.json | head -20

# 查看轨迹文件统计
wc -l trajectories/my_wave_001.json
```

### 2. 轨迹文件格式

```json
{
  "id": "my_wave_001",
  "description": "第一次挥手练习",
  "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
  "record_frequency": 20,
  "record_time": "2024-11-12T12:30:45.123456",
  "frames": [
    {
      "timestamp": 1636705845.123,
      "positions": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    },
    {
      "timestamp": 1636705845.173,
      "positions": [0.15, 0.25, 0.35, 0.0, 0.0, 0.0]
    }
  ]
}
```

### 3. 加载已有轨迹

```bash
# 加载指定轨迹到内存（不立即播放）
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{
    "command": "load_trajectory",
    "trajectory_id": "my_wave_001"
  }'
```

## 🎮 实际使用示例

### 示例1：学习挥手动作

```bash
# Step 1: 开始记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "start_record", "description": "学习挥手", "record_frequency": 25.0}'

# Step 2: 手动移动机械臂执行挥手动作（等待8-10秒）

# Step 3: 停止记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command": "stop_record"}'

# Step 4: 保存轨迹
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "save_trajectory", "trajectory_id": "learned_wave", "description": "学会的挥手动作"}'

# Step 5: 验证回放
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "play_trajectory", "trajectory_id": "learned_wave", "play_speed": 1.0}'
```

### 示例2：复杂动作序列训练

```bash
# Step 1: 记录组合动作
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "start_record", "description": "组合动作：挥手+点头", "record_frequency": 30.0}'

# Step 2: 执行动作序列
# - 0-3秒：挥手动作
# - 3-4秒：暂停（暂停记录）
# - 4-6秒：点头动作（恢复记录）

# 暂停记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command": "pause_record"}'

# 恢复记录
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command": "resume_record"}'

# Step 3: 保存组合动作
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "save_trajectory", "trajectory_id": "combo_action", "description": "挥手点头组合"}'

# Step 4: 循环练习
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "play_trajectory", "trajectory_id": "combo_action", "loop_playback": true}'
```

### 示例3：动作优化训练

```bash
# 记录第一版本
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "start_record", "description": "挥手V1.0", "record_frequency": 20.0}'
# ... 执行动作 ...
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command": "stop_record"}'
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "save_trajectory", "trajectory_id": "wave_v1", "description": "挥手动作第一版"}'

# 记录改进版本
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "start_record", "description": "挥手V2.0", "record_frequency": 20.0}'
# ... 执行改进动作 ...
ros2 service call /teach_control wave_control_msgs/srv/TeachControl '{command": "stop_record"}'
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "save_trajectory", "trajectory_id": "wave_v2", "description": "挥手动作改进版"}'

# 对比回放
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "play_trajectory", "trajectory_id": "wave_v1", "play_speed": 1.0}'
# 观察后...
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "play_trajectory", "trajectory_id": "wave_v2", "play_speed": 1.0}'
```

## 🛠️ 系统管理

### 1. 运行自动化测试

```bash
# 运行完整的功能测试套件
ros2 run wave_control_system test_teach_function

# 运行独立测试（不依赖ROS2）
python3 standalone_test.py
```

### 2. 生成示例轨迹

```bash
# 生成各种示例轨迹用于测试
ros2 run wave_control_system wave_example
```

### 3. 重置系统

```bash
# 停止所有相关进程
pkill -f "teach_manager"
pkill -f "motor_driver_sim"

# 清理轨迹文件（可选）
rm -rf trajectories/*.json
```

## ⚠️ 故障排除

### 常见问题及解决方案

#### 1. 服务调用失败
```bash
# 检查teach_manager是否运行
ros2 node list | grep teach_manager

# 检查服务是否可用
ros2 service list | grep teach_control

# 解决方案：重新启动teach_manager
ros2 run wave_control_system teach_manager
```

#### 2. 轨迹文件未找到
```bash
# 检查轨迹文件是否存在
ls -la trajectories/

# 检查文件名拼写
ros2 service call /teach_control wave_control_msgs/srv/TeachControl \
  '{"command": "play_trajectory", "trajectory_id": "wave_example"}'
```

#### 3. 回放时没有电机动作
```bash
# 检查motor_driver_sim是否运行
ros2 node list | grep motor

# 检查电机命令话题
ros2 topic echo /motor_commands

# 解决方案：重新启动motor_driver_sim
ros2 run wave_control_system motor_driver_sim
```

#### 4. 记录没有数据
```bash
# 检查记录状态
ros2 topic echo /teach_record_status

# 确保记录状态为"recording"
# 检查记录频率设置是否合理（10-50Hz）
```

### 性能优化建议

1. **记录频率选择**：
   - 简单动作：10-20Hz
   - 复杂动作：30-50Hz
   - 高精度要求：50-100Hz

2. **回放速度**：
   - 学习阶段：0.5x-1.0x
   - 正常演示：1.0x
   - 快速展示：1.5x-2.0x

3. **内存管理**：
   - 长时间记录会产生大量数据
   - 建议单个动作记录不超过60秒
   - 定期清理不需要的轨迹文件

## 📚 API参考

### TeachControl服务命令

| 命令 | 功能 | 必需参数 | 可选参数 |
|------|------|----------|----------|
| `start_record` | 开始记录 | - | `description`, `record_frequency` |
| `stop_record` | 停止记录 | - | - |
| `pause_record` | 暂停记录 | - | - |
| `resume_record` | 恢复记录 | - | - |
| `save_trajectory` | 保存轨迹 | `trajectory_id` | `description` |
| `load_trajectory` | 加载轨迹 | `trajectory_id` | - |
| `play_trajectory` | 回放轨迹 | `trajectory_id` | `play_speed`, `loop_playback` |

### 话题消息

| 话题名 | 消息类型 | 功能 |
|--------|----------|------|
| `/teach_control` | `TeachControl` (service) | 示教控制服务 |
| `/teach_record_status` | `TeachRecord` (msg) | 记录状态发布 |
| `/motor_commands` | `MotorCommand` (msg) | 电机命令发布 |
| `/joint_trajectory` | `JointTrajectory` (msg) | 轨迹数据发布 |

## 🎯 总结

示教-回放系统现在已经完全集成到你的wave_control系统中，可以：

1. **简单易用**：通过几个简单的service命令即可完成复杂的示教流程
2. **功能完整**：支持记录、保存、加载、回放的完整工作流程
3. **高度可定制**：支持多种记录频率、播放速度和循环模式
4. **稳定可靠**：经过充分测试，具有良好的错误处理机制
5. **易于扩展**：模块化设计，便于添加新功能

现在你可以开始使用这个系统来训练和重复执行各种机械臂动作了！🎉