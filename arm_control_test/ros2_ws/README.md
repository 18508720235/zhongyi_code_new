# 真实机械臂示教系统使用说明

## 功能概述

本系统提供真实机械臂的示教录制与回放功能：
- **录制轨迹**：通过手动拖动机械臂，记录关节位置/速度/力矩
- **保存轨迹**：将录制的轨迹保存为 JSON 文件
- **回放轨迹**：加载已保存的轨迹，通过电机命令回放动作

## 系统架构

```
┌─────────────────────┐
│  /joint_states_pub  │  ← 机械臂实时关节状态 (1000Hz)
└──────────┬──────────┘    类型: sensor_msgs/JointState
           │
           ↓ (降采样到50Hz)
┌─────────────────────┐
│ teach_manager_real  │  ← 示教管理器节点
└──────────┬──────────┘
           │
           ├─ 服务: /teach_control_real (录制/停止/保存/加载/回放)
           ├─ 发布: /motor_commands (回放时的电机命令)
           │         类型: sensor_msgs/JointState ← 标准消息，无需source
           └─ 发布: /teach_record_status (录制状态)
```

## 快速开始

### 1. 编译安装

```bash
cd ~/Desktop/arm_control/arm_control2_test/ros2_ws
colcon build
source install/setup.bash
```

### 2. 录制轨迹

**重要**: 必须使用 root 权限运行（因为 `/joint_states_pub` 只能在 root 用户下访问）

**终端 1 - 启动示教管理器**:
```bash
sudo su
cd /home/niic/Desktop/arm_control/arm_control2_test/ros2_ws
source install/setup.bash
ros2 launch wave_control_system teach_real.launch.py
```
**备注：模拟真实电机测试，需要在新开一个 终端 3 -运行 ros2 run wave_control_system mock_joint_publisher（模拟发布关节数据，真机中忽略该步骤）**

**终端 2 - 运行录制程序**:
```bash
sudo su
cd /home/niic/Desktop/arm_control/arm_control2_test/ros2_ws
source install/setup.bash
ros2 run wave_control_system test_teach_real
```
查看 话题： /joint_states_pub 是否有数据发布
后续和其他部分一样

**交互流程**:
```
准备开始示教录制...
按 Enter 键开始录制...           [← 按 Enter]
✓ 录制已开始

正在录制中... 请手动操作机械臂    [← 手动拖动机械臂]
完成示教动作后，按 Enter 键停止录制... [← 按 Enter]

停止录制...
请输入轨迹名称（将自动添加 .json 后缀）: wave_motion  [← 输入名称]
✓ 保存成功: trajectories/wave_motion.json
```

### 3. 回放轨迹

**终端 1 - 启动示教管理器** (如果没运行的话):
```bash
sudo su
cd /home/niic/Desktop/arm_control/arm_control2_test/ros2_ws
source install/setup.bash
ros2 launch wave_control_system teach_real.launch.py
```

**终端 2 - 运行回放程序**:
```bash
sudo su
cd /home/niic/Desktop/arm_control/arm_control2_test/ros2_ws
source install/setup.bash
ros2 run wave_control_system test_playback
```

**交互流程**:
```
可用的轨迹文件:
  1. wave_motion
  2. pick_and_place
  
请输入要回放的轨迹名称（不含.json后缀）: wave_motion
请输入回放速度倍数 (默认1.0): 1.5
是否循环播放? (y/n, 默认n): n

✓ 回放已开始
回放进行中...
```

## 高级使用

### 使用 ROS2 服务直接控制

你也可以通过命令行直接调用服务：

**开始录制**:
```bash
ros2 service call /teach_control_real wave_control_msgs/srv/TeachControl \
  "{command: 'start_record', description: '我的动作', record_frequency: 50.0}"
```

**停止录制**:
```bash
ros2 service call /teach_control_real wave_control_msgs/srv/TeachControl \
  "{command: 'stop_record'}"
```

**保存轨迹**:
```bash
ros2 service call /teach_control_real wave_control_msgs/srv/TeachControl \
  "{command: 'save_trajectory', trajectory_id: 'my_trajectory'}"
```

**回放轨迹**:
```bash
ros2 service call /teach_control_real wave_control_msgs/srv/TeachControl \
  "{command: 'play_trajectory', trajectory_id: 'my_trajectory', play_speed: 1.0, loop_playback: false}"
```

### 查看录制状态

```bash
ros2 topic echo /teach_record_status
```

输出示例：
```
record_id: "record_20251210_143520"
state: "recording"
frame_count: 147
duration: 2.94
frequency: 50.0
description: "Recording from real joints"
```

### 监听电机命令（回放时）

**注意**：现在使用标准 `sensor_msgs/JointState` 消息，可以在任何地方直接查看，无需 source！

```bash
ros2 topic echo /motor_commands
```

输出示例：
```yaml
header:
  stamp:
    sec: 1765268120
    nanosec: 123456789
name: ['joint1_ti5', 'joint2_ti5', 'joint3_ti5', 'joint4_ti5', 'joint5_encos']
position: [0.1, 0.2, 0.3, 0.4, 0.5]
velocity: [30.0, 30.0, 30.0, 30.0, 30.0]
effort: []
```

## 轨迹文件格式

轨迹保存在 `trajectories/` 目录下，格式为 JSON：

```json
{
  "id": "wave_motion",
  "description": "挥手动作",
  "joint_names": ["joint1_ti5", "joint2_ti5", "joint3_ti5", "joint4_ti5", "joint5_encos"],
  "num_joints": 5,
  "record_frequency": 50.0,
  "record_time": "2025-12-10T14:35:20",
  "data_source": "real_joint_states",
  "frames": [
    {
      "timestamp": 1765268120.123,
      "positions": [0.1, 0.2, 0.3, 0.4, 0.5],
      "velocities": [0.01, 0.02, 0.03, 0.04, 0.05],
      "efforts": [0.0, 0.0, 0.0, 0.0, 0.0]
    },
    ...
  ]
}
```

## 技术参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 录制频率 | 50 Hz | 自动从1000Hz降采样 |
| 关节数量 | 5 | 可自动检测 |
| 支持的服务命令 | start_record, stop_record, pause_record, resume_record, save_trajectory, load_trajectory, play_trajectory | - |
| 回放速度范围 | 0.1x - 10.0x | 可调节 |
| 数据源 | `/joint_states_pub` | 真实关节状态 |
| 电机控制 | `/motor_commands` | 回放时发布 |

## 常见问题

### Q1: 为什么必须用 root 权限运行？
**A**: 因为 `/joint_states_pub` 话题由 root 权限的节点发布。ROS2 DDS 默认隔离不同用户的节点，导致普通用户无法订阅该话题。

### Q2: 录制的数据量太大怎么办？
**A**: 系统已经将 1000Hz 的原始数据降采样到 50Hz，大幅减少数据量。如果仍然太大，可以修改 `teach_manager_real.py` 中的 `min_interval = 1.0 / 50.0` 改为更低的频率（如 25Hz: `1.0 / 25.0`）。

### Q3: 回放时机械臂不动？
**A**: 请确认：
1. 电机控制节点正在监听 `/motor_commands` 话题
2. 轨迹文件格式正确
3. 检查终端输出是否有错误信息

### Q4: 如何在虚拟机中测试？
**A**: 使用 mock 节点模拟关节数据：
```bash
# 终端1
ros2 run wave_control_system mock_joint_publisher

# 终端2
ros2 launch wave_control_system teach_real.launch.py

# 终端3
ros2 run wave_control_system test_teach_real
```

## 文件说明

- `teach_manager_real.py` - 示教管理器节点（核心服务）
- `test_teach_real.py` - 交互式录制测试程序
- `test_playback.py` - 交互式回放测试程序
- `mock_joint_publisher.py` - 模拟关节状态发布器（用于虚拟机测试）
- `teach_real.launch.py` - 启动配置文件
- `trajectories/` - 轨迹文件存储目录（自动创建）

## 下一步开发

- [ ] 添加轨迹可视化工具
- [ ] 支持轨迹编辑和拼接
- [ ] 添加安全检测（速度/加速度限制）
- [ ] Web 界面控制
