# Teleop2CanTransformer

这个包将远程端的控制指令（JSON 格式）转换为 cannode 可以接收的控制指令（protobuf 格式）。

## 功能

- 订阅 `/controls/teleop` 话题，接收 JSON 格式的控制命令
- 将控制命令转换为 `ControlCommand` protobuf 消息
- 发布到 `/vehicle_command` 话题供 cannode 使用
- 实现范围映射和死区处理

## 输入格式

订阅 `/controls/teleop` 话题（`std_msgs/String`），JSON 格式示例：

```json
{
  "steering": 0.5,
  "throttle": 0.8,
  "brake": 0.0,
  "gear": "D",
  "boom": 0.3,
  "bucket": -0.2,
  "parking_brake": false,
  "emergency_stop": false,
  "power_enable": true
}
```

### 字段说明

- `steering` / `rotation`: 转向控制，范围 [-1, 1]，-1 左转，1 右转
- `throttle`: 油门，范围 [0, 1]
- `brake`: 刹车，范围 [0, 1]
- `gear`: 档位，"N"（空档）、"D"（前进）、"R"（后退）
- `boom`: 大臂控制，范围 [-1, 1]
- `bucket`: 铲斗控制，范围 [-1, 1]
- `parking_brake`: 驻车制动，布尔值
- `emergency_stop`: 紧急停止，布尔值
- `power_enable`: 上高压，布尔值

## 输出格式

发布到 `/vehicle_command` 话题（`sa_msgs::msg::ProtoAdapter`），包含序列化的 `control::ControlCommand` protobuf 消息。

### 映射关系

- `steering: [-1, 1]` → `steering_target: [-100, 100]`（百分比，反向）
- `throttle: [0, 1]` → `throttle: [0, 100]`（百分比）
- `brake: [0, 1]` → `brake: [0, 100]`（百分比）
- `gear: "N"/"D"/"R"` → `gear_location: GEAR_NEUTRAL/GEAR_DRIVE/GEAR_REVERSE`
- `boom: [-1, 1]` → `arm_angle: [arm_angle_min, arm_angle_max]`（度）
- `bucket: [-1, 1]` → `shovel_angle: [shovel_angle_min, shovel_angle_max]`（度，反向）

## 死区处理

所有控制输入都应用死区处理，避免小幅度抖动：

- 死区内的值被映射为 0
- 死区外的值线性缩放到有效范围

## 参数配置

可通过 launch 文件或 ROS2 参数配置：

- `steering_deadzone`: 转向死区（默认 0.05）
- `throttle_deadzone`: 油门死区（默认 0.05）
- `brake_deadzone`: 刹车死区（默认 0.05）
- `boom_deadzone`: 大臂死区（默认 0.05）
- `bucket_deadzone`: 铲斗死区（默认 0.05）
- `arm_angle_min`: 大臂最小角度（度，默认 -60.0）
- `arm_angle_max`: 大臂最大角度（度，默认 60.0）
- `shovel_angle_min`: 铲斗最小角度（度，默认 -60.0）
- `shovel_angle_max`: 铲斗最大角度（度，默认 60.0）
- `max_speed`: 最大速度（m/s，默认 3.0）

## 使用方法

### 编译

```bash
cd /home/cyber007/cannode_ws
colcon build --packages-select Teleop2CanTransformer
source install/setup.bash
```

### 运行

```bash
ros2 launch Teleop2CanTransformer teleop_converter.launch.py
```

### 测试

可以使用以下命令发布测试消息：

```bash
ros2 topic pub /controls/teleop std_msgs/String "data: '{\"steering\": 0.5, \"throttle\": 0.8, \"gear\": \"D\"}'"
```

## 测试程序

提供了一个完整的测试程序 (`scripts/test_converter.py`)，包含所有测试功能：

```bash
python3 src/Teleop2CanTransformer/scripts/test_converter.py
```

### 功能特性

1. **模拟输入信息及其范围**（参考 `keyboard_piston_joint_publisher_2_updated.py`）
   - steering/rotation: [-1.0, 1.0]
   - throttle: [0.0, 1.0]
   - brake: [0.0, 1.0]
   - boom: [-1.0, 1.0]
   - bucket: [-1.0, 1.0]
   - gear: 'N', 'D', 'R'

2. **验证输出信息及范围**（参考 `control_cmd.proto` 和 cannode 实现）
   - steering_target: [-100, 100]%
   - throttle: [0, 100]%
   - brake: [0, 100]%
   - arm_angle: [0, 60]°（cannode限制）
   - shovel_angle: [-60, 60]°
   - gear_location: 1(D), 2(N), 3(R)

3. **测试用例类型**（共 23 个测试用例）
   - 基础功能测试（空档、前进、倒车等）
   - 边界值测试（最小值、最大值）
   - 档位测试（N/D/R）
   - 大臂/铲斗测试（角度范围）
   - 超出范围测试（验证限制功能）
   - 特殊功能测试（紧急停止、驻车制动、发动机开关）
   - 组合测试（多参数组合）

4. **自动验证**
   - 解析 protobuf 消息
   - 验证期望值与实际值
   - 验证输出范围
   - 生成测试报告和统计

## 注意事项

1. 确保 cannode 包已编译，protobuf 文件已生成
2. 确保 `/vehicle_command` 话题的订阅者（cannode）正在运行
3. 角度映射范围可根据实际车辆参数调整
4. 死区大小可根据控制精度需求调整
5. 测试前确保 teleop2can_transformer 节点正在运行：
   ```bash
   ros2 launch Teleop2CanTransformer teleop_converter.launch.py
   ```
