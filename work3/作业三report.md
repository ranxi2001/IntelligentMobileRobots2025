# 作业3：ROS2实现激光雷达地图生成

## 1. 项目介绍与目标

本项目基于ROS2框架实现了激光雷达数据处理和栅格地图生成系统。通过接收激光雷达数据和里程计数据，系统能够实时构建环境的占用栅格地图，并将结果保存为图像文件。

**主要目标：**
- 将原始LMS激光雷达数据和NAV轨迹数据转换为ROS2 bag格式
- 开发ROS2节点，处理激光数据并生成高质量的占用栅格地图
- 实现参数化设计，便于调整和优化地图生成效果

## 2. 系统设计与架构

系统由两个主要模块组成：

### 2.1 数据转换模块 (lms_data)
- 负责将原始数据转换为ROS2标准消息格式
- 激光数据 → `sensor_msgs/LaserScan`
- 轨迹数据 → `nav_msgs/Odometry` 和 `tf2_msgs/TFMessage`
- 生成标准ROS2 bag文件

### 2.2 地图生成模块 (lms_mapper)
- 订阅 `/scan` 和 `/odom` 话题
- 将激光数据转换到世界坐标系
- 使用逆传感器模型更新栅格地图
- 发布 `/map` 话题并保存结果

## 3. 实现过程与关键技术

### 3.1 坐标转换
实现了从激光雷达坐标系到世界坐标系的转换，关键代码如下：
```python
# 计算激光点在世界坐标系中的坐标
cos_theta = math.cos(robot_theta)
sin_theta = math.sin(robot_theta)

world_x = robot_x + laser_x * cos_theta - laser_y * sin_theta
world_y = robot_y + laser_x * sin_theta + laser_y * cos_theta
```

### 3.2 栅格地图更新
采用批量处理方式提高效率，并使用自适应阈值处理不同区域：
```python
# 根据距离计算局部阈值
if distance > 5.0:  # 5米以外的点
    # 距离因子影响较小，使远处障碍物更容易被检测到
    local_threshold += distance * self.distance_threshold_factor * 0.5
else:
    # 近处障碍物使用更低的阈值
    local_threshold = max(1, self.base_threshold - 1)
```

### 3.3 运动过滤
实现了运动过滤功能，只在机器人移动时记录障碍物，避免静止时的噪声：
```python
# 立即跳过起始静止阶段和长时间静止的数据
if (self.in_static_start_phase and self.processed_scans > 10) or static_duration > self.static_ignore_time:
    if self.processed_scans % 50 == 0:  # 降低日志频率
        self.get_logger().info(f'机器人静止中 ({static_duration:.1f}秒)，跳过处理激光数据')
    return
```

## 4. 参数调优过程

为了获得高质量的地图，我们进行了多轮参数调优：

### 4.1 初始参数设置
- `map_threshold`: 6
- `min_motion_distance`: 0.4
- `static_ignore_time`: 0.5
- `base_threshold`: 4
- `distance_threshold_factor`: 0.9

### 4.2 调试问题
- 发现初始位置累积了过多的障碍点
- 转弯处障碍物不够清晰
- 遇到障碍物清除功能的错误

### 4.3 最终优化参数
- `map_threshold`: 3（降低阈值，使转弯处障碍物更清晰）
- `min_motion_distance`: 0.5（增加最小运动距离，减少静止时记录的点）
- `static_ignore_time`: 0.2（更快识别为静止状态）
- `base_threshold`: 2（降低基础阈值，使转弯处显示更多障碍点）
- `distance_threshold_factor`: 0.5（降低距离因子，远处障碍物也能更好地被检测）

## 5. 结果分析

最终地图效果显著改善：
- 转弯处轮廓更加清晰可见
- 起始位置的重复障碍点大幅减少
- 整体地图质量更高，更准确地表示了环境

## 6. 结论与改进方向

成功实现了基于ROS2的激光雷达地图生成系统，通过多轮参数调优获得了高质量的栅格地图。

**未来改进方向：**
- 实现动态参数调整，便于实时优化
- 添加地图滤波算法，进一步提高质量
- 优化计算效率，减少内存占用
- 添加更多可视化工具，辅助调试

## 7. ROS2相关技术应用

本作业应用了以下ROS2技术：

1. **ROS2包结构**: 使用标准的ROS2 Python包结构
2. **消息类型**: 使用标准ROS2消息类型进行通信
3. **节点通信**: 实现了发布者/订阅者模式
4. **参数系统**: 使用ROS2参数系统配置节点行为
5. **rosbag2**: 使用rosbag2进行数据记录和回放
6. **坐标变换**: 使用tf2实现坐标系转换

## 8. 与ROS1版本的对比

与ROS1相比，ROS2具有以下优势：

1. **实时性**: 基于DDS中间件，提供更好的实时性能
2. **安全性**: 提供内置的安全功能
3. **跨平台**: 支持更多操作系统
4. **模块化**: 更加模块化的设计
5. **QoS**: 提供服务质量设置
6. **多节点支持**: 更好的多节点、多进程支持

