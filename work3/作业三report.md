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

## 7. 国际化与多语言支持

为了提高代码的通用性和可读性，对可视化部分进行了国际化处理，将显示在图表中的中文标题和标签替换为英文：

### 7.1 修改内容

1. **将图表标题从中文改为英文**
   - 将"激光地图"改为"Laser Map"
   - 将"应急地图 (由于错误自动保存)"改为"Emergency Map (Auto-saved due to error)"
   - 将"占用栅格图"改为"Occupancy Grid Map"
   - 将"机器人运动轨迹"改为"Robot Trajectory"

2. **更新图例标签**
   - 将"机器人轨迹"改为"Robot Trajectory"
   - 将"起点"改为"Start"
   - 将"终点"改为"End"
   - 将"障碍物"改为"Obstacles"

3. **更新坐标轴标签**
   - 将"X 坐标 (米)"和"Y 坐标 (米)"改为"X Coordinate (m)"和"Y Coordinate (m)"
   - 将"占用概率"改为"Occupancy Probability"

### 7.2 字体处理

移除了专门设置中文字体的代码，如`setup_chinese_font()`函数，改为使用通用的英文字体设置：

```python
# 设置默认字体为通用英文字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号
```

这些修改确保了生成的可视化图像能够正确显示英文标题和标签，提高了代码的国际化水平，同时保留了日志输出和代码注释中的中文信息，以便于中文环境下的开发和调试。

## 8. 转弯优化与地图质量提升

针对小车在转弯场景下地图生成质量不佳的问题，我们进行了针对性的优化。主要问题表现为：在小车转弯时，生成的障碍物点呈现歪斜状态，地图连接不流畅，墙壁轮廓不连续。

### 8.1 转弯场景的特殊处理

1. **转弯检测机制**
   - 实现了基于角速度的转弯检测算法，通过计算机器人方向角的变化率来判断是否处于转弯状态
   - 使用滑动窗口平均角速度，防止瞬时角度变化导致误判
   - 在转弯状态下应用特殊的地图生成策略

```python
def detect_turning(self, current_theta):
    """检测机器人是否正在转弯"""
    if hasattr(self, 'last_theta'):
        # 计算角度差，处理角度环绕
        angle_diff = self.normalize_angle(current_theta - self.last_theta)
        
        # 记录角度变化历史
        self.turning_history.append(abs(angle_diff))
        if len(self.turning_history) > self.turning_history_max_len:
            self.turning_history.pop(0)
        
        # 如果最近几次角度变化的平均值超过阈值，认为正在转弯
        avg_angle_change = sum(self.turning_history) / len(self.turning_history)
        self.is_turning = avg_angle_change > self.turn_detection_threshold
```

2. **方向感知的点云过滤**
   - 根据机器人的运动方向和障碍物的相对位置，对激光点进行加权处理
   - 转弯时提高侧面障碍物的权重，直行时提高前方障碍物的权重
   - 通过这种方式使地图生成更符合实际环境布局

```python
def filter_points_with_direction(self, points, robot_pose):
    # ...
    # 转弯时使用不同的过滤策略
    if self.is_turning:
        # 转弯时更关注侧面的障碍物
        weight = 1.0
        if angle_diff < math.pi/4:  # 前方45度范围
            weight = 1.2  # 增强前方障碍物权重
        elif math.pi/4 <= angle_diff < 3*math.pi/4:  # 侧面90度范围
            weight = 1.5  # 大幅增强侧面障碍物权重，这是转弯时最关键的部分
        filtered_points.append((x, y, weight))
```

### 8.2 阈值动态调整

为了解决转弯处障碍物不连续的问题，实现了动态阈值调整机制：

1. **转弯时降低障碍物检测阈值**
   - 在转弯状态下，将障碍物判定阈值降低30%，使障碍物更容易被标记
   - 通过参数`turn_threshold_factor`控制降低比例

```python
# 转弯时使用更低的阈值，更容易标记障碍物
if self.is_turning:
    local_threshold *= self.turn_threshold_factor
```

2. **距离与方向双重自适应**
   - 结合点距离和方向因素，对障碍物计数阈值进行综合调整
   - 近距离且位于转弯侧面的点使用更低阈值，远距离的点适当提高阈值

### 8.3 地图平滑处理

1. **高斯滤波平滑**
   - 使用scipy库的高斯滤波对地图进行平滑处理，减少不连续性
   - 根据转弯状态动态调整滤波窗口大小

```python
def smooth_map(self):
    # ...
    # 使用scipy的高斯滤波器对障碍物部分进行平滑
    obstacle_map = (self.grid_map > 0).astype(np.float32)
    smoothed = scipy.ndimage.gaussian_filter(obstacle_map, sigma=0.8, truncate=1.5)
```

2. **智能空隙填充**
   - 检测障碍物之间的空隙，在满足特定条件时进行填充
   - 消除转弯处墙壁上的不连续缝隙

```python
# 如果原来不是障碍物，但周围有很多障碍物支持，则标记为障碍物
elif self.grid_map[x, y] == 0 and smoothed[x, y] > 0.7:
    # 转弯时更容易填充空隙
    if self.is_turning or self.count_neighbor_obstacles(x, y) >= 5:
        self.grid_map[x, y] = 1
```

### 8.4 效果与改进

经过上述优化，地图生成质量有显著提升：

1. **墙壁轮廓更加连续**：转弯处的障碍物连接更加平滑，减少了锯齿状边缘
2. **减少歪斜障碍物条**：通过方向感知过滤，有效减少了歪斜障碍物的产生
3. **更准确的环境表示**：优化后的地图更好地保留了环境的实际几何特征

这些改进使得地图在转弯区域也能保持高质量，为路径规划和导航提供更可靠的环境表示。

## 9. ROS2相关技术应用

本作业应用了以下ROS2技术：

1. **ROS2包结构**: 使用标准的ROS2 Python包结构
2. **消息类型**: 使用标准ROS2消息类型进行通信
3. **节点通信**: 实现了发布者/订阅者模式
4. **参数系统**: 使用ROS2参数系统配置节点行为
5. **rosbag2**: 使用rosbag2进行数据记录和回放
6. **坐标变换**: 使用tf2实现坐标系转换

## 10. 与ROS1版本的对比

与ROS1相比，ROS2具有以下优势：

1. **实时性**: 基于DDS中间件，提供更好的实时性能
2. **安全性**: 提供内置的安全功能
3. **跨平台**: 支持更多操作系统
4. **模块化**: 更加模块化的设计
5. **QoS**: 提供服务质量设置
6. **多节点支持**: 更好的多节点、多进程支持

