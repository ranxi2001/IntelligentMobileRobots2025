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

栅格地图与小车行驶轨迹：
![](.\map_checkpoint_29.png)

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



> 本人本次作业的项目代码仓库：https://github.com/ranxi2001/IntelligentMobileRobots2025

> 因Python绘制栅格地图的代码过长，仅放置代码链接地址：https://github.com/ranxi2001/IntelligentMobileRobots2025/blob/main/work3/src/lms_mapper/lms_mapper/lms_mapper_node.py

## 附录：作业三完整代码
```bash
#!/bin/bash

# 作业3：使用ROS2重新实现作业2中的激光雷达数据处理和占用栅格地图生成
# 脚本说明：
# 1. 首先执行setup.sh创建ROS2工作空间结构
# 2. 构建ROS2包
# 3. 将原始LMS和NAV数据转换为rosbag格式
# 4. 运行栅格地图生成节点，接收rosbag数据并生成地图

# 设置日志文件
LOG_FILE="$(date +%Y%m%d_%H%M%S)_run.log"
echo "所有输出将记录到日志文件: $LOG_FILE"
# 使用tee命令同时将输出发送到终端和日志文件
exec > >(tee -a "$LOG_FILE") 2>&1
echo "======== 运行日志开始 $(date) ========"

set -e  # 遇到错误立即退出

# 设置ROS2环境
if ! source /opt/ros/humble/setup.bash; then
    echo "❌ 错误: ROS2环境设置失败"
    exit 1
fi

# 设置变量 - 使用正确的路径
WORK_DIR=$(pwd)
SRC_DIR="${WORK_DIR}/src"
LMS_FILE="../data/URG_X_20130903_195003.lms"
NAV_FILE="../data/ld.nav"
BAG_PATH="lms_data.bag"
MAP_PATH="occupancy_grid.png"

# 功能：清理进程
function cleanup {
    # 终止可能还在运行的进程
    echo "正在清理进程..."
    
    # 检查并终止mapper进程
    if [ ! -z ${MAPPER_PID+x} ] && ps -p $MAPPER_PID > /dev/null; then
        echo "终止LMS Mapper进程 (PID: $MAPPER_PID)..."
        kill -SIGINT $MAPPER_PID 2>/dev/null || true
        sleep 2
        if ps -p $MAPPER_PID > /dev/null; then
            echo "⚠️ LMS Mapper节点未能优雅退出，强制终止..."
            kill -9 $MAPPER_PID 2>/dev/null || true
        fi
    fi
    
    # 检查并终止rosbag播放进程
    if [ ! -z ${PLAY_PID+x} ] && ps -p $PLAY_PID > /dev/null; then
        echo "终止rosbag播放进程 (PID: $PLAY_PID)..."
        kill -SIGINT $PLAY_PID 2>/dev/null || true
        sleep 2
        if ps -p $PLAY_PID > /dev/null; then
            kill -9 $PLAY_PID 2>/dev/null || true
        fi
    fi
    
    # 检查并终止时钟进程
    if [ ! -z ${CLOCK_PID+x} ] && ps -p $CLOCK_PID > /dev/null; then
        echo "终止时钟进程 (PID: $CLOCK_PID)..."
        kill -9 $CLOCK_PID 2>/dev/null || true
    fi
}

# 注册退出处理函数
trap cleanup EXIT

echo "======== 开始执行作业3 ========"

# 步骤1：创建ROS2工作空间结构
echo "正在创建ROS2工作空间..."
if ! bash ./setup.sh; then
    echo "❌ 错误: 工作空间创建失败"
    exit 1
fi

# 步骤2：构建ROS2包
echo "正在构建ROS2包..."
cd ${WORK_DIR}
rm -rf build/ install/ log/  # 清理之前的构建

echo "执行: colcon build --symlink-install"
if ! colcon build --symlink-install; then
    echo "❌ 错误: ROS2包构建失败！请检查编译错误。"
    exit 1
fi

# 设置环境变量
if ! source install/setup.bash; then
    echo "❌ 错误: 工作空间环境设置失败"
    exit 1
fi

# 检查可执行文件是否存在，如果不存在使用直接Python调用
echo "检查可执行文件..."
if [ ! -f "install/lms_data/lib/lms_data/lms_to_rosbag" ]; then
    echo "警告: 没有找到lms_to_rosbag可执行文件，将直接使用Python调用"
    LMS_TO_ROSBAG_CMD="python3 ${SRC_DIR}/lms_data/lms_data/lms_to_rosbag.py"
else
    LMS_TO_ROSBAG_CMD="ros2 run lms_data lms_to_rosbag"
fi

if [ ! -f "install/lms_mapper/lib/lms_mapper/lms_mapper_node" ]; then
    echo "警告: 没有找到lms_mapper_node可执行文件，将直接使用Python调用"
    LMS_MAPPER_CMD="python3 ${SRC_DIR}/lms_mapper/lms_mapper/lms_mapper_node.py --ros-args"
else
    LMS_MAPPER_CMD="ros2 run lms_mapper lms_mapper_node --ros-args"
fi

# 检查Python文件是否存在
if [ ! -f "${SRC_DIR}/lms_data/lms_data/lms_to_rosbag.py" ]; then
    echo "❌ 错误: 找不到 ${SRC_DIR}/lms_data/lms_data/lms_to_rosbag.py 文件"
    exit 1
fi

if [ ! -f "${SRC_DIR}/lms_mapper/lms_mapper/lms_mapper_node.py" ]; then
    echo "❌ 错误: 找不到 ${SRC_DIR}/lms_mapper/lms_mapper/lms_mapper_node.py 文件"
    exit 1
fi

# 确保Python文件有执行权限
chmod +x "${SRC_DIR}/lms_data/lms_data/lms_to_rosbag.py"
chmod +x "${SRC_DIR}/lms_mapper/lms_mapper/lms_mapper_node.py"

# 添加一个总体超时控制函数
function run_with_timeout() {
    local cmd="$1"
    local timeout=$2
    local message="$3"
    
    echo "⏱️ 执行命令 (最长${timeout}秒): $cmd"
    
    # 启动后台进程
    eval "$cmd" &
    local pid=$!
    
    # 等待进程完成或超时
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if ! ps -p $pid > /dev/null; then
            echo "✅ 命令已完成"
            wait $pid
            return $?
        fi
        sleep 1
        ((elapsed++))
        
        # 每10秒报告一次进度
        if [ $((elapsed % 10)) -eq 0 ]; then
            echo "⏳ $message - 已等待 ${elapsed}/${timeout} 秒..."
        fi
    done
    
    # 如果超时，终止进程
    echo "⚠️ 超时 (${timeout}秒) - 终止进程 $pid"
    kill -SIGTERM $pid 2>/dev/null || true
    sleep 2
    if ps -p $pid > /dev/null; then
        echo "❌ 进程未响应，强制终止..."
        kill -9 $pid 2>/dev/null || true
    fi
    return 1
}

# 步骤3：将原始数据转换为rosbag格式
echo "正在转换原始数据为rosbag格式..."
echo "执行: ${LMS_TO_ROSBAG_CMD} ${LMS_FILE} ${NAV_FILE} ${BAG_PATH}"

# 确保旧的bag文件被完全删除，防止写入冲突
if [ -d "${BAG_PATH}" ]; then
    echo "删除旧的rosbag目录..."
    rm -rf "${BAG_PATH}"
elif [ -f "${BAG_PATH}" ]; then
    echo "删除旧的rosbag文件..."
    rm -f "${BAG_PATH}"
fi

# 使用超时函数运行数据转换
if ! run_with_timeout "${LMS_TO_ROSBAG_CMD} ${LMS_FILE} ${NAV_FILE} ${BAG_PATH}" 300 "正在转换数据"; then
    echo "❌ 错误: 数据转换执行超时或失败"
    # 尝试检查进程状态
    echo "检查Python进程状态..."
    ps aux | grep python | grep -v grep
    echo "尝试查找部分生成的bag文件..."
    find ${WORK_DIR} -name "*.bag*" -ls
    exit 1
fi

# 检查rosbag是否生成
if [ ! -f "${BAG_PATH}" ] && [ ! -d "${BAG_PATH}" ]; then
    echo "❌ 错误: 未能创建rosbag文件 ${BAG_PATH}"
    exit 1
fi

# 检查rosbag内容
echo "检查rosbag内容..."
if ! run_with_timeout "ros2 bag info ${BAG_PATH}" 30 "读取bag信息"; then
    echo "❌ 错误: 读取rosbag信息超时或失败"
    echo "尝试修复rosbag文件..."
    # 创建一个最小的有效rosbag用于测试
    TMP_BAG_PATH="tmp_test.bag"
    if ros2 bag record -o ${TMP_BAG_PATH} /parameter_events -d 3; then
        echo "测试rosbag创建成功，继续尝试使用原始数据"
    else
        echo "❌ 错误: 无法创建测试rosbag文件，可能是ROS2环境问题"
        exit 1
    fi
    exit 1
fi

# 检查是否包含/clock话题
if ! ros2 bag info ${BAG_PATH} | grep -q "/clock"; then
    echo "⚠️ 警告: rosbag中未找到/clock话题，将使用模拟时钟"
    # 创建模拟时钟
    ros2 topic pub /clock rosgraph_msgs/msg/Clock "{clock: {sec: 0, nanosec: 0}}" --rate 10 --once
    NEED_CLOCK=true
else
    NEED_CLOCK=false
fi

# 步骤4：在另一个终端中运行激光雷达数据处理节点
echo "正在启动LMS Mapper节点..."
echo "执行: ${LMS_MAPPER_CMD} -p map_save_path:=${MAP_PATH}"

# 设置使用模拟时钟
echo "设置ROS2使用模拟时钟..."
ros2 param set /use_sim_time true || echo "无法设置全局参数，将在节点启动时设置"

# 启动节点并保存PID
${LMS_MAPPER_CMD} -p map_save_path:=${MAP_PATH} -p map_resolution:=0.1 -p map_threshold:=1 -p filter_points:=true -p filter_threshold:=1.0 -p use_sim_time:=true &
MAPPER_PID=$!

# 等待节点启动
echo "等待节点启动..."
sleep 15  # 增加等待时间确保节点完全启动

# 检查节点是否正在运行
if ! ps -p $MAPPER_PID > /dev/null; then
    echo "❌ 错误: LMS Mapper节点启动失败，进程不存在"
    exit 1
fi

echo "节点已启动，PID: ${MAPPER_PID}"

# 检查话题列表
echo "检查可用的话题..."
ros2 topic list

# 等待话题准备就绪，设置超时
echo "等待话题准备就绪..."
TIMEOUT=30
ELAPSED=0
while ! ros2 topic list | grep -q "/map"; do
    echo "等待 /map 话题...(${ELAPSED}/${TIMEOUT}秒)"
    sleep 1
    ((ELAPSED++))
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "❌ 超时: /map 话题未能出现，检查节点是否正常运行"
        exit 1
    fi
done

echo "✅ /map 话题已准备就绪"

# 步骤5：播放rosbag数据
echo "正在播放rosbag数据..."
if [ -f "${BAG_PATH}" ] || [ -d "${BAG_PATH}" ]; then
    # 确保时钟同步
    echo "等待系统准备就绪，确保时钟同步..."
    sleep 5
    
    # 使用较慢的播放速度，增加消息队列大小，并使用稳定的时钟选项
    echo "开始播放rosbag..."
    # 使用简化的命令，确保能正常播放
    ros2 bag play ${BAG_PATH} --rate 0.1 --read-ahead-queue-size 2000 --clock &
    PLAY_PID=$!
    
    # 检查播放器是否启动成功
    sleep 3
    if ! ps -p $PLAY_PID > /dev/null; then
        echo "❌ 错误: rosbag播放器启动失败"
        echo "尝试使用更基本的命令播放..."
        ros2 bag play ${BAG_PATH} --clock &
        PLAY_PID=$!
        sleep 3
        if ! ps -p $PLAY_PID > /dev/null; then
            echo "❌ 错误: 无法以任何方式播放rosbag"
            exit 1
        fi
    fi
    
    # 使用超时等待rosbag播放完成
    echo "等待rosbag播放完成..."
    # 设置更合理的超时时间，防止无限等待
    PLAY_TIMEOUT=3600
    PLAY_ELAPSED=0
    while ps -p $PLAY_PID > /dev/null; do
        sleep 5
        ((PLAY_ELAPSED+=5))
        
        # 每15秒报告一次进度
        if [ $((PLAY_ELAPSED % 15)) -eq 0 ]; then
            echo "⏳ 正在播放rosbag - 已等待 ${PLAY_ELAPSED}/${PLAY_TIMEOUT} 秒..."
            # 打印当前活跃话题来确认是否仍在正常工作
            echo "当前活跃话题:"
            ros2 topic list | sort
        fi
        
        # 如果超过超时时间，终止播放
        if [ $PLAY_ELAPSED -ge $PLAY_TIMEOUT ]; then
            echo "⚠️ 播放超时 (${PLAY_TIMEOUT}秒) - 终止播放"
            kill -SIGINT $PLAY_PID 2>/dev/null || true
            sleep 5
            if ps -p $PLAY_PID > /dev/null; then
                kill -9 $PLAY_PID 2>/dev/null || true
            fi
            break
        fi
    done
    
    echo "rosbag播放已完成或终止"
else
    echo "❌ 错误: 找不到rosbag文件 ${BAG_PATH}"
    kill $MAPPER_PID 2>/dev/null || true
    exit 1
fi

# 等待处理完成（给足够时间保存地图）
echo "等待处理完成..."
# 增加等待时间，确保有足够时间处理所有数据和保存最终地图
sleep 240  

# 检查地图文件是否已经生成
if [ -f "${MAP_PATH}" ]; then
    echo "✅ 检测到地图已经生成: ${MAP_PATH}"
    MAP_GENERATED=true
else
    echo "⚠️ 地图尚未生成，给节点更多时间完成处理..."
    MAP_GENERATED=false
    # 再多等待60秒
    sleep 60
fi

# 检查节点是否仍在运行
if ps -p $MAPPER_PID > /dev/null; then
    # 先检查地图是否已经生成
    MAP_WAIT_ATTEMPT=0
    MAX_MAP_WAIT_ATTEMPTS=3
    
    while [ "$MAP_GENERATED" = false ] && [ $MAP_WAIT_ATTEMPT -lt $MAX_MAP_WAIT_ATTEMPTS ]; do
        if [ -f "${MAP_PATH}" ]; then
            echo "✅ 检测到地图已经生成: ${MAP_PATH}"
            MAP_GENERATED=true
            break
        else
            MAP_WAIT_ATTEMPT=$((MAP_WAIT_ATTEMPT+1))
            echo "⚠️ 等待地图生成，尝试 $MAP_WAIT_ATTEMPT/$MAX_MAP_WAIT_ATTEMPTS..."
            # 每次再多等待30秒
            sleep 30
        fi
    done
    
    echo "正在关闭节点..."
    
    # 先尝试优雅地关闭节点
    echo "发送SIGTERM信号，请求节点优雅关闭..."
    kill -SIGTERM $MAPPER_PID || true
    
    # 等待节点关闭，最多60秒（增加等待时间）
    for i in {1..600}; do
        if ! ps -p $MAPPER_PID > /dev/null; then
            echo "✅ 节点已正常关闭"
            break
        fi
        echo "等待节点关闭... ($i/600)"
        sleep 1
        
        # 每10秒检查一次地图是否已生成
        if [ $((i % 10)) -eq 0 ] && [ ! -f "${MAP_PATH}" ]; then
            echo "⚠️ 地图仍未生成，继续等待节点处理..."
        fi
    done
    
    # 如果节点仍未关闭，尝试SIGINT信号
    if ps -p $MAPPER_PID > /dev/null; then
        echo "节点未响应SIGTERM，发送SIGINT信号..."
        kill -SIGINT $MAPPER_PID || true
        
        # 再等待30秒
        for i in {1..30}; do
            if ! ps -p $MAPPER_PID > /dev/null; then
                echo "✅ 节点响应SIGINT信号并已关闭"
                break
            fi
            echo "等待节点响应SIGINT... ($i/30)"
            sleep 1
        done
    fi
    
    # 如果节点仍在运行，最后再检查地图是否已生成
    if ps -p $MAPPER_PID > /dev/null; then
        # 最后一次检查地图文件
        if [ ! -f "${MAP_PATH}" ]; then
            echo "⚠️ 地图仍未生成，再等待30秒..."
            sleep 30
        fi
        
        # 如果仍需要强制终止
        if ps -p $MAPPER_PID > /dev/null; then
            echo "⚠️ 警告: 节点无法正常关闭，强制终止..."
            kill -9 $MAPPER_PID || true
        fi
    fi
else
    echo "节点已经停止运行"
fi

# 再次检查地图文件是否生成
if [ -f "${MAP_PATH}" ]; then
    echo "✅ 地图生成成功！保存在: ${MAP_PATH}"
    # 显示文件大小和时间戳
    ls -lh "${MAP_PATH}"
else
    echo "⚠️ 注意: 未找到地图文件，可能在其他位置生成或出现错误"
    # 尝试查找可能的地图文件
    echo "搜索可能的地图文件..."
    find ${WORK_DIR} -name "*.png" -type f -mtime -1 -ls
fi

echo "======== 作业3执行完成 ========"
echo "生成的栅格地图应保存到: ${MAP_PATH}"
echo "如需查看地图，请使用图片查看器打开 ${MAP_PATH}" 
echo "======== 运行日志结束 $(date) ========" 

```

