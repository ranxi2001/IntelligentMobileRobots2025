# 作业3：ROS2实现

## 学习目标
- 掌握ROS2最新版本的基本概念和使用方法
- 学习ROS2节点通信、话题订阅与发布
- 了解rosbag数据记录和回放机制

## 任务说明

本作业基于作业2进行ROS2框架的重新实现，主要包括以下内容：

1. 将原始的LMS激光雷达数据和NAV轨迹数据转换为ROS2 bag格式
2. 开发ROS2节点，接收rosbag数据并实现占用栅格地图生成功能

## 目录结构

```
work3/
├── setup.sh                   # 工作空间设置脚本
├── run.sh                     # 运行脚本
├── src/                       # 源代码目录
│   ├── lms_data/              # 数据转换包
│   │   ├── lms_data/
│   │   │   └── lms_to_rosbag.py  # 数据转换节点
│   │   ├── package.xml
│   │   └── setup.py
│   └── lms_mapper/            # 地图生成包
│       ├── lms_mapper/
│       │   └── lms_mapper_node.py  # 地图生成节点
│       ├── package.xml
│       └── setup.py
└── README.md                  # 项目说明文件
```

## 功能模块说明

### 1. 数据转换模块 (lms_data)

将原始的LMS激光雷达数据和NAV轨迹数据转换为ROS2标准消息格式，并保存为rosbag文件：

- 激光数据转换为 `sensor_msgs/LaserScan` 消息
- 轨迹数据转换为 `nav_msgs/Odometry` 消息和 `tf2_msgs/TFMessage` 消息
- 创建了 `/scan`、`/odom` 和 `/tf` 三个标准ROS2话题

### 2. 栅格地图生成模块 (lms_mapper)

接收rosbag数据，进行激光数据处理和占用栅格地图生成：

- 订阅 `/scan` 和 `/odom` 话题
- 将激光数据从激光坐标系转换到全局坐标系
- 使用逆传感器模型更新占用栅格地图
- 定期发布 `/map` 话题 (nav_msgs/OccupancyGrid)
- 将栅格地图保存为图像文件

## 运行方法

在工作目录中执行以下命令：

```bash
# 1. 确保脚本可执行
chmod +x setup.sh run.sh

# 2. 执行运行脚本
./run.sh
```

脚本将自动执行以下步骤：
1. 创建ROS2工作空间结构
2. 构建ROS2包
3. 将原始数据转换为rosbag格式
4. 启动地图生成节点
5. 播放rosbag数据
6. 生成并保存占用栅格地图

## 输出结果

成功执行后，将生成以下文件：
- `lms_data.bag`: 转换后的ROS2 bag数据文件
- `occupancy_grid.png`: 生成的占用栅格地图图像

## ROS2相关技术应用

本作业应用了以下ROS2技术：

1. **ROS2包结构**: 使用标准的ROS2 Python包结构
2. **消息类型**: 使用标准ROS2消息类型进行通信
3. **节点通信**: 实现了发布者/订阅者模式
4. **参数系统**: 使用ROS2参数系统配置节点行为
5. **rosbag2**: 使用rosbag2进行数据记录和回放
6. **坐标变换**: 使用tf2实现坐标系转换

## 与ROS1版本的对比

与ROS1相比，ROS2具有以下优势：

1. **实时性**: 基于DDS中间件，提供更好的实时性能
2. **安全性**: 提供内置的安全功能
3. **跨平台**: 支持更多操作系统
4. **模块化**: 更加模块化的设计
5. **QoS**: 提供服务质量设置
6. **多节点支持**: 更好的多节点、多进程支持

## 基本要求
- 选择之前完成的作业1或作业2中的一个任务（**选择作业2将获得加分**）
- 使用ROS2框架重新实现该任务
- 完成以下两个关键步骤：
  1. 编写代码将原始数据转换为rosbag格式
  2. 编写ROS2节点，接收rosbag数据并实现对应功能

## 技术要求
- 编程语言：Python/C/C++均可
- 需使用ROS2最新稳定版本
- 代码应遵循ROS2的标准规范

## 提交内容
- 完整源代码（包含注释）
- 技术报告（至少1页）：
  - 描述实现思路
  - 说明ROS2节点设计
  - 展示运行结果
  - 与ROS1版本对比（如有）

## 截止日期
- 2024年3月23日

## 参考资源
- [ROS2官方文档](https://docs.ros.org/en/humble/index.html)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [rosbag2使用指南](https://docs.ros.org/en/humble/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)

