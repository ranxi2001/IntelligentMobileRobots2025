#!/bin/bash

# 作业3：使用ROS2重新实现作业2中的激光雷达数据处理和占用栅格地图生成
# 脚本说明：
# 1. 首先执行setup.sh创建ROS2工作空间结构
# 2. 构建ROS2包
# 3. 将原始LMS和NAV数据转换为rosbag格式
# 4. 运行栅格地图生成节点，接收rosbag数据并生成地图

set -e  # 遇到错误立即退出

# 设置ROS2环境
source /opt/ros/humble/setup.bash || echo "警告: ROS2环境设置失败，尝试继续运行..."

# 设置变量 - 使用正确的路径
WORK_DIR=$(pwd)
SRC_DIR="${WORK_DIR}/src"
LMS_FILE="../data/URG_X_20130903_195003.lms"
NAV_FILE="../data/ld.nav"
BAG_PATH="lms_data.bag"
MAP_PATH="occupancy_grid.png"

echo "======== 开始执行作业3 ========"

# 步骤1：创建ROS2工作空间结构
echo "正在创建ROS2工作空间..."
bash ./setup.sh

# 步骤2：构建ROS2包
echo "正在构建ROS2包..."
cd ${WORK_DIR}
rm -rf build/ install/ log/  # 清理之前的构建
colcon build --symlink-install

# 设置环境变量
source install/setup.bash || echo "警告: 工作空间环境设置失败，尝试继续运行..."

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
    echo "错误: 找不到 ${SRC_DIR}/lms_data/lms_data/lms_to_rosbag.py 文件"
    exit 1
fi

if [ ! -f "${SRC_DIR}/lms_mapper/lms_mapper/lms_mapper_node.py" ]; then
    echo "错误: 找不到 ${SRC_DIR}/lms_mapper/lms_mapper/lms_mapper_node.py 文件"
    exit 1
fi

# 步骤3：将原始数据转换为rosbag格式
echo "正在转换原始数据为rosbag格式..."
echo "执行: ${LMS_TO_ROSBAG_CMD} ${LMS_FILE} ${NAV_FILE} ${BAG_PATH}"
${LMS_TO_ROSBAG_CMD} ${LMS_FILE} ${NAV_FILE} ${BAG_PATH}

# 检查rosbag是否生成
if [ ! -f "${BAG_PATH}" ] && [ ! -d "${BAG_PATH}" ]; then
    echo "错误: 未能创建rosbag文件 ${BAG_PATH}"
    exit 1
fi

# 步骤4：在另一个终端中运行激光雷达数据处理节点
echo "正在启动LMS Mapper节点..."
echo "执行: ${LMS_MAPPER_CMD} -p map_save_path:=${MAP_PATH}"
${LMS_MAPPER_CMD} -p map_save_path:=${MAP_PATH} &
MAPPER_PID=$!

# 等待节点启动
sleep 5
echo "节点已启动，PID: ${MAPPER_PID}"

# 步骤5：播放rosbag数据
echo "正在播放rosbag数据..."
if [ -f "${BAG_PATH}" ] || [ -d "${BAG_PATH}" ]; then
    ros2 bag play ${BAG_PATH}
else
    echo "错误: 找不到rosbag文件 ${BAG_PATH}"
    kill $MAPPER_PID || true
    exit 1
fi

# 等待处理完成（给足够时间保存地图）
echo "等待处理完成..."
sleep 15

# 关闭节点
echo "正在关闭节点..."
kill $MAPPER_PID || true

# 检查地图文件是否生成
if [ -f "${MAP_PATH}" ]; then
    echo "✅ 地图生成成功！保存在: ${MAP_PATH}"
else
    echo "⚠️ 注意: 未找到地图文件，可能在其他位置生成或出现错误"
fi

echo "======== 作业3执行完成 ========"
echo "生成的栅格地图应保存到: ${MAP_PATH}"
echo "如需查看地图，请使用图片查看器打开 ${MAP_PATH}" 