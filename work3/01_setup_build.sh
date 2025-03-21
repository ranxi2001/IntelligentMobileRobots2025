#!/bin/bash

# 阶段01：设置ROS2环境和编译工作空间
# 1. 设置ROS2环境
# 2. 创建工作空间结构
# 3. 编译ROS2包

# 设置日志文件
LOG_FILE="$(date +%Y%m%d_%H%M%S)_01_setup.log"
echo "所有输出将记录到日志文件: $LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1
echo "======== 阶段01-环境设置与编译 日志开始 $(date) ========"

set -e  # 遇到错误立即退出

# 设置变量
WORK_DIR=$(pwd)
SRC_DIR="${WORK_DIR}/src"

# 设置ROS2环境
echo "正在设置ROS2环境..."
if ! source /opt/ros/humble/setup.bash; then
    echo "❌ 错误: ROS2环境设置失败"
    exit 1
fi

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

# 检查可执行文件和脚本是否存在
echo "检查可执行文件和脚本..."
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

echo "✅ 阶段01完成: 环境设置和编译成功!"
echo "要继续执行下一阶段，请运行: ./02_convert_data.sh"

echo "======== 阶段01-环境设置与编译 日志结束 $(date) ========" 