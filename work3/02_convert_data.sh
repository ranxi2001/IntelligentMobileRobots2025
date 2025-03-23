#!/bin/bash

# 阶段02：将原始数据转换为ROS2 bag格式
# 将LMS和NAV原始数据转换为ROS2 bag格式，供后续处理使用

# 设置日志文件
LOG_FILE="$(date +%Y%m%d_%H%M%S)_02_convert.log"
echo "所有输出将记录到日志文件: $LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1
echo "======== 阶段02-数据转换 日志开始 $(date) ========"

set -e  # 遇到错误立即退出

# 设置变量
WORK_DIR=$(pwd)
SRC_DIR="${WORK_DIR}/src"
LMS_FILE="../data/URG_X_20130903_195003.lms"
NAV_FILE="../data/ld.nav"
BAG_PATH="lms_data.bag"

# 设置ROS2环境
echo "正在设置ROS2环境..."
if ! source /opt/ros/humble/setup.bash; then
    echo "❌ 错误: ROS2环境设置失败"
    exit 1
fi

# 设置工作空间环境
if [ -f "install/setup.bash" ]; then
    echo "设置工作空间环境..."
    source install/setup.bash
else
    echo "❌ 错误: 工作空间环境未找到，请先运行 01_setup_build.sh"
    exit 1
fi

# 检查原始数据文件是否存在
echo "检查原始数据文件..."
if [ ! -f "$LMS_FILE" ]; then
    echo "❌ 错误: 找不到激光雷达数据文件 $LMS_FILE"
    exit 1
fi

if [ ! -f "$NAV_FILE" ]; then
    echo "❌ 错误: 找不到轨迹数据文件 $NAV_FILE"
    exit 1
fi

# 检查可执行文件
if [ ! -f "install/lms_data/lib/lms_data/lms_to_rosbag" ]; then
    echo "提示: 没有找到lms_to_rosbag可执行文件，将直接使用Python脚本"
    LMS_TO_ROSBAG_CMD="python3 ${SRC_DIR}/lms_data/lms_data/lms_to_rosbag.py"
else
    LMS_TO_ROSBAG_CMD="ros2 run lms_data lms_to_rosbag"
fi

# 添加超时控制功能
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

# 步骤1：将原始数据转换为rosbag格式
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
    exit 1
fi

echo "✅ 阶段02完成: 数据转换成功! 已生成ROS2 bag文件: ${BAG_PATH}"
echo "要继续执行下一阶段，请运行: ./03_map_generation.sh"

echo "======== 阶段02-数据转换 日志结束 $(date) ========" 