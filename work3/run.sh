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
    for i in {1..60}; do
        if ! ps -p $MAPPER_PID > /dev/null; then
            echo "✅ 节点已正常关闭"
            break
        fi
        echo "等待节点关闭... ($i/60)"
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