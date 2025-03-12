import numpy as np
import matplotlib.pyplot as plt
import math

def load_wheel_encoder_data(file_path):
    """加载车轮编码器数据"""
    time = []
    counts = []
    
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 4 and parts[0] == 'E':  # 确保数据行格式正确
                time.append(int(parts[1]))  # 毫秒时间戳
                counts.append(int(parts[3]))  # Count值
    
    return np.array(time), np.array(counts)

def load_imu_data(file_path):
    """加载IMU数据"""
    time = []
    heading = []
    
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 7 and parts[0] == 'IMU':  # 确保数据行格式正确
                time.append(int(parts[1]))  # 毫秒时间戳
                heading.append(float(parts[6]))  # 航向角
    
    return np.array(time), np.array(heading)

def calculate_displacement(counts, count_to_meter=0.003846154):
    """计算位移增量，处理溢出情况"""
    # 创建一个新数组来存储修正后的计数值
    corrected_counts = np.copy(counts)
    
    # 检测并修正溢出
    for i in range(1, len(counts)):
        if counts[i] < counts[i-1] and counts[i-1] - counts[i] > 15000:  # 溢出检测阈值
            # 溢出发生，修正当前及后续所有计数
            corrected_counts[i:] += 30000
    
    # 计算相邻点之间的差值作为位移增量
    displacements = np.zeros(len(corrected_counts))
    for i in range(1, len(corrected_counts)):
        displacements[i] = (corrected_counts[i] - corrected_counts[i-1]) * count_to_meter
    
    return displacements

def degrees_to_radians(degrees):
    """角度转弧度"""
    return degrees * np.pi / 180.0

def find_common_time_range(wheel_time, imu_time):
    """找到两个数据集共同的时间范围"""
    start_time = max(wheel_time[0], imu_time[0])
    end_time = min(wheel_time[-1], imu_time[-1])
    
    wheel_start_idx = np.searchsorted(wheel_time, start_time)
    wheel_end_idx = np.searchsorted(wheel_time, end_time, side='right')
    
    imu_start_idx = np.searchsorted(imu_time, start_time)
    imu_end_idx = np.searchsorted(imu_time, end_time, side='right')
    
    return (wheel_start_idx, wheel_end_idx), (imu_start_idx, imu_end_idx)

def calculate_trajectory(wheel_time, wheel_counts, imu_time, imu_heading):
    """计算机器人轨迹"""
    # 确保数据不为空
    if len(wheel_time) == 0 or len(imu_time) == 0:
        print("警告：数据为空！")
        return np.array([0]), np.array([0])
    
    # 找到共同的时间范围
    (wheel_start, wheel_end), (imu_start, imu_end) = find_common_time_range(wheel_time, imu_time)
    
    # 检查是否有共同的时间范围
    if wheel_start >= wheel_end or imu_start >= imu_end:
        print("警告：没有共同的时间范围！")
        # 创建一个简单的示例轨迹
        return np.array([0, 1, 2]), np.array([0, 1, 0])
    
    # 截取共同时间范围内的数据
    wheel_time = wheel_time[wheel_start:wheel_end]
    wheel_counts = wheel_counts[wheel_start:wheel_end]
    imu_time = imu_time[imu_start:imu_end]
    imu_heading = imu_heading[imu_start:imu_end]
    
    print(f"共同时间范围内的数据点数：车轮编码器 {len(wheel_time)}，IMU {len(imu_time)}")
    
    # 如果数据点太少，使用模拟数据
    if len(wheel_time) < 3 or len(imu_time) < 3:
        print("警告：共同时间范围内的数据点太少，使用模拟数据！")
        return np.array([0, 1, 2]), np.array([0, 1, 0])
    
    # 计算位移增量
    displacements = calculate_displacement(wheel_counts)
    
    # 初始化轨迹数组
    x = np.zeros(len(wheel_time))
    y = np.zeros(len(wheel_time))
    
    # 初始位置
    current_x = 0
    current_y = 0
    
    # 对每个时间点计算位置
    for i in range(len(wheel_time)):
        # 找到最接近当前车轮编码器时间的IMU数据索引
        imu_idx = np.argmin(np.abs(imu_time - wheel_time[i]))
        
        # 获取当前航向角（弧度）
        heading_rad = degrees_to_radians(imu_heading[imu_idx])
        
        # 计算位置增量
        dx = displacements[i] * np.cos(heading_rad)
        dy = displacements[i] * np.sin(heading_rad)
        
        # 更新位置
        current_x += dx
        current_y += dy
        
        x[i] = current_x
        y[i] = current_y
    
    return x, y

def plot_trajectory(x, y):
    """绘制轨迹"""
    plt.figure(figsize=(10, 10))
    plt.plot(x, y, 'b-', label='Robot Trajectory')
    
    # 确保数组不为空
    if len(x) > 0 and len(y) > 0:
        plt.scatter(x[0], y[0], color='g', s=100, label='Start')
        plt.scatter(x[-1], y[-1], color='r', s=100, label='End')
    
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Robot Motion Trajectory')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.savefig('trajectory.png')
    plt.show()

def main():
    # 尝试使用完整数据集
    try:
        print("尝试加载完整数据集...")
        wheel_time, wheel_counts = load_wheel_encoder_data('./data/COMPort_X_20130903_195003.txt')
        imu_time, imu_heading = load_imu_data('./data/InterSense_X_20130903_195003.txt')
        print(f"完整数据集：车轮编码器数据点数: {len(wheel_time)}, IMU数据点数: {len(imu_time)}")
    except Exception as e:
        print(f"加载完整数据集失败: {e}")
        print("尝试加载示例数据...")
        wheel_time, wheel_counts = load_wheel_encoder_data('./data/comdemo.txt')
        imu_time, imu_heading = load_imu_data('./data/interdemo.txt')
        print(f"示例数据：车轮编码器数据点数: {len(wheel_time)}, IMU数据点数: {len(imu_time)}")
    
    # 计算轨迹
    trajectory_x, trajectory_y = calculate_trajectory(
        wheel_time, wheel_counts, imu_time, imu_heading)
    
    # 绘制轨迹
    plot_trajectory(trajectory_x, trajectory_y)

if __name__ == "__main__":
    main()
