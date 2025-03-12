#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
基于航位推算的机器人运动估计
生成占用栅格地图(Occupancy Grid)
"""

import numpy as np
import matplotlib.pyplot as plt
import struct
import os
from matplotlib.colors import LinearSegmentedColormap

# 设置matplotlib支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号

class LaserDataReader:
    """激光数据读取类"""
    def __init__(self, lms_file):
        self.lms_file = lms_file
        self.header = None
        self.scans = []
        self._read_lms_file()
    
    def _read_lms_file(self):
        """读取激光雷达扫描数据文件"""
        with open(self.lms_file, 'rb') as f:
            # 读取头部信息
            header_data = f.read(12)  # 3个float，每个4字节
            self.header = struct.unpack('fff', header_data)
            
            # 解析头部信息
            self.ang_range = self.header[0]  # 激光扫描范围
            self.ang_res = self.header[1]    # 角分辨率
            self.unit = self.header[2]       # 单位
            
            # 直接设置确认的最佳点数，通过数据分析验证为361
            self.max_dat_len = 361
            
            # 循环读取所有帧的激光数据
            while True:
                # 读取时间戳
                timestamp_data = f.read(4)  # long类型，4字节
                if not timestamp_data or len(timestamp_data) < 4:
                    break
                    
                timestamp = struct.unpack('l', timestamp_data)[0]
                
                # 读取激光数据
                laser_data = f.read(2 * self.max_dat_len)  # unsigned short类型，每个2字节
                if not laser_data or len(laser_data) < 2 * self.max_dat_len:
                    print(f"警告：在时间戳 {timestamp} 处读取的激光数据长度不一致")
                    break
                    
                # 解析激光数据
                format_str = '{}H'.format(self.max_dat_len)
                laser_values = struct.unpack(format_str, laser_data)
                
                # 保存帧数据
                self.scans.append({
                    'timestamp': timestamp,
                    'laser_data': np.array(laser_values)
                })
            
            print(f"成功读取 {len(self.scans)} 帧激光雷达数据")
    
    def get_scan_angles(self):
        """获取每个激光点的角度值（弧度）"""
        # 优化角度计算，确保从0度开始，正确覆盖180度范围
        angles = np.arange(0, self.max_dat_len) * self.ang_res * np.pi / 180.0
        return angles
    
    def convert_to_distances(self, laser_data):
        """将激光数据转换为实际距离（米）"""
        return laser_data / self.unit


class TrajectoryReader:
    """轨迹数据读取类"""
    def __init__(self, nav_file):
        self.nav_file = nav_file
        self.trajectory = []
        self._read_nav_file()
    
    def _read_nav_file(self):
        """读取轨迹数据文件"""
        with open(self.nav_file, 'r') as f:
            # 跳过标题行
            next(f)
            
            # 读取所有轨迹点
            for line in f:
                data = line.strip().split()
                if len(data) >= 7:
                    self.trajectory.append({
                        'timestamp': int(data[0]),
                        'ang_x': float(data[1]),
                        'ang_y': float(data[2]),
                        'ang_z': float(data[3]),
                        'shv_x': float(data[4]),
                        'shv_y': float(data[5]),
                        'shv_z': float(data[6])
                    })
            
            print(f"成功读取 {len(self.trajectory)} 个轨迹点")
    
    def find_closest_pose(self, timestamp):
        """根据时间戳查找最接近的姿态"""
        if not self.trajectory:
            return None
            
        # 找到时间戳差值最小的轨迹点
        closest_idx = 0
        min_diff = abs(self.trajectory[0]['timestamp'] - timestamp)
        
        for i, pose in enumerate(self.trajectory):
            diff = abs(pose['timestamp'] - timestamp)
            if diff < min_diff:
                min_diff = diff
                closest_idx = i
        
        return self.trajectory[closest_idx]


class OccupancyGridMapper:
    """占用栅格地图创建类"""
    def __init__(self, resolution=0.2, size_x=500, size_y=500):
        """
        初始化占用栅格地图
        
        参数:
            resolution: 栅格分辨率（米/格）
            size_x, size_y: 地图尺寸（格数）
        """
        self.resolution = resolution
        self.size_x = size_x
        self.size_y = size_y
        # 使用二进制地图：0=空闲，1=占用
        self.grid_map = np.zeros((size_x, size_y), dtype=int)
        # 原点设在地图中心
        self.origin_x = size_x // 2
        self.origin_y = size_y // 2
        # 追踪地图边界
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        # 激光点数量累积计数
        self.count_map = np.zeros((size_x, size_y), dtype=int)
    
    def world_to_grid(self, x, y):
        """将世界坐标转换为栅格坐标"""
        grid_x = int(self.origin_x + x / self.resolution)
        grid_y = int(self.origin_y + y / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """将栅格坐标转换为世界坐标"""
        world_x = (grid_x - self.origin_x) * self.resolution
        world_y = (grid_y - self.origin_y) * self.resolution
        return world_x, world_y
    
    def update_map(self, points, threshold=1):
        """
        使用激光点更新占用栅格地图
        
        参数:
            points: 激光点在世界坐标系中的坐标，形状为 (n, 2)
            threshold: 阈值，一个栅格中激光点数超过此值时标记为占用
        """
        # 添加额外的预处理，过滤掉可能的异常点
        # 检测离群点 - 这些点可能是噪声
        if len(points) > 10:  # 确保有足够的点进行统计
            x_coords = points[:, 0]
            y_coords = points[:, 1]
            
            # 计算距离原点的距离
            distances = np.sqrt(x_coords**2 + y_coords**2)
            
            # 使用四分位数范围(IQR)方法检测离群点
            q1 = np.percentile(distances, 25)
            q3 = np.percentile(distances, 75)
            iqr = q3 - q1
            
            # 定义离群点的阈值（通常是1.5倍IQR）
            lower_bound = q1 - 1.5 * iqr
            upper_bound = q3 + 1.5 * iqr
            
            # 过滤离群点
            inlier_mask = (distances >= lower_bound) & (distances <= upper_bound)
            points = points[inlier_mask]
        
        # 统计每个栅格中的激光点数
        for x, y in points:
            # 更新地图边界
            self.min_x = min(self.min_x, x)
            self.max_x = max(self.max_x, x)
            self.min_y = min(self.min_y, y)
            self.max_y = max(self.max_y, y)
            
            grid_x, grid_y = self.world_to_grid(x, y)
            
            # 检查是否在地图范围内
            if 0 <= grid_x < self.size_x and 0 <= grid_y < self.size_y:
                self.count_map[grid_x, grid_y] += 1
        
        # 更新占用栅格地图，使用更严格的阈值处理左侧区域
        for x in range(self.size_x):
            for y in range(self.size_y):
                # 转换回世界坐标计算位置
                world_x, world_y = self.grid_to_world(x, y)
                
                # 左侧区域（x值较小的区域）使用更高的阈值
                local_threshold = threshold
                if world_x < 0:  # 左侧区域
                    local_threshold = max(2, threshold + 1)  # 更严格的阈值
                
                if self.count_map[x, y] > local_threshold:
                    self.grid_map[x, y] = 1
    
    def visualize_map(self, robot_trajectory=None, save_path=None):
        """
        可视化占用栅格地图
        
        参数:
            robot_trajectory: 机器人轨迹点列表，形状为 (n, 2)
            save_path: 保存图像的路径
        """
        # 创建图形
        plt.figure(figsize=(12, 12))
        
        # 创建红白配色方案
        red_white_cmap = LinearSegmentedColormap.from_list('red_white_cmap', 
                                                          ['#FFFFFF', '#FF0000'])
        
        # 提取需要显示的栅格区域
        visible_grid = self.grid_map
        
        # 绘制占用栅格地图
        plt.imshow(visible_grid.T, cmap=red_white_cmap, origin='lower',
                  extent=[
                      -self.origin_x * self.resolution,
                      (self.size_x - self.origin_x) * self.resolution,
                      -self.origin_y * self.resolution,
                      (self.size_y - self.origin_y) * self.resolution
                  ],
                  aspect='equal')
        
        # 设置白色背景
        plt.gca().set_facecolor('#FFFFFF')
        
        # 添加网格线
        plt.grid(True, color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # 设置固定的显示范围
        plt.xlim(-10, 45)
        plt.ylim(-14, 50)
        
        # 设置坐标轴刻度
        grid_step = 10  # 网格线间隔（米）
        plt.xticks(np.arange(-10, 46, grid_step))
        plt.yticks(np.arange(-15, 51, grid_step))
        
        # 绘制机器人轨迹
        if robot_trajectory is not None and len(robot_trajectory) > 0:
            traj_x, traj_y = zip(*robot_trajectory)
            plt.plot(traj_x, traj_y, 'b-', linewidth=2, label='机器人轨迹')
            # 标记起点和终点
            plt.plot(traj_x[0], traj_y[0], 'go', markersize=10, label='起点')
            plt.plot(traj_x[-1], traj_y[-1], 'ko', markersize=10, label='终点')
        
        # 添加图例
        plt.legend(loc='upper right')
        
        # 添加标题和轴标签
        plt.title('占用栅格地图 (白色=空闲, 红色=障碍物)', fontsize=14)
        plt.xlabel('X (m)', fontsize=12)
        plt.ylabel('Y (m)', fontsize=12)
        
        # 调整坐标轴颜色为黑色
        plt.tick_params(colors='black')
        plt.gca().xaxis.label.set_color('black')
        plt.gca().yaxis.label.set_color('black')
        plt.gca().title.set_color('black')
        
        # 保存图像
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
            print(f"地图已保存至 {save_path}")
        
        plt.show()


def laser_to_world(robot_pose, laser_ranges, laser_angles):
    """
    将激光坐标转换到世界坐标系
    
    参数:
        robot_pose: 机器人姿态 {ang_z, shv_x, shv_y}
        laser_ranges: 激光距离数组
        laser_angles: 激光角度数组（弧度）
    
    返回:
        world_points: 激光点在世界坐标系中的坐标，形状为 (n, 2)
    """
    # 机器人位姿
    robot_x = robot_pose['shv_x']
    robot_y = robot_pose['shv_y']
    robot_theta = robot_pose['ang_z']
    
    # 增强过滤条件，避免将异常值解释为障碍物
    min_valid_range = 0.1  # 设置最小有效距离（米）
    max_valid_range = 50.0  # 设置最大有效距离（米）
    valid_indices = (laser_ranges > min_valid_range) & (laser_ranges < max_valid_range)
    
    # 额外过滤左侧区域异常点（约0-20度范围）
    # 在这个范围内，应用更严格的过滤条件
    left_side_angles = (laser_angles >= 0) & (laser_angles <= np.radians(20))
    left_side_indices = np.where(left_side_angles)[0]
    
    # 对左侧区域进行特殊处理
    for i in left_side_indices:
        # 如果有连续的点突然跳变，可能是噪声
        if i > 0 and i < len(laser_ranges) - 1:
            # 计算与相邻点的距离差异
            diff_prev = abs(laser_ranges[i] - laser_ranges[i-1])
            diff_next = abs(laser_ranges[i] - laser_ranges[i+1])
            
            # 如果与相邻点的差异过大，标记为无效
            if diff_prev > 1.0 and diff_next > 1.0:
                valid_indices[i] = False
    
    valid_ranges = laser_ranges[valid_indices]
    valid_angles = laser_angles[valid_indices]
    
    # 计算激光点在机器人坐标系中的坐标
    laser_x = valid_ranges * np.cos(valid_angles)
    laser_y = valid_ranges * np.sin(valid_angles)
    
    # 创建旋转矩阵 R_(ak)
    cos_theta = np.cos(robot_theta)
    sin_theta = np.sin(robot_theta)
    
    # 转换到世界坐标系: p_w^i = R_(ak) * p_k^i + (x_k, y_k)^t
    world_x = robot_x + laser_x * cos_theta - laser_y * sin_theta
    world_y = robot_y + laser_x * sin_theta + laser_y * cos_theta
    
    return np.column_stack((world_x, world_y))


def main():
    """主函数，实现整体处理流程"""
    # 文件路径设置
    print("====== 基于航位推算的机器人运动估计 - 占用栅格地图生成 ======")
    
    # 允许用户指定文件路径或使用默认路径
    laser_file = input("请输入激光雷达数据文件路径 (默认为'./data/URG_X_20130903_195003.lms'): ").strip()
    if not laser_file:
        laser_file = "./data/URG_X_20130903_195003.lms"
    
    trajectory_file = input("请输入轨迹数据文件路径 (默认为'./data/ld.nav'): ").strip()
    if not trajectory_file:
        trajectory_file = "./data/ld.nav"
    
    # 检查文件是否存在
    if not os.path.exists(laser_file):
        print(f"错误：激光数据文件 {laser_file} 不存在")
        return
    
    if not os.path.exists(trajectory_file):
        print(f"错误：轨迹数据文件 {trajectory_file} 不存在")
        return
    
    # 推荐参数设置
    print("\n推荐参数设置：")
    print("  - 分辨率: 0.2米/格 (高精度显示障碍物)")
    print("  - 地图尺寸: 500格 (确保覆盖完整场景)")
    print("  - 阈值: 1 (将所有检测到的激光点显示为障碍物)")
    
    # 设置栅格地图参数
    try:
        resolution = float(input("\n请输入栅格地图分辨率(米/格)，默认为0.2: ") or 0.2)
        size = int(input("请输入栅格地图尺寸(格数)，默认为500: ") or 500)
        threshold = int(input("请输入占用阈值(点数大于此值标记为占用)，默认为1: ") or 1)
    except ValueError:
        print("输入无效，使用默认值")
        resolution = 0.2
        size = 500
        threshold = 1
    
    # 读取激光数据和轨迹数据
    print(f"\n1. 正在读取激光数据文件：{laser_file}")
    laser_reader = LaserDataReader(laser_file)
    
    print(f"2. 正在读取轨迹数据文件：{trajectory_file}")
    trajectory_reader = TrajectoryReader(trajectory_file)
    
    # 获取激光点角度
    laser_angles = laser_reader.get_scan_angles()
    
    # 创建占用栅格地图
    print(f"3. 创建占用栅格地图，分辨率: {resolution}米/格，尺寸: {size}x{size}格")
    grid_mapper = OccupancyGridMapper(resolution=resolution, size_x=size, size_y=size)
    
    # 存储机器人轨迹
    robot_trajectory = []
    
    # 处理每一帧激光数据
    print("4. 开始处理激光雷达数据...")
    print("   - 将激光点从激光坐标系转换到全局坐标系")
    print("   - 更新占用栅格地图")
    
    # 设置进度显示间隔
    process_interval = max(1, len(laser_reader.scans) // 20)
    
    # 首先收集所有轨迹点
    print("   - 收集轨迹信息并预估地图边界")
    for pose in trajectory_reader.trajectory:
        robot_trajectory.append((pose['shv_x'], pose['shv_y']))
    
    # 开始处理激光扫描数据
    for i, scan in enumerate(laser_reader.scans):
        # 找到对应的机器人姿态
        pose = trajectory_reader.find_closest_pose(scan['timestamp'])
        if pose is None:
            continue
        
        # 将激光数据转换为实际距离（米）
        laser_ranges = laser_reader.convert_to_distances(scan['laser_data'])
        
        # 将激光点转换到世界坐标系
        world_points = laser_to_world(pose, laser_ranges, laser_angles)
        
        # 更新占用栅格地图
        grid_mapper.update_map(world_points, threshold=threshold)
        
        # 打印进度
        if i % process_interval == 0:
            progress = (i + 1) / len(laser_reader.scans) * 100
            print(f"   处理进度: {progress:.1f}%")
    
    print("5. 处理完成！")
    
    # 可视化和保存结果
    save_path = input("请输入地图保存路径 (默认为'./work2/occupancy_grid.png'): ").strip()
    if not save_path:
        save_path = "./work2/occupancy_grid.png"
    
    print(f"6. 正在可视化占用栅格地图并保存至 {save_path}")
    print("   - 绿色区域表示障碍物")
    print("   - 黑色区域表示空闲空间")
    
    # 可视化结果
    grid_mapper.visualize_map(robot_trajectory, save_path=save_path)
    
    print("====== 处理完成 ======")


if __name__ == "__main__":
    main()
