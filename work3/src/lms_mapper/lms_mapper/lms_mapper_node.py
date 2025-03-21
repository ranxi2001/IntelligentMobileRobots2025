#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 LMS 栅格地图生成节点 - 从LaserScan和Odometry消息创建占用栅格地图
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped
from std_msgs.msg import Header
import math
import threading
import time
from collections import Counter
import copy

# 设置matplotlib支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号

# 移除固定的地图边界常量，改为函数来动态计算边界
def calculate_map_bounds(min_x, max_x, min_y, max_y, padding=5.0):
    """计算地图显示边界，带有边距"""
    # 如果没有有效数据，使用默认值
    if min_x == float('inf') or max_x == float('-inf') or min_y == float('inf') or max_y == float('-inf'):
        return (-10, 45, -14, 70)  # 修改默认值，将y轴上限从50扩大到70
        
    # 添加边距
    x_range = max_x - min_x
    y_range = max_y - min_y
    
    # 确保边距至少占范围的10%，并且对y轴上限特别关注
    x_padding = max(padding, x_range * 0.2)  # 增加边距百分比
    y_padding = max(padding, y_range * 0.25)  # 增加y轴的边距比例
    
    xlim = (min_x - x_padding, max_x + x_padding)
    ylim = (min_y - y_padding, max_y + y_padding * 1.5)  # 对y轴上限额外增加空间
    
    # 确保显示范围足够大
    if ylim[1] < 100:
        ylim = (ylim[0], max(100, ylim[1]))
    if xlim[1] < 80:
        xlim = (xlim[0], max(80, xlim[1]))
    
    # 确保下限也足够低，能显示负坐标
    if xlim[0] > -20:
        xlim = (min(-20, xlim[0]), xlim[1])
    if ylim[0] > -20:
        ylim = (min(-20, ylim[0]), ylim[1])
        
    return xlim + ylim

def quaternion_to_euler(q):
    """四元数转欧拉角"""
    # 提取四元数分量
    x, y, z, w = q.x, q.y, q.z, q.w
    
    # 计算欧拉角
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw

def laser_to_world(robot_pose, laser_ranges, laser_angles):
    """激光数据转换到世界坐标系"""
    # 提取机器人位置和方向
    robot_x = robot_pose.position.x
    robot_y = robot_pose.position.y
    _, _, robot_theta = quaternion_to_euler(robot_pose.orientation)
    
    # 初始化结果数组
    world_points = np.zeros((len(laser_ranges), 2))
    
    # 对每个激光点进行转换
    for i, (angle, distance) in enumerate(zip(laser_angles, laser_ranges)):
        # 跳过无效点（通常是inf或非常大的值）
        if not np.isfinite(distance) or distance > 30.0:
            world_points[i] = [float('nan'), float('nan')]
            continue
            
        # 计算激光点在激光坐标系下的坐标
        # 注意：激光坐标系中x轴向前，y轴向左
        lx = distance * np.cos(angle)
        ly = distance * np.sin(angle)
        
        # 转换到世界坐标系
        # 考虑机器人的位置和朝向
        wx = robot_x + lx * np.cos(robot_theta) - ly * np.sin(robot_theta)
        wy = robot_y + lx * np.sin(robot_theta) + ly * np.cos(robot_theta)
        
        world_points[i] = [wx, wy]
    
    # 过滤掉NaN值
    valid_points = world_points[~np.isnan(world_points[:, 0])]
    
    return valid_points

class OccupancyGridMapper:
    """占用栅格地图生成器"""
    def __init__(self, resolution=0.1, size_x=1000, size_y=1000):
        """
        初始化占用栅格地图
        
        参数:
            resolution: 栅格分辨率（米/格）
            size_x, size_y: 地图尺寸（格数）
        """
        self.resolution = resolution
        self.size_x = size_x
        self.size_y = size_y
        # 使用概率地图：0-100 (0=空闲, 100=占用, -1=未知)
        self.grid_map = np.ones((size_x, size_y), dtype=np.int8) * -1
        # 原点设在地图中心
        self.origin_x = size_x // 2
        self.origin_y = size_y // 2
        # 追踪地图边界
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        # 激光点数量累积计数
        self.hit_map = np.zeros((size_x, size_y), dtype=np.int16)
        self.miss_map = np.zeros((size_x, size_y), dtype=np.int16)
    
    def world_to_grid(self, x, y):
        """世界坐标系转栅格坐标系
        
        参数:
            x: 世界坐标系下的X坐标
            y: 世界坐标系下的Y坐标
            
        返回:
            (grid_x, grid_y): 栅格坐标系下的整数坐标
        """
        try:
            # 检查输入坐标是否在合理范围内 - 扩大允许范围
            if not -200 < x < 200 or not -200 < y < 200:
                self.get_logger().warn(f'世界坐标超出扩展范围: x={x}, y={y}')
            
            # 将世界坐标转换为栅格坐标
            # origin_x和origin_y表示地图原点(0,0)在栅格中的位置（通常是地图中心）
            # resolution是每个栅格代表的实际距离（单位：米）
            grid_x = int(self.origin_x + x / self.resolution)
            grid_y = int(self.origin_y + y / self.resolution)
            
            # 调试输出 - 每500次转换输出一次以避免日志过多
            if not hasattr(self, '_world_to_grid_count'):
                self._world_to_grid_count = 0
            self._world_to_grid_count += 1
            
            if self._world_to_grid_count % 500 == 0:
                self.get_logger().info(f'【坐标转换】世界坐标(x={x:.2f},y={y:.2f}) --> 栅格坐标(grid_x={grid_x},grid_y={grid_y})')
                self.get_logger().info(f'【参数确认】原点(origin_x={self.origin_x},origin_y={self.origin_y}), 分辨率={self.resolution}米/格')
            
            # 扩大栅格范围处理 - 动态处理超出范围的坐标
            # 如果坐标超出范围，记录日志但仍返回实际计算值，由调用者决定如何处理
            if grid_x < 0 or grid_x >= self.size_x or grid_y < 0 or grid_y >= self.size_y:
                if self._world_to_grid_count % 100 == 0:  # 减少日志频率
                    self.get_logger().warn(f'栅格坐标超出地图范围: (grid_x={grid_x},grid_y={grid_y}), 来自世界坐标(x={x:.2f},y={y:.2f})')
                
                # 仍然需要限制在合理范围内以避免数组访问错误
                grid_x = max(0, min(grid_x, self.size_x - 1))
                grid_y = max(0, min(grid_y, self.size_y - 1))
            
            return grid_x, grid_y
        except Exception as e:
            self.get_logger().error(f'世界坐标转栅格坐标出错: {str(e)}')
            # 返回地图中心作为默认值
            return self.origin_x, self.origin_y
    
    def grid_to_world(self, grid_x, grid_y):
        """将栅格坐标转换为世界坐标"""
        world_x = (grid_x - self.origin_x) * self.resolution
        world_y = (grid_y - self.origin_y) * self.resolution
        return world_x, world_y
    
    def ray_trace(self, start_x, start_y, end_x, end_y):
        """
        使用Bresenham算法进行光线追踪，获取从起点到终点的所有栅格
        返回所有栅格坐标的列表
        """
        # 转换为栅格坐标
        sx, sy = self.world_to_grid(start_x, start_y)
        ex, ey = self.world_to_grid(end_x, end_y)
        
        # 确保坐标在地图范围内
        if not (0 <= sx < self.size_x and 0 <= sy < self.size_y and 
                0 <= ex < self.size_x and 0 <= ey < self.size_y):
            return []
        
        # Bresenham算法实现
        dx = abs(ex - sx)
        dy = abs(ey - sy)
        sx_step = 1 if sx < ex else -1
        sy_step = 1 if sy < ey else -1
        err = dx - dy
        
        cells = []
        current_x, current_y = sx, sy
        
        while True:
            cells.append((current_x, current_y))
            
            if current_x == ex and current_y == ey:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                current_x += sx_step
            if e2 < dx:
                err += dx
                current_y += sy_step
        
        return cells
    
    def update_map(self, robot_pose, points):
        """
        使用激光点更新占用栅格地图
        
        参数:
            robot_pose: 机器人位姿
            points: 激光点在世界坐标系中的坐标，形状为 (n, 2)
        """
        if len(points) == 0:
            return
            
        # 提取机器人位置
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        
        # 更新地图边界
        for x, y in points:
            self.min_x = min(self.min_x, x)
            self.max_x = max(self.max_x, x)
            self.min_y = min(self.min_y, y)
            self.max_y = max(self.max_y, y)
        
        # 使用逆传感器模型更新地图
        # 1. 所有激光点击中的栅格为占用
        for x, y in points:
            grid_x, grid_y = self.world_to_grid(x, y)
            
            # 检查是否在地图范围内
            if 0 <= grid_x < self.size_x and 0 <= grid_y < self.size_y:
                self.hit_map[grid_x, grid_y] += 1
        
        # 2. 从机器人到激光点之间的栅格为空闲
        for x, y in points:
            cells = self.ray_trace(robot_x, robot_y, x, y)
            
            # 最后一个栅格是障碍物，不算入空闲
            if cells:
                cells = cells[:-1]
                
            for cx, cy in cells:
                if 0 <= cx < self.size_x and 0 <= cy < self.size_y:
                    self.miss_map[cx, cy] += 1
        
        # 3. 更新概率地图
        # 使用简单的投票法：如果命中次数多于错过次数，则为占用
        for x in range(self.size_x):
            for y in range(self.size_y):
                hits = self.hit_map[x, y]
                misses = self.miss_map[x, y]
                
                # 只考虑有观测的栅格
                if hits > 0 or misses > 0:
                    # 简单投票法
                    if hits > misses:
                        self.grid_map[x, y] = 100  # 占用
                    else:
                        self.grid_map[x, y] = 0    # 空闲
    
    def get_ros_occupancy_grid(self, frame_id='map'):
        """获取ROS OccupancyGrid消息"""
        msg = OccupancyGrid()
        
        # 设置消息头
        msg.header = Header()
        msg.header.stamp.sec = int(time.time())
        msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
        msg.header.frame_id = frame_id
        
        # 设置地图元数据
        msg.info.resolution = self.resolution
        msg.info.width = self.size_x
        msg.info.height = self.size_y
        
        # 设置地图原点（左下角）
        origin_x = -self.origin_x * self.resolution
        origin_y = -self.origin_y * self.resolution
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        
        # 设置地图原点方向（默认为单位四元数）
        msg.info.origin.orientation.w = 1.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        
        # 填充地图数据
        # 注意：ROS中的OccupancyGrid是行优先的，而我们的地图是列优先的
        # 所以需要进行转置，使用flatten确保正确的顺序
        msg.data = self.grid_map.T.flatten().tolist()
        
        return msg
    
    def visualize_map(self, robot_trajectory=None, save_path=None):
        """
        可视化占用栅格地图
        
        参数:
            robot_trajectory: 机器人轨迹点列表，每个元素是(x, y)坐标
            save_path: 保存图像的路径
        """
        # 创建图形
        plt.figure(figsize=(12, 12))
        
        # 创建自定义的配色方案
        colors = [(0.0, 0.0, 1.0),  # 蓝色，表示空闲区域
                 (1.0, 1.0, 1.0),  # 白色，表示未知区域
                 (1.0, 0.0, 0.0)]  # 红色，表示障碍物
        
        # 创建从0到100的cmap
        n_bins = 101  # 100 + 1，包括-1
        occupancy_cmap = LinearSegmentedColormap.from_list('occupancy_cmap', colors, N=n_bins)
        
        # 绘制占用栅格地图
        plt.imshow(self.grid_map.T, cmap=occupancy_cmap, interpolation='nearest', origin='lower',
                  extent=[
                      -self.origin_x * self.resolution,
                      (self.size_x - self.origin_x) * self.resolution,
                      -self.origin_y * self.resolution,
                      (self.size_y - self.origin_y) * self.resolution
                  ])
        
        # 绘制机器人轨迹
        if robot_trajectory is not None and len(robot_trajectory) > 0:
            traj_x = [p[0] for p in robot_trajectory]
            traj_y = [p[1] for p in robot_trajectory]
            plt.plot(traj_x, traj_y, 'g-', linewidth=2, alpha=0.7)
            
            # 标记起点和终点
            plt.plot(traj_x[0], traj_y[0], 'go', markersize=10)
            plt.plot(traj_x[-1], traj_y[-1], 'gx', markersize=10)
        
        # 添加标题和坐标轴标签
        plt.title('占用栅格地图', fontsize=20)
        plt.xlabel('X (m)', fontsize=14)
        plt.ylabel('Y (m)', fontsize=14)
        plt.colorbar(label='占用概率')
        plt.grid(True, alpha=0.3)
        
        # 使用全局定义的固定显示范围
        plt.xlim(*calculate_map_bounds(self.min_x, self.max_x, self.min_y, self.max_y))
        plt.ylim(*calculate_map_bounds(self.min_y, self.max_y, self.min_x, self.max_x))
        
        # 设置网格线刻度
        x_range = calculate_map_bounds(self.max_x, self.max_x, self.min_y, self.max_y)[1] - calculate_map_bounds(self.min_x, self.min_x, self.min_y, self.max_y)[0]
        y_range = calculate_map_bounds(self.max_y, self.max_y, self.min_x, self.max_x)[3] - calculate_map_bounds(self.min_y, self.min_y, self.min_x, self.max_x)[2]
        grid_step_m = min(10, max(x_range, y_range) / 5)
        grid_step_m = max(2, round(grid_step_m))  # 至少2米，并取整
        plt.xticks(np.arange(math.floor(calculate_map_bounds(self.min_x, self.max_x, self.min_y, self.max_y)[0]), math.ceil(calculate_map_bounds(self.min_x, self.max_x, self.min_y, self.max_y)[1])+1, grid_step_m))
        plt.yticks(np.arange(math.floor(calculate_map_bounds(self.min_y, self.max_y, self.min_x, self.max_x)[2]), math.ceil(calculate_map_bounds(self.min_y, self.max_y, self.min_x, self.max_x)[3])+1, grid_step_m))
        
        # 保存图像或显示
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"地图已保存至: {save_path}")
        else:
            plt.show()
        
        # 关闭图形以释放资源
        plt.close()

    def update_map_batch(self, points):
        """批量更新占用栅格地图，提高性能"""
        if len(points) == 0:
            return
        
        try:
            # 更新地图边界
            min_x = np.min(points[:, 0])
            max_x = np.max(points[:, 0])
            min_y = np.min(points[:, 1])
            max_y = np.max(points[:, 1])
            
            self.min_x = min(self.min_x, min_x)
            self.max_x = max(self.max_x, max_x)
            self.min_y = min(self.min_y, min_y)
            self.max_y = max(self.max_y, max_y)
            
            # 批量转换为栅格坐标
            # 使用向量化操作进行坐标转换
            grid_x = np.clip(
                (self.origin_x + points[:, 0] / self.resolution).astype(np.int32),
                0, self.size_x - 1
            )
            grid_y = np.clip(
                (self.origin_y + points[:, 1] / self.resolution).astype(np.int32),
                0, self.size_y - 1
            )
            
            # 合并坐标为元组键
            grid_coords = list(zip(grid_x, grid_y))
            
            # 计算每个坐标出现的次数
            coord_counts = Counter(grid_coords)
            
            # 更新地图
            points_added = 0
            for (gx, gy), count in coord_counts.items():
                key = (gx, gy)
                # 累积计数
                self.occupancy_count[key] = self.occupancy_count.get(key, 0) + count
                
                # 更新栅格
                if self.occupancy_count[key] > self.threshold:
                    if self.grid_map[gx, gy] == 0:  # 只有当格子状态改变时才计数
                        points_added += 1
                    self.grid_map[gx, gy] = 1
            
            # 降低日志频率
            if points_added > 50:  # 只有当添加了大量新点时才记录
                self.get_logger().info(f'批量添加了{points_added}个新障碍物点到地图')
            
            # 定期打印地图统计信息
            if not hasattr(self, '_map_stats_count'):
                self._map_stats_count = 0
            
            total_obstacle_cells = np.sum(self.grid_map == 1)
            
            # 如果障碍物数量与上次统计相比增加了500个以上，或者是初次统计
            if self._map_stats_count == 0 or total_obstacle_cells - self._map_stats_count >= 500:
                self.get_logger().info(f'地图统计: 障碍物单元格={total_obstacle_cells}, 地图边界=[{self.min_x:.1f}, {self.max_x:.1f}]x[{self.min_y:.1f}, {self.max_y:.1f}]')
                self._map_stats_count = total_obstacle_cells
                
        except Exception as e:
            self.get_logger().error(f'批量更新地图时出错: {str(e)}')
            # 如果批量处理失败，回退到单点处理
            for point in points:
                self.update_map([point])


class LmsMapperNode(Node):
    """LMS数据处理和栅格地图生成节点"""
    
    def __init__(self):
        super().__init__('lms_mapper_node')
        
        # 声明参数
        self.declare_parameter('map_resolution', 0.1)  # 提高默认分辨率从0.2到0.1
        self.declare_parameter('map_size', 1000)  # 增加默认地图尺寸从500到1000
        self.declare_parameter('map_threshold', 1)
        self.declare_parameter('filter_points', True)
        self.declare_parameter('filter_threshold', 1.0)
        self.declare_parameter('map_save_path', 'occupancy_grid.png')
        self.declare_parameter('map_reset_enabled', False)  # 添加新参数，默认为False
        self.declare_parameter('clear_expired_obstacles_enabled', False)  # 添加新参数，默认为False
        self.declare_parameter('save_interval', 60.0)  # 新增参数：中间地图保存间隔，默认60秒
        
        # 获取参数
        self.resolution = self.get_parameter('map_resolution').value
        self.map_size = self.get_parameter('map_size').value
        self.threshold = self.get_parameter('map_threshold').value
        self.filter_points = self.get_parameter('filter_points').value
        self.filter_threshold = self.get_parameter('filter_threshold').value
        self.map_save_path = self.get_parameter('map_save_path').value
        self.map_reset_enabled = self.get_parameter('map_reset_enabled').value  # 读取参数
        self.clear_expired_obstacles_enabled = self.get_parameter('clear_expired_obstacles_enabled').value  # 读取参数
        self.save_interval = self.get_parameter('save_interval').value  # 读取中间地图保存间隔
        
        # 输出地图参数信息
        self.get_logger().info(f'地图参数: 尺寸={self.map_size}x{self.map_size}, 分辨率={self.resolution}米/格')
        
        # 输出地图重置功能状态
        if self.map_reset_enabled:
            self.get_logger().info('地图重置功能已启用，每处理%d个扫描后将重置地图' % self.map_reset_interval)
        else:
            self.get_logger().info('地图重置功能已禁用，将持续累积所有障碍物数据')
        
        # 输出中间地图保存间隔
        self.get_logger().info(f'中间地图保存间隔: {self.save_interval}秒')
            
        # 输出清除过期障碍物功能状态
        if self.clear_expired_obstacles_enabled:
            self.get_logger().info('清除过期障碍物功能已启用，超过%.1f秒的障碍物将被清除' % self.max_obstacle_age)
        else:
            self.get_logger().info('清除过期障碍物功能已禁用，将保留所有检测到的障碍物')
        
        # 创建占用栅格地图
        self.size_x = self.map_size
        self.size_y = self.map_size
        self.grid_map = np.zeros((self.size_x, self.size_y), dtype=np.int8)
        self.origin_x = self.size_x // 2
        self.origin_y = self.size_y // 2
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        self.occupancy_count = {}  # 用于跟踪每个栅格的激光点数
        
        # 存储机器人轨迹
        self.robot_trajectory = []
        
        # 增加轨迹同步变量
        self.traj_lock = threading.Lock()  # 添加线程锁保护轨迹数据
        self.last_scan_trajectory = []     # 上次扫描时的轨迹，用于同步
        self.new_trajectory_since_last_map = False  # 标记是否有新轨迹点
        
        # 记录最近收到的位姿
        self.latest_pose = None
        self.first_scan_received = False
        self.last_scan_time = None
        
        # 添加进度跟踪
        self.start_time = time.time()
        self.last_saved_time = self.start_time
        self.checkpoint_counter = 0
        self.processed_scans = 0
        self.processed_points = 0
        
        # 添加地图重置和更新控制变量
        self.map_reset_interval = 3  # 每隔多少个扫描点重置地图
        self.scan_count_since_reset = 0
        self.map_enabled = True  # 地图构建开关
        self.dynamic_map = True  # 是否使用动态地图（根据机器人位置实时更新）
        self.max_obstacle_age = 10.0  # 障碍物最大存活时间(秒)
        self.obstacle_timestamps = {}  # 记录每个障碍物的时间戳
        
        # 创建订阅
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 创建地图发布者
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )
        
        # 创建地图更新定时器
        self.map_timer = self.create_timer(1.0, self.publish_map)  # 每秒发布一次地图
        
        # 添加进度更新和中间结果保存定时器
        self.progress_timer = self.create_timer(5.0, self.update_progress)  # 每5秒更新一次进度
        
        self.get_logger().info('LMS mapper node已启动')
    
    def odom_callback(self, msg):
        """处理里程计消息，获取机器人位姿"""
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        
        # 降低日志频率，只在特定间隔输出详细信息
        log_interval = 100  # 每100个消息才打印一次详细信息
        should_log_details = not hasattr(self, '_odom_count') or self._odom_count % log_interval == 0
        
        if not hasattr(self, '_odom_count'):
            self._odom_count = 0
        self._odom_count += 1
        
        if should_log_details:
            self.get_logger().info(f'接收到里程计消息: frame_id={msg.header.frame_id}, child_frame_id={msg.child_frame_id}')
            self.get_logger().info(f'位置: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}')
            # 添加更详细的轨迹坐标输出
            self.get_logger().info(f'【轨迹坐标】x={position.x:.4f}, y={position.y:.4f}, z={position.z:.4f}')
        
        # 将四元数转换为欧拉角
        euler = self.quaternion_to_euler(
            orientation.x, 
            orientation.y, 
            orientation.z, 
            orientation.w
        )
        
        # 保存最新的位姿
        self.latest_pose = {
            'timestamp': self.get_clock().now().nanoseconds // 1000000,  # 转换为毫秒
            'x': position.x,
            'y': position.y,
            'theta': euler[2]  # yaw角
        }
        
        # 记录轨迹点 - 使用线程锁保护
        with self.traj_lock:
            self.robot_trajectory.append((position.x, position.y))
            self.new_trajectory_since_last_map = True  # 标记有新轨迹点
        
        # 减少轨迹点日志频率
        if should_log_details:
            self.get_logger().info(f'添加轨迹点 #{len(self.robot_trajectory)}: ({position.x:.2f}, {position.y:.2f})')
            # 每100个点打印一次统计信息，避免日志过多
            self.get_logger().info(f'轨迹统计: 总点数={len(self.robot_trajectory)}, 开始=({self.robot_trajectory[0][0]:.2f}, {self.robot_trajectory[0][1]:.2f}), 最新=({position.x:.2f}, {position.y:.2f})')
            # 添加欧拉角信息(度数)
            roll_deg = euler[0] * 180.0 / np.pi
            pitch_deg = euler[1] * 180.0 / np.pi
            yaw_deg = euler[2] * 180.0 / np.pi
            self.get_logger().info(f'【姿态角度】roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}°')
    
    def scan_callback(self, msg):
        """处理激光扫描消息，更新占用栅格地图"""
        try:
            # 降低日志频率，只在特定间隔输出详细信息
            if not hasattr(self, '_scan_count'):
                self._scan_count = 0
            self._scan_count += 1
            
            # 更新处理统计
            self.processed_scans += 1
            self.scan_count_since_reset += 1
            
            # 是否需要重置地图 - 只有在启用重置功能时才执行
            if self.map_reset_enabled and self.dynamic_map and self.scan_count_since_reset >= self.map_reset_interval:
                self.reset_map()
                self.scan_count_since_reset = 0
            
            # 每50个消息才打印一次详细信息
            log_interval = 50
            should_log_details = self._scan_count % log_interval == 0
            
            # 检查激光点数是否合理
            if len(msg.ranges) == 0 or len(msg.ranges) > 1000:  # 设定合理范围
                self.get_logger().warn(f'激光点数异常: {len(msg.ranges)}个点，跳过处理')
                return
                
            # 保存部分原始数据用于调试
            if not hasattr(self, 'debug_scans'):
                self.debug_scans = []
            if len(self.debug_scans) < 10:  # 只保存前10帧方便调试
                self.debug_scans.append({
                    'timestamp': self.get_clock().now().nanoseconds,
                    'ranges': list(msg.ranges),
                    'angles': [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
                })
        
            if self.latest_pose is None:
                self.get_logger().warn('接收到激光数据，但没有位姿信息，跳过处理')
                return
            
            # 获取当前扫描的时间戳和frame_id
            current_scan_time = self.get_clock().now().nanoseconds // 1000000
            
            if should_log_details:
                self.get_logger().info(f'接收到激光数据: frame_id={msg.header.frame_id}, 点数={len(msg.ranges)}')
                # 添加激光雷达坐标输出 - 使用机器人当前位姿(来自里程计)
                self.get_logger().info(f'【激光雷达坐标】x={self.latest_pose["x"]:.4f}, y={self.latest_pose["y"]:.4f}, theta={self.latest_pose["theta"]:.4f}')
            
            # 如果是第一帧，记录时间
            if not self.first_scan_received:
                self.first_scan_received = True
                self.last_scan_time = current_scan_time
                self.get_logger().info('接收到第一帧激光数据')
                
                # 首次接收，保存当前轨迹副本
                with self.traj_lock:
                    self.last_scan_trajectory = self.robot_trajectory.copy()
            
            # 计算与上一帧的时间差
            time_diff = current_scan_time - self.last_scan_time
            self.last_scan_time = current_scan_time
            
            # 如果时间差过小，可能是重复帧，跳过处理
            if time_diff < 10:  # 小于10毫秒认为是重复帧
                return
            
            # 获取激光扫描数据 - 使用numpy更高效地处理
            ranges = np.array(msg.ranges, dtype=np.float32)  # 明确指定数据类型以提高性能
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            angles = np.arange(len(ranges), dtype=np.float32) * angle_increment + angle_min
            
            # 添加调试信息
            if should_log_details and len(ranges) > 0:
                self.get_logger().info(f'接收到激光数据: {len(ranges)}个点, 角度范围=[{angle_min:.2f}, {angle_min + angle_increment * (len(ranges)-1):.2f}]')
            
            # 更严格的数据过滤 - 使用numpy高效处理
            valid_indices = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges > 0.1) & (ranges < 30.0)
            valid_count = np.sum(valid_indices)
            
            # 检查有效点百分比
            valid_percent = valid_count / len(ranges) * 100
            if valid_percent < 10:  # 如果有效点太少，可能是异常帧
                if should_log_details:
                    self.get_logger().warn(f'有效激光点比例过低: {valid_percent:.1f}%，可能是异常帧，跳过处理')
                return
            
            # 使用布尔索引更高效地提取有效数据
            valid_ranges = ranges[valid_indices]
            valid_angles = angles[valid_indices]
            
            # 更新处理点数统计
            self.processed_points += len(valid_ranges)
            
            if len(valid_ranges) == 0:
                if should_log_details:
                    self.get_logger().warn('没有有效的激光点')
                return
            
            if should_log_details:
                self.get_logger().info(f'有效激光点: {len(valid_ranges)}/{len(ranges)}')
            
            # 转换激光点到机器人坐标系 - 使用numpy向量化操作
            # 激光坐标系: x轴向前，y轴向左，原点在激光雷达位置
            laser_x = valid_ranges * np.cos(valid_angles)
            laser_y = valid_ranges * np.sin(valid_angles)
            
            # 坐标转换前检查
            if np.any(np.isnan(laser_x)) or np.any(np.isnan(laser_y)):
                if should_log_details:
                    self.get_logger().warn('激光坐标包含NaN值，进行过滤')
                valid_mask = ~np.isnan(laser_x) & ~np.isnan(laser_y)
                laser_x = laser_x[valid_mask]
                laser_y = laser_y[valid_mask]
            
            # 获取机器人位姿
            robot_x = self.latest_pose['x']
            robot_y = self.latest_pose['y']
            robot_theta = self.latest_pose['theta']
            
            # 更加严格的数据过滤 - 只保留当前时刻的激光数据
            current_time = time.time()
            
            # 转换到世界坐标系 - 使用numpy向量化操作
            # 坐标转换过程：
            # 1. 激光点坐标(laser_x,laser_y)是相对于激光雷达位置的坐标
            # 2. 机器人位姿(robot_x,robot_y,robot_theta)是机器人在世界坐标系中的位置和朝向
            # 3. 将激光点从激光坐标系转换到世界坐标系需要:
            #    a. 考虑机器人的位置偏移(robot_x,robot_y)
            #    b. 考虑机器人的朝向旋转(robot_theta)
            cos_theta = np.cos(robot_theta)
            sin_theta = np.sin(robot_theta)
            
            # 记录机器人位置信息以便日志记录
            if should_log_details:
                self.get_logger().info(f'机器人位姿: x={robot_x:.2f}, y={robot_y:.2f}, theta={robot_theta:.2f}, cos_theta={cos_theta:.2f}, sin_theta={sin_theta:.2f}')
            
            # 执行坐标转换: 从激光坐标系→世界坐标系
            # 公式: world_x = robot_x + laser_x * cos(theta) - laser_y * sin(theta)
            #       world_y = robot_y + laser_x * sin(theta) + laser_y * cos(theta)
            # 其中(robot_x,robot_y)是机器人在世界坐标系中的位置
            world_x = robot_x + laser_x * cos_theta - laser_y * sin_theta
            world_y = robot_y + laser_x * sin_theta + laser_y * cos_theta
            
            # 这样得到的world_x和world_y就是障碍物点在世界坐标系(全局坐标系)中的绝对坐标
            # 相对于地图原点(0,0)，而不是相对于机器人的相对坐标
            
            # 记录坐标转换范围
            if should_log_details:
                self.get_logger().info(f'激光点世界坐标范围: x=[{np.min(world_x):.2f}, {np.max(world_x):.2f}], y=[{np.min(world_y):.2f}, {np.max(world_y):.2f}]')
                
                # 添加前几个转换后的点的详细信息
                n_samples = min(5, len(world_x))
                if n_samples > 0:
                    sample_indices = np.linspace(0, len(world_x)-1, n_samples, dtype=int)
                    sample_info = []
                    for i in sample_indices:
                        orig_laser_x = laser_x[i]
                        orig_laser_y = laser_y[i]
                        w_x = world_x[i]
                        w_y = world_y[i]
                        sample_info.append(f"激光坐标({orig_laser_x:.2f},{orig_laser_y:.2f})→世界坐标({w_x:.2f},{w_y:.2f})")
                    self.get_logger().info(f'【坐标转换样本】: {" | ".join(sample_info)}')
                
            # 检查转换后的坐标是否有异常值
            if np.any(np.isnan(world_x)) or np.any(np.isnan(world_y)):
                if should_log_details:
                    self.get_logger().warn('世界坐标包含NaN值，进行过滤')
                valid_mask = ~np.isnan(world_x) & ~np.isnan(world_y)
                world_x = world_x[valid_mask]
                world_y = world_y[valid_mask]
            
            # 检查范围是否合理 - 使用numpy向量化操作
            valid_mask = (np.abs(world_x) < 100.0) & (np.abs(world_y) < 100.0)
            if not np.all(valid_mask):
                if should_log_details:
                    self.get_logger().warn(f'检测到{np.sum(~valid_mask)}个超出合理范围的点，将被过滤')
                world_x = world_x[valid_mask]
                world_y = world_y[valid_mask]
            
            # 更新占用栅格地图 - 使用高效的批量处理
            if len(world_x) > 0:
                world_points = np.column_stack((world_x, world_y))
                
                # 添加测试：检查是否有y值大于26的点
                high_y_mask = world_y > 26.0
                high_y_count = np.sum(high_y_mask)
                if high_y_count > 0:
                    self.get_logger().info(f'检测到{high_y_count}个y值>26的点')
                    # 打印一些高y值点的示例
                    high_y_indices = np.where(high_y_mask)[0][:5]  # 最多取5个示例
                    for i in high_y_indices:
                        wx, wy = world_x[i], world_y[i]
                        grid_x, grid_y = self.world_to_grid(wx, wy)
                        self.get_logger().info(f'高y值点: 世界({wx:.2f},{wy:.2f}) -> 栅格({grid_x},{grid_y})')
                
                # 使用优化后的update_map方法处理批量点，添加时间戳
                self.update_map(world_points, current_time)
                # 添加调试信息
                if should_log_details:
                    self.get_logger().info(f'更新地图: 添加了{len(world_points)}个点')
                    # 添加实时位姿与障碍物关系调试信息
                    if len(world_points) > 10:
                        self.get_logger().info(f'【实时位姿】x={robot_x:.3f}, y={robot_y:.3f}, theta={robot_theta:.3f}rad, 用于计算障碍物与机器人相对位置')
                        # 计算最近和最远的激光点
                        distances = np.sqrt(np.square(world_x - robot_x) + np.square(world_y - robot_y))
                        min_dist = np.min(distances)
                        max_dist = np.max(distances)
                        self.get_logger().info(f'【激光点距离】最近={min_dist:.2f}m, 最远={max_dist:.2f}m')
                        
                        # 找出最近的几个点，用于示例相对坐标转换
                        if len(distances) > 5:
                            closest_indices = np.argsort(distances)[:3]  # 取3个最近点作为示例
                            self.get_logger().info("【坐标转换示例】世界坐标→相对坐标→栅格坐标:")
                            for idx in closest_indices:
                                # 世界坐标(绝对坐标)
                                wx, wy = world_x[idx], world_y[idx]
                                # 相对于机器人的坐标
                                rx, ry = wx - robot_x, wy - robot_y
                                # 栅格坐标
                                gx, gy = self.world_to_grid(wx, wy)
                                # 输出三种坐标
                                self.get_logger().info(f"  点{idx}: 世界坐标({wx:.2f},{wy:.2f}) → 相对坐标({rx:.2f},{ry:.2f}) → 栅格坐标({gx},{gy})")
            elif should_log_details:
                self.get_logger().warn('所有点均被过滤，无点添加到地图')
            
            # 保存扫描时的轨迹副本，确保与当前扫描数据同步
            with self.traj_lock:
                self.last_scan_trajectory = self.robot_trajectory.copy()
                self.new_trajectory_since_last_map = True  # 激光扫描更新也需要更新地图
        
        except Exception as e:
            # 捕获并记录所有异常，防止节点崩溃
            self.get_logger().error(f'处理激光数据时发生错误: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def reset_map(self):
        """重置地图，清除所有障碍物"""
        try:
            self.get_logger().info(f'重置地图，清除旧障碍物数据')
            
            # 重置地图和计数器，但保留轨迹数据
            self.grid_map = np.zeros((self.size_x, self.size_y), dtype=np.int8)
            self.occupancy_count = {}
            self.obstacle_timestamps = {}
            
            # 不重置地图边界，以保持显示范围稳定
        except Exception as e:
            self.get_logger().error(f'重置地图时出错: {str(e)}')
    
    def update_map(self, points, current_time=None):
        """更新占用栅格地图"""
        try:
            if len(points) == 0:
                return
                
            points_added = 0
            
            # 如果没有提供时间戳，使用当前时间
            if current_time is None:
                current_time = time.time()
            
            # 获取当前机器人位置 - 使用最新位姿
            robot_x = self.latest_pose['x'] if self.latest_pose else 0.0
            robot_y = self.latest_pose['y'] if self.latest_pose else 0.0
            robot_theta = self.latest_pose['theta'] if self.latest_pose else 0.0
            
            # 记录机器人当前位置作为调试信息
            if hasattr(self, '_map_stats_count') and self._map_stats_count % 50 == 0:
                self.get_logger().info(f'【障碍物计算】机器人位置: x={robot_x:.2f}, y={robot_y:.2f}, theta={robot_theta:.2f}')
            
            # 计算点与机器人的距离 - 用于优先处理近距离障碍物
            def calculate_distance_to_robot(point):
                """计算点到机器人的距离"""
                x, y = point
                dx = x - robot_x
                dy = y - robot_y
                return np.sqrt(dx*dx + dy*dy)
            
            # 对于批量点，优化处理方式
            if isinstance(points, np.ndarray) and points.ndim == 2 and points.shape[1] == 2 and len(points) > 10:
                # 计算所有点到机器人的距离
                if len(points) > 100:  # 对于大量点，计算距离并记录
                    distances = np.zeros(len(points))
                    for i in range(len(points)):
                        distances[i] = calculate_distance_to_robot(points[i])
                    
                    # 找出最近的几个点，记录到日志
                    closest_indices = np.argsort(distances)[:5]
                    closest_points = points[closest_indices]
                    closest_distances = distances[closest_indices]
                    
                    # 记录最近的障碍物
                    self.get_logger().info(f'【最近障碍物】距离机器人: {", ".join([f"({p[0]:.2f},{p[1]:.2f})={d:.2f}m" for p,d in zip(closest_points, closest_distances)])}')
                    
                    # 按照用户要求提供三种坐标表示
                    # 1. 小车的当前绝对坐标
                    self.get_logger().info(f'1.小车的当前绝对坐标: ({robot_x:.2f}m,{robot_y:.2f}m), 朝向={robot_theta:.2f}rad')
                    
                    # 2. 障碍物相对于小车的坐标
                    rel_coords = []
                    for p in closest_points:
                        # 计算障碍物相对于机器人的坐标
                        rel_x = p[0] - robot_x
                        rel_y = p[1] - robot_y
                        rel_coords.append((rel_x, rel_y))
                    
                    self.get_logger().info(f'2.障碍物相对于小车的坐标: {", ".join([f"({rx:.2f}m,{ry:.2f}m)" for rx,ry in rel_coords])}')
                    
                    # 3. 障碍物的绝对坐标
                    self.get_logger().info(f'3.障碍物的绝对坐标: {", ".join([f"({p[0]:.2f}m,{p[1]:.2f}m)" for p in closest_points])}')
                
                # 更新地图边界 - 确保包含所有点，即使它们在当前地图范围之外
                min_x = np.min(points[:, 0])
                max_x = np.max(points[:, 0])
                min_y = np.min(points[:, 1])
                max_y = np.max(points[:, 1])
                
                self.min_x = min(self.min_x, min_x)
                self.max_x = max(self.max_x, max_x)
                self.min_y = min(self.min_y, min_y)
                self.max_y = max(self.max_y, max_y)
                
                # 每100次更新记录一次地图边界的更新
                if not hasattr(self, '_boundary_update_count'):
                    self._boundary_update_count = 0
                self._boundary_update_count += 1
                if self._boundary_update_count % 100 == 0:
                    self.get_logger().info(f'【地图边界】x=[{self.min_x:.2f}, {self.max_x:.2f}], y=[{self.min_y:.2f}, {self.max_y:.2f}]')
                
                # 检查是否有点超出安全处理范围
                out_of_range_points = np.any((points[:, 0] < -180) | (points[:, 0] > 180) | 
                                          (points[:, 1] < -180) | (points[:, 1] > 180))
                if out_of_range_points:
                    self.get_logger().warn('存在超出±180米范围的点，这些点可能不会正确显示在地图上')
                
                # 批量转换为栅格坐标 - 在转换前不裁剪，允许更大范围的世界坐标
                grid_coords = []
                for i in range(len(points)):
                    try:
                        # 使用优化后的world_to_grid函数
                        gx, gy = self.world_to_grid(points[i, 0], points[i, 1])
                        grid_coords.append((gx, gy))
                    except Exception as e:
                        pass  # 忽略无法转换的点
                
                # 计算每个坐标出现的次数
                coord_counts = Counter(grid_coords)
                
                # 更新地图
                for (gx, gy), count in coord_counts.items():
                    key = (gx, gy)
                    # 累积计数
                    if key in self.occupancy_count:
                        self.occupancy_count[key] += count
                    else:
                        self.occupancy_count[key] = count
                    
                    # 如果累积次数达到阈值，标记为障碍物
                    if self.occupancy_count[key] >= self.threshold:
                        if 0 <= gx < self.size_x and 0 <= gy < self.size_y:  # 确保在有效范围内
                            self.grid_map[gx, gy] = 1
                            self.obstacle_timestamps[key] = current_time
                            points_added += 1
            else:
                # 传统的点一个个处理
                for x, y in points:
                    # 更新地图边界
                    self.min_x = min(self.min_x, x)
                    self.max_x = max(self.max_x, x)
                    self.min_y = min(self.min_y, y)
                    self.max_y = max(self.max_y, y)
                    
                    # 转换为栅格坐标
                    grid_x, grid_y = self.world_to_grid(x, y)
                    
                    # 检查是否在地图范围内
                    if 0 <= grid_x < self.size_x and 0 <= grid_y < self.size_y:
                        # 累积计数
                        key = (grid_x, grid_y)
                        self.occupancy_count[key] = self.occupancy_count.get(key, 0) + 1
                        
                        # 更新栅格地图
                        if self.occupancy_count[key] > self.threshold:
                            if self.grid_map[grid_x, grid_y] == 0:  # 只有当格子状态改变时才计数
                                points_added += 1
                            self.grid_map[grid_x, grid_y] = 1
                            # 更新障碍物时间戳
                            self.obstacle_timestamps[key] = current_time
            
            # 降低日志频率，提高处理速度
            if points_added > 50:  # 只有当添加了大量新点时才记录
                self.get_logger().info(f'添加了{points_added}个新障碍物点到地图')
            
            # 定期打印地图统计信息，从每100个改为每500个障碍物单元格打印一次
            if not hasattr(self, '_map_stats_count'):
                self._map_stats_count = 0
            
            total_obstacle_cells = np.sum(self.grid_map == 1)
            
            # 如果障碍物数量与上次统计相比增加了500个以上，或者是初次统计
            if self._map_stats_count == 0 or total_obstacle_cells - self._map_stats_count >= 500:
                self.get_logger().info(f'地图统计: 障碍物单元格={total_obstacle_cells}, 地图边界=[{self.min_x:.1f}, {self.max_x:.1f}]x[{self.min_y:.1f}, {self.max_y:.1f}]')
                self._map_stats_count = total_obstacle_cells
        except Exception as e:
            self.get_logger().error(f'更新地图时出错: {str(e)}')
            # 失败时降级到简单处理
            try:
                if isinstance(points, np.ndarray):
                    for i in range(len(points)):
                        x, y = points[i]
                        # 简单更新
                        grid_x, grid_y = self.world_to_grid(x, y)
                        if 0 <= grid_x < self.size_x and 0 <= grid_y < self.size_y:
                            self.grid_map[grid_x, grid_y] = 1
            except:
                self.get_logger().error('回退处理也失败')
    
    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角（滚转、俯仰、偏航）"""
        # 计算方向余弦矩阵元素
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # 俯仰角（pitch）
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # 使用copysign从sinp获取符号
        else:
            pitch = np.arcsin(sinp)
        
        # 偏航角（yaw）
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)
    
    def world_to_grid(self, x, y):
        """世界坐标系转栅格坐标系
        
        参数:
            x: 世界坐标系下的X坐标
            y: 世界坐标系下的Y坐标
            
        返回:
            (grid_x, grid_y): 栅格坐标系下的整数坐标
        """
        try:
            # 检查输入坐标是否在合理范围内 - 扩大允许范围
            if not -200 < x < 200 or not -200 < y < 200:
                self.get_logger().warn(f'世界坐标超出扩展范围: x={x}, y={y}')
            
            # 将世界坐标转换为栅格坐标
            # origin_x和origin_y表示地图原点(0,0)在栅格中的位置（通常是地图中心）
            # resolution是每个栅格代表的实际距离（单位：米）
            grid_x = int(self.origin_x + x / self.resolution)
            grid_y = int(self.origin_y + y / self.resolution)
            
            # 调试输出 - 每500次转换输出一次以避免日志过多
            if not hasattr(self, '_world_to_grid_count'):
                self._world_to_grid_count = 0
            self._world_to_grid_count += 1
            
            if self._world_to_grid_count % 500 == 0:
                self.get_logger().info(f'【坐标转换】世界坐标(x={x:.2f},y={y:.2f}) --> 栅格坐标(grid_x={grid_x},grid_y={grid_y})')
                self.get_logger().info(f'【参数确认】原点(origin_x={self.origin_x},origin_y={self.origin_y}), 分辨率={self.resolution}米/格')
            
            # 扩大栅格范围处理 - 动态处理超出范围的坐标
            # 如果坐标超出范围，记录日志但仍返回实际计算值，由调用者决定如何处理
            if grid_x < 0 or grid_x >= self.size_x or grid_y < 0 or grid_y >= self.size_y:
                if self._world_to_grid_count % 100 == 0:  # 减少日志频率
                    self.get_logger().warn(f'栅格坐标超出地图范围: (grid_x={grid_x},grid_y={grid_y}), 来自世界坐标(x={x:.2f},y={y:.2f})')
                
                # 仍然需要限制在合理范围内以避免数组访问错误
                grid_x = max(0, min(grid_x, self.size_x - 1))
                grid_y = max(0, min(grid_y, self.size_y - 1))
            
            return grid_x, grid_y
        except Exception as e:
            self.get_logger().error(f'世界坐标转栅格坐标出错: {str(e)}')
            # 返回地图中心作为默认值
            return self.origin_x, self.origin_y
    
    def publish_map(self):
        """发布占用栅格地图"""
        try:
            if self.min_x == float('inf'):
                # 还没有接收到任何数据
                return
            
            # 创建并发布地图消息
            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'map'
            
            map_msg.info.resolution = self.resolution
            map_msg.info.width = self.size_x
            map_msg.info.height = self.size_y
            map_msg.info.origin.position.x = -self.origin_x * self.resolution
            map_msg.info.origin.position.y = -self.origin_y * self.resolution
            map_msg.info.origin.position.z = 0.0
            map_msg.info.origin.orientation.w = 1.0
            
            # 确保grid_map的类型正确
            if self.grid_map.dtype != np.int8:
                self.get_logger().warn(f'grid_map类型错误: {self.grid_map.dtype}，转换为int8')
                self.grid_map = self.grid_map.astype(np.int8)
            
            # 使用向量化操作更高效地转换为占用率数组
            # 创建一个全为0的数组
            occupancy_values = np.zeros(self.size_x * self.size_y, dtype=np.int8)
            
            # 更高效地将二进制地图转换为占用率
            # 先创建与grid_map形状相同的视图
            occupancy_2d = occupancy_values.reshape(self.size_y, self.size_x).T
            
            # 用向量化操作设置障碍物单元格为100
            obstacle_indices = np.where(self.grid_map == 1)
            if len(obstacle_indices[0]) > 0:
                occupancy_2d[obstacle_indices] = 100
            
            # 将二维数组转换回一维
            occupancy_values = occupancy_2d.T.reshape(-1)
            
            # 约束值范围
            occupancy_values = np.clip(occupancy_values, -1, 100).astype(np.int8)
            
            # 转换为list前，确保所有值在有效范围内
            if np.any(occupancy_values > 127) or np.any(occupancy_values < -128):
                self.get_logger().error(f'数据范围超出[-128, 127]: min={np.min(occupancy_values)}, max={np.max(occupancy_values)}')
                occupancy_values = np.clip(occupancy_values, -128, 127).astype(np.int8)
            
            # 发布地图
            try:
                # 转换为list可能很耗时，所以只在必要时转换
                occupancy_list = occupancy_values.tolist()
                map_msg.data = occupancy_list
                self.map_publisher.publish(map_msg)
            except Exception as e:
                self.get_logger().error(f'发布地图时出错: {str(e)}')
                # 保存导致错误的数据
                try:
                    np.save('/tmp/error_grid_map.npy', self.grid_map)
                    np.save('/tmp/error_occupancy_values.npy', occupancy_values)
                    self.get_logger().info('已保存错误数据到/tmp目录')
                except Exception as save_err:
                    self.get_logger().error(f'保存错误数据失败: {str(save_err)}')
        except Exception as e:
            self.get_logger().error(f'准备地图数据时出错: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def save_map(self):
        """保存占用栅格地图为图像文件"""
        try:
            # 创建彩色地图图像
            rgb_map = np.zeros((self.size_y, self.size_x, 3), dtype=np.uint8)
            
            # 白色背景
            rgb_map.fill(255)
            
            # 红色表示障碍物
            obstacle_indices = np.where(self.grid_map == 1)
            if len(obstacle_indices[0]) > 0:
                # 添加调试信息：输出栅格和世界坐标中的障碍物y值范围
                min_grid_y = np.min(obstacle_indices[1])
                max_grid_y = np.max(obstacle_indices[1])
                
                # 计算对应的世界坐标y值
                min_world_y, max_world_y = self.grid_to_world(0, min_grid_y)[1], self.grid_to_world(0, max_grid_y)[1]
                
                self.get_logger().info(f'障碍物栅格坐标y范围: [{min_grid_y}, {max_grid_y}]')
                self.get_logger().info(f'障碍物世界坐标y范围: [{min_world_y:.2f}, {max_world_y:.2f}]')
                
                # 检查是否有大于26的y值障碍物
                high_y_obs = [(x, y) for x, y in zip(obstacle_indices[0], obstacle_indices[1]) if self.grid_to_world(x, y)[1] > 26]
                if high_y_obs:
                    self.get_logger().info(f'发现y>26的障碍物: {len(high_y_obs)}个点')
                    # 取样几个高y值障碍物点用于调试
                    sample_high_y = high_y_obs[:5] if len(high_y_obs) >= 5 else high_y_obs
                    for grid_x, grid_y in sample_high_y:
                        world_x, world_y = self.grid_to_world(grid_x, grid_y)
                        self.get_logger().info(f'高y值障碍物: 栅格({grid_x},{grid_y}) -> 世界({world_x:.2f},{world_y:.2f})')
                    
                # 确保所有障碍物都被标记为红色（RGB: 255, 0, 0）
                rgb_map[obstacle_indices[1], obstacle_indices[0]] = [255, 0, 0]  # RGB: 红色
            
            # 添加网格线
            grid_step = max(1, int(10 / self.resolution))  # 每10米一条线
            for i in range(0, self.size_x, grid_step):
                rgb_map[:, i] = [0, 0, 255]  # 蓝色垂直线
            for i in range(0, self.size_y, grid_step):
                rgb_map[i, :] = [0, 0, 255]  # 蓝色水平线
            
            # 安全获取轨迹副本
            current_trajectory = []
            with self.traj_lock:
                current_trajectory = self.robot_trajectory.copy()
            
            # 绘制机器人轨迹（绿色）
            if current_trajectory:
                self.get_logger().info(f'绘制轨迹: {len(current_trajectory)}个点')
                
                # 计算轨迹的边界
                traj_x = [p[0] for p in current_trajectory]
                traj_y = [p[1] for p in current_trajectory]
                traj_min_x = min(traj_x)
                traj_max_x = max(traj_x)
                traj_min_y = min(traj_y)
                traj_max_y = max(traj_y)
                self.get_logger().info(f'轨迹边界: x=[{traj_min_x:.2f}, {traj_max_x:.2f}], y=[{traj_min_y:.2f}, {traj_max_y:.2f}]')
                
                # 绘制所有轨迹点连线
                prev_x, prev_y = None, None
                for x, y in current_trajectory:
                    try:
                        grid_x, grid_y = self.world_to_grid(x, y)
                        
                        # 绘制轨迹点
                        if 0 <= grid_x < self.size_x and 0 <= grid_y < self.size_y:
                            # 绘制3x3的点，确保可见性
                            for dx in [-1, 0, 1]:
                                for dy in [-1, 0, 1]:
                                    gx, gy = grid_x + dx, grid_y + dy
                                    if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                                        rgb_map[gy, gx] = [0, 255, 0]  # 绿色点
                        
                        # 绘制连接线
                        if prev_x is not None and prev_y is not None:
                            # 使用Bresenham算法绘制线段
                            try:
                                line_points = self.bresenham_line(prev_x, prev_y, grid_x, grid_y)
                                for lx, ly in line_points:
                                    if 0 <= lx < self.size_x and 0 <= ly < self.size_y:
                                        rgb_map[ly, lx] = [0, 200, 0]  # 稍暗的绿色线
                            except Exception as line_err:
                                self.get_logger().warn(f'绘制轨迹线时出错，跳过此线段: {str(line_err)}')
                        
                        prev_x, prev_y = grid_x, grid_y
                    except Exception as point_err:
                        self.get_logger().warn(f'处理轨迹点({x}, {y})时出错: {str(point_err)}')
                
                # 特别标记起点和终点
                if len(current_trajectory) >= 2:
                    try:
                        # 起点(蓝色)
                        start_x, start_y = self.world_to_grid(current_trajectory[0][0], current_trajectory[0][1])
                        for dx in range(-4, 5):
                            for dy in range(-4, 5):
                                gx, gy = start_x + dx, start_y + dy
                                if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                                    # 绘制圆形起点标记
                                    if dx*dx + dy*dy <= 16:
                                        rgb_map[gy, gx] = [0, 0, 255]  # 起点蓝色
                        
                        # 终点(红色)
                        end_x, end_y = self.world_to_grid(current_trajectory[-1][0], current_trajectory[-1][1])
                        for dx in range(-4, 5):
                            for dy in range(-4, 5):
                                gx, gy = end_x + dx, end_y + dy
                                if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                                    # 绘制X形终点标记
                                    if abs(dx) == abs(dy):
                                        rgb_map[gy, gx] = [255, 0, 0]  # 终点红色
                    except Exception as mark_err:
                        self.get_logger().warn(f'标记轨迹起终点时出错: {str(mark_err)}')
            else:
                self.get_logger().warn('没有轨迹点可绘制')
            
            # 保存图像
            fig = plt.figure(figsize=(12, 12))
            
            try:
                # 显示完整地图
                display_map = rgb_map
                
                # 计算完整地图的世界坐标范围
                extent = [
                    -self.origin_x * self.resolution,
                    (self.size_x - self.origin_x) * self.resolution,
                    -self.origin_y * self.resolution,
                    (self.size_y - self.origin_y) * self.resolution
                ]
                
                # 添加调试日志，记录地图边界信息
                obstacle_indices = np.where(self.grid_map == 1)
                self.get_logger().info(f'保存最终地图: extent={extent}, 障碍物数量={len(obstacle_indices[0])}')
                self.get_logger().info(f'地图统计: 地图大小={self.size_x}x{self.size_y}, 原点=({self.origin_x}, {self.origin_y}), 分辨率={self.resolution}')
                self.get_logger().info(f'障碍物边界: x=[{self.min_x:.2f}, {self.max_x:.2f}], y=[{self.min_y:.2f}, {self.max_y:.2f}]')
                
                # 绘制地图
                plt.imshow(display_map, origin='lower', extent=extent)
                
                # 动态计算显示范围，根据轨迹和障碍物数据
                map_bounds = calculate_map_bounds(self.min_x, self.max_x, self.min_y, self.max_y)
                
                # 如果轨迹数据有效，也考虑轨迹的边界
                if current_trajectory:
                    traj_x = [p[0] for p in current_trajectory]
                    traj_y = [p[1] for p in current_trajectory]
                    traj_min_x, traj_max_x = min(traj_x), max(traj_x)
                    traj_min_y, traj_max_y = min(traj_y), max(traj_y)
                    
                    # 取二者边界的并集
                    map_bounds = calculate_map_bounds(
                        min(self.min_x, traj_min_x),
                        max(self.max_x, traj_max_x),
                        min(self.min_y, traj_min_y),
                        max(self.max_y, traj_max_y)
                    )
                
                # 设置动态计算的显示范围
                plt.xlim(map_bounds[0], map_bounds[1])
                
                # 关键修改：确保y轴范围足够大以显示所有障碍物
                # 检查是否有高y值障碍物，如果有，确保ylim上限足够大
                if max_world_y > map_bounds[3]:
                    self.get_logger().warn(f'发现高y值障碍物({max_world_y:.2f})超出了原地图范围({map_bounds[3]:.2f})，自动扩展显示范围')
                    plt.ylim(map_bounds[2], max(map_bounds[3], max_world_y + 5))
                else:
                    plt.ylim(map_bounds[2], map_bounds[3])
                
                # 输出最终使用的显示范围
                final_ylim = plt.gca().get_ylim()
                self.get_logger().info(f'最终使用的y轴显示范围: [{final_ylim[0]:.2f}, {final_ylim[1]:.2f}]')
                
                # 根据地图边界计算适合的网格线间隔
                x_range = map_bounds[1] - map_bounds[0]
                y_range = map_bounds[3] - map_bounds[2]
                
                # 设置合适的网格线间隔，10米或者范围的1/5，取较小值
                grid_step_m = min(10, max(x_range, y_range) / 5)
                grid_step_m = max(2, round(grid_step_m))  # 至少2米，并取整
                
                # 设置网格线
                plt.grid(True, color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
                plt.xticks(np.arange(math.floor(map_bounds[0]), math.ceil(map_bounds[1])+1, grid_step_m))
                plt.yticks(np.arange(math.floor(map_bounds[2]), math.ceil(map_bounds[3])+1, grid_step_m))
                
                plt.xlabel('X (m)')
                plt.ylabel('Y (m)')
                plt.title('占用栅格图 (白色=空闲, 红色=障碍物)', fontsize=14)
                
                # 保存并关闭图像
                try:
                    plt.savefig(self.map_save_path, dpi=300, bbox_inches='tight')
                    self.get_logger().info(f'地图已保存到: {self.map_save_path}, 显示范围: x=[{map_bounds[0]:.1f}, {map_bounds[1]:.1f}], y=[{map_bounds[2]:.1f}, {map_bounds[3]:.1f}]')
                except Exception as save_err:
                    self.get_logger().error(f'保存地图文件失败: {str(save_err)}')
                    # 尝试保存到备用位置
                    try:
                        backup_path = '/tmp/emergency_map.png'
                        plt.savefig(backup_path, dpi=200)
                        self.get_logger().info(f'已保存应急地图到: {backup_path}')
                    except:
                        self.get_logger().error('无法保存地图到任何位置')
            except Exception as plt_err:
                self.get_logger().error(f'绘制地图时出错: {str(plt_err)}')
            finally:
                # 确保关闭图形以释放资源
                plt.close(fig)
            
            # 如果轨迹点数量适中，额外保存一个只包含轨迹的图
            if current_trajectory:
                try:
                    plt.figure(figsize=(12, 12))
                    traj_x = [p[0] for p in current_trajectory]
                    traj_y = [p[1] for p in current_trajectory]
                    plt.plot(traj_x, traj_y, 'g-', linewidth=2, markersize=2)
                    
                    # 标记起点和终点
                    plt.plot(traj_x[0], traj_y[0], 'bo', markersize=10, label='起点')
                    plt.plot(traj_x[-1], traj_y[-1], 'ro', markersize=10, label='终点')
                    
                    # 使用动态计算的边界，不再使用固定值
                    map_bounds = calculate_map_bounds(
                        min(traj_x), max(traj_x),
                        min(traj_y), max(traj_y),
                        padding=5.0  # 为轨迹图提供更大的边距
                    )
                    plt.xlim(map_bounds[0], map_bounds[1])
                    plt.ylim(map_bounds[2], map_bounds[3])
                    
                    # 添加网格线，使用动态计算的间隔
                    x_range = map_bounds[1] - map_bounds[0]
                    y_range = map_bounds[3] - map_bounds[2]
                    grid_step_m = min(10, max(x_range, y_range) / 5)
                    grid_step_m = max(2, round(grid_step_m))  # 至少2米，并取整
                    
                    plt.grid(True, color='gray', linestyle='-', linewidth=0.5, alpha=0.5)
                    plt.xticks(np.arange(math.floor(map_bounds[0]), math.ceil(map_bounds[1])+1, grid_step_m))
                    plt.yticks(np.arange(math.floor(map_bounds[2]), math.ceil(map_bounds[3])+1, grid_step_m))
                    
                    plt.xlabel('X (m)', fontsize=12)
                    plt.ylabel('Y (m)', fontsize=12)
                    plt.title('机器人轨迹', fontsize=14)
                    plt.legend(loc='upper right')
                    
                    traj_path = 'robot_trajectory.png'
                    plt.savefig(traj_path, dpi=300, bbox_inches='tight')
                    self.get_logger().info(f'轨迹图已保存到: {traj_path}，显示范围: x=[{map_bounds[0]:.1f}, {map_bounds[1]:.1f}], y=[{map_bounds[2]:.1f}, {map_bounds[3]:.1f}]')
                except Exception as traj_err:
                    self.get_logger().error(f'绘制轨迹图时出错: {str(traj_err)}')
                finally:
                    plt.close()
        except Exception as e:
            self.get_logger().error(f'保存地图过程中出错: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def bresenham_line(self, x0, y0, x1, y1):
        """使用Bresenham算法生成两点间的线段像素坐标"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return points

    def update_progress(self):
        """更新和显示处理进度，并根据需要保存中间结果"""
        try:
            if self.min_x == float('inf'):  # 还没有数据
                return
                
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            
            # 检查轨迹是否有更新
            trajectory_changed = False
            with self.traj_lock:
                trajectory_changed = self.new_trajectory_since_last_map
            
            # 添加进度日志控制 - 每5次处理才输出一次日志
            if not hasattr(self, '_progress_log_counter'):
                self._progress_log_counter = 0
            self._progress_log_counter += 1
            
            # 只有当计数为5的倍数，或轨迹有更新时才输出进度日志
            if self._progress_log_counter % 5 == 0 or trajectory_changed:
                self.get_logger().info(f'【进度报告】运行时间: {elapsed_time:.1f}秒, 处理扫描: {self.processed_scans}帧, 轨迹点: {len(self.robot_trajectory)}个, 障碍物格子: {np.sum(self.grid_map == 1)}个' + 
                                    (", 轨迹已更新" if trajectory_changed else ""))
            
            # 检查是否需要保存中间结果 - 增加条件：如果轨迹有重大变化也保存
            if current_time - self.last_saved_time >= self.save_interval or trajectory_changed:
                self.save_intermediate_map()
                self.last_saved_time = current_time
                # 重置标志
                with self.traj_lock:
                    self.new_trajectory_since_last_map = False
        except Exception as e:
            self.get_logger().error(f'更新进度信息时出错: {str(e)}')
    
    def save_intermediate_map(self):
        """保存中间地图结果"""
        try:
            self.checkpoint_counter += 1
            checkpoint_path = f'map_checkpoint_{self.checkpoint_counter}.png'
            
            # 创建彩色地图图像
            rgb_map = np.zeros((self.size_y, self.size_x, 3), dtype=np.uint8)
            
            # 白色背景
            rgb_map.fill(255)
            
            # 红色表示障碍物
            obstacle_indices = np.where(self.grid_map == 1)
            if len(obstacle_indices[0]) > 0:
                rgb_map[obstacle_indices[1], obstacle_indices[0]] = [255, 0, 0]  # RGB: 红色
                self.get_logger().info(f'共绘制障碍物点 {len(obstacle_indices[0])}个')
                
                # 记录障碍物点的世界坐标范围，帮助调试
                obstacle_world_coords = []
                for i in range(len(obstacle_indices[0])):
                    grid_x, grid_y = obstacle_indices[0][i], obstacle_indices[1][i]
                    world_x = (grid_x - self.origin_x) * self.resolution
                    world_y = (grid_y - self.origin_y) * self.resolution
                    obstacle_world_coords.append((world_x, world_y))
                
                if obstacle_world_coords:
                    min_obs_x = min(p[0] for p in obstacle_world_coords)
                    max_obs_x = max(p[0] for p in obstacle_world_coords)
                    min_obs_y = min(p[1] for p in obstacle_world_coords)
                    max_obs_y = max(p[1] for p in obstacle_world_coords)
                    self.get_logger().info(f'障碍物世界坐标范围: X={min_obs_x:.2f}到{max_obs_x:.2f}, Y={min_obs_y:.2f}到{max_obs_y:.2f}')
            
            # 获取当前轨迹的安全副本
            current_trajectory = []
            with self.traj_lock:
                current_trajectory = self.robot_trajectory.copy()
            
            # 绘制机器人轨迹（绿色）- 使用当前同步的轨迹
            if current_trajectory:
                # 简化日志，仅打印轨迹点数
                self.get_logger().info(f'中间地图 #{self.checkpoint_counter}: 绘制轨迹 {len(current_trajectory)}个点, 障碍物 {len(obstacle_indices[0])}个')
                prev_x, prev_y = None, None
                for x, y in current_trajectory:
                    try:
                        grid_x, grid_y = self.world_to_grid(x, y)
                        
                        # 绘制轨迹点
                        if 0 <= grid_x < self.size_x and 0 <= grid_y < self.size_y:
                            # 绘制3x3的点，确保可见性
                            for dx in [-1, 0, 1]:
                                for dy in [-1, 0, 1]:
                                    gx, gy = grid_x + dx, grid_y + dy
                                    if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                                        rgb_map[gy, gx] = [0, 255, 0]  # 绿色点
                        
                        # 绘制连接线
                        if prev_x is not None and prev_y is not None:
                            # 使用Bresenham算法绘制线段
                            try:
                                line_points = self.bresenham_line(prev_x, prev_y, grid_x, grid_y)
                                for lx, ly in line_points:
                                    if 0 <= lx < self.size_x and 0 <= ly < self.size_y:
                                        rgb_map[ly, lx] = [0, 200, 0]  # 稍暗的绿色线
                            except:
                                pass
                        
                        prev_x, prev_y = grid_x, grid_y
                    except:
                        pass
                
                # 特别标记起点和终点
                if len(current_trajectory) >= 2:
                    try:
                        # 起点(蓝色)
                        start_x, start_y = self.world_to_grid(current_trajectory[0][0], current_trajectory[0][1])
                        for dx in range(-4, 5):
                            for dy in range(-4, 5):
                                gx, gy = start_x + dx, start_y + dy
                                if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                                    # 绘制圆形起点标记
                                    if dx*dx + dy*dy <= 16:
                                        rgb_map[gy, gx] = [0, 0, 255]  # 起点蓝色
                        
                        # 终点(红色)
                        end_x, end_y = self.world_to_grid(current_trajectory[-1][0], current_trajectory[-1][1])
                        for dx in range(-4, 5):
                            for dy in range(-4, 5):
                                gx, gy = end_x + dx, end_y + dy
                                if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                                    # 绘制X形终点标记
                                    if abs(dx) == abs(dy):
                                        rgb_map[gy, gx] = [255, 0, 0]  # 终点红色
                    except Exception as mark_err:
                        self.get_logger().warn(f'标记轨迹起终点时出错: {str(mark_err)}')
            else:
                self.get_logger().warn('没有轨迹点可绘制')
            
            # 保存简化版的地图
            fig = plt.figure(figsize=(15, 15))  # 增加图像尺寸
            try:
                # 计算完整地图的世界坐标范围
                extent = [
                    -self.origin_x * self.resolution,
                    (self.size_x - self.origin_x) * self.resolution,
                    -self.origin_y * self.resolution,
                    (self.size_y - self.origin_y) * self.resolution
                ]
                
                # 减少输出的调试信息，只保留关键信息
                # 动态计算显示范围，根据当前数据 - 确保使用全部数据范围
                map_bounds = calculate_map_bounds(self.min_x, self.max_x, self.min_y, self.max_y, padding=15.0)  # 增加边距
                
                # 如果轨迹数据有效，也考虑轨迹的边界，并确保范围充分大以显示所有障碍物
                if current_trajectory:
                    traj_x = [p[0] for p in current_trajectory]
                    traj_y = [p[1] for p in current_trajectory]
                    traj_min_x, traj_max_x = min(traj_x), max(traj_x)
                    traj_min_y, traj_max_y = min(traj_y), max(traj_y)
                    
                    # 取二者边界的并集，并确保提供足够大的显示范围
                    map_bounds = calculate_map_bounds(
                        min(self.min_x, traj_min_x) - 20.0,  # 左侧额外留出空间
                        max(self.max_x, traj_max_x) + 20.0,  # 右侧额外留出空间
                        min(self.min_y, traj_min_y) - 20.0,  # 下方额外留出空间
                        max(self.max_y, traj_max_y) + 30.0   # 上方额外留出空间
                    )
                
                # 输出当前地图边界以便调试
                self.get_logger().info(f'地图显示边界: X={map_bounds[0]:.1f}到{map_bounds[1]:.1f}, Y={map_bounds[2]:.1f}到{map_bounds[3]:.1f}')
                
                # 只保留一次imshow调用
                plt.imshow(rgb_map, origin='lower', extent=extent)
                
                # 设置动态计算的显示范围
                plt.xlim(map_bounds[0], map_bounds[1])
                plt.ylim(map_bounds[2], map_bounds[3])
                
                # 计算合适的网格线间隔
                x_range = map_bounds[1] - map_bounds[0]
                y_range = map_bounds[3] - map_bounds[2]
                grid_step_m = min(10, max(x_range, y_range) / 8)  # 增加网格线数量
                grid_step_m = max(2, round(grid_step_m))  # 至少2米，并取整
                
                # 设置网格线
                plt.grid(True, color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
                plt.xticks(np.arange(math.floor(map_bounds[0]), math.ceil(map_bounds[1])+1, grid_step_m))
                plt.yticks(np.arange(math.floor(map_bounds[2]), math.ceil(map_bounds[3])+1, grid_step_m))
                
                # 添加轨迹信息到标题
                title_text = f'中间地图结果 - 检查点 {self.checkpoint_counter} (时间: {(time.time() - self.start_time):.1f}秒)'
                if current_trajectory:
                    title_text += f' - 轨迹点数: {len(current_trajectory)}个'
                plt.title(title_text)
                
                # 添加轨迹图层 - 直接在matplotlib中再画一遍轨迹确保可见性
                if current_trajectory and len(current_trajectory) > 1:
                    traj_x = [p[0] for p in current_trajectory]
                    traj_y = [p[1] for p in current_trajectory]
                    plt.plot(traj_x, traj_y, color='blue', linewidth=2, alpha=0.7)
                    plt.plot(traj_x[0], traj_y[0], 'go', markersize=8)  # 起点
                    plt.plot(traj_x[-1], traj_y[-1], 'ro', markersize=8)  # 终点
                    
                    # 添加坐标轴标签和图例
                    plt.xlabel('X坐标 (米)')
                    plt.ylabel('Y坐标 (米)')
                    plt.legend(['机器人轨迹', '起点', '终点'], loc='upper right')
                
                # 降低DPI进一步加快保存速度
                plt.savefig(checkpoint_path, dpi=100)
                self.get_logger().info(f'已保存中间地图 #{self.checkpoint_counter} 到: {checkpoint_path}')
            except Exception as e:
                self.get_logger().error(f'保存中间地图时出错: {str(e)}')
            finally:
                plt.close(fig)
                
        except Exception as e:
            self.get_logger().error(f'保存中间地图过程中出错: {str(e)}')


def main(args=None):
    try:
        rclpy.init(args=args)
        
        # 创建节点
        node = LmsMapperNode()
        node.get_logger().info('LMS mapper node已启动，等待数据...')
        
        try:
            # 记录开始时间
            start_time = time.time()
            
            # 主动处理一段时间的消息
            timeout = 0
            max_timeout = 600  # 增加最大等待时间到10分钟
            
            while rclpy.ok() and timeout < max_timeout:
                try:
                    # 将超时时间从1.0秒减少到0.1秒，加快处理速度
                    rclpy.spin_once(node, timeout_sec=0.05)  # 进一步减少延迟
                    
                    # 检查是否已接收到数据
                    if len(node.robot_trajectory) > 0:
                        # 减少日志频率，避免过多I/O操作
                        if len(node.robot_trajectory) % 100 == 0:  # 从50改为100
                            node.get_logger().info(f'已接收轨迹点: {len(node.robot_trajectory)}')
                        if timeout == 0:
                            node.get_logger().info('开始接收数据，继续处理...')
                    else:
                        timeout += 1
                        if timeout % 20 == 0:  # 从10秒改为20秒提示一次
                            node.get_logger().warn(f'等待数据中... {timeout}/{max_timeout}秒')
                except KeyboardInterrupt:
                    raise  # 重新抛出键盘中断异常
                except Exception as e:
                    node.get_logger().error(f'处理消息时出错: {str(e)}')
                    # 继续运行，不中断
            
            # 主循环处理
            if timeout < max_timeout:
                node.get_logger().info('进入主处理循环')
                # 设置循环安全标志
                last_error_time = None
                error_count = 0
                
                # 设置处理超时
                processing_timeout = 300  # 默认5分钟
                processing_start_time = time.time()
                
                while rclpy.ok():
                    try:
                        # 检查是否超时
                        current_time = time.time()
                        if current_time - processing_start_time > processing_timeout:
                            node.get_logger().warn(f'处理时间已超过{processing_timeout}秒，尝试保存当前结果并退出...')
                            break
                            
                        # 使用较小的timeout_sec值，加快处理速度
                        rclpy.spin_once(node, timeout_sec=0.01)
                    except KeyboardInterrupt:
                        node.get_logger().info('用户中断，退出...')
                        break
                    except Exception as e:
                        # 错误频率控制，防止日志刷屏
                        current_time = time.time()
                        if last_error_time is None or (current_time - last_error_time) > 5.0:
                            node.get_logger().error(f'主循环处理出错: {str(e)}')
                            last_error_time = current_time
                            error_count = 1
                        else:
                            error_count += 1
                            if error_count % 100 == 0:  # 从50个改为100个错误才记录一次
                                node.get_logger().error(f'持续出错 ({error_count}次)')
                        
                        # 将暂停时间从0.1秒减少到0.01秒，大幅减少等待时间
                        time.sleep(0.01)
            else:
                node.get_logger().error('超时未接收到数据，退出')
        
        except KeyboardInterrupt:
            node.get_logger().info('用户中断，退出')
        except Exception as e:
            node.get_logger().error(f'发生错误: {str(e)}')
            import traceback
            node.get_logger().error(traceback.format_exc())
        finally:
            try:
                # 保存最终地图
                node.get_logger().info('保存地图...')
                
                try:
                    # 保存当前结果，无论处理是否完整
                    node.save_map()
                    
                    # 计算总运行时间
                    total_runtime = time.time() - start_time
                    node.get_logger().info(f'总运行时间: {total_runtime:.1f}秒')
                    
                    # 打印处理统计
                    node.get_logger().info(f'保存最终地图到 {node.map_save_path}')
                    node.get_logger().info(f'处理统计: 扫描帧={node.processed_scans}, 处理点={node.processed_points}')
                except Exception as save_err:
                    node.get_logger().error(f'保存地图时出错: {str(save_err)}')
                    try:
                        # 尝试保存到备用位置
                        backup_path = '/tmp/emergency_map.png'
                        plt.figure(figsize=(10, 10))
                        plt.title('应急地图 (由于错误自动保存)')
                        plt.imshow(node.grid_map.T, cmap='binary', origin='lower')
                        plt.savefig(backup_path)
                        plt.close()
                        node.get_logger().info(f'已保存应急地图到: {backup_path}')
                    except:
                        node.get_logger().error('保存应急地图也失败，无法保存地图')
                
                # 打印最终统计信息
                total_obstacle_cells = np.sum(node.grid_map == 1)
                node.get_logger().info(f'最终地图统计: 轨迹点={len(node.robot_trajectory)}, 障碍物单元格={total_obstacle_cells}')
                if len(node.robot_trajectory) > 0:
                    node.get_logger().info(f'轨迹起点: ({node.robot_trajectory[0][0]:.2f}, {node.robot_trajectory[0][1]:.2f})')
                    node.get_logger().info(f'轨迹终点: ({node.robot_trajectory[-1][0]:.2f}, {node.robot_trajectory[-1][1]:.2f})')
                    
                # 打印结果摘要
                node.get_logger().info('结果摘要:')
                node.get_logger().info(f'  - 总运行时间: {total_runtime:.1f}秒')
                node.get_logger().info(f'  - 处理点云: {node.processed_points}个点')
                node.get_logger().info(f'  - 轨迹点数: {len(node.robot_trajectory)}个')
                node.get_logger().info(f'  - 障碍物单元格: {total_obstacle_cells}个')
                node.get_logger().info(f'  - 地图边界: [{node.min_x:.1f}, {node.max_x:.1f}]x[{node.min_y:.1f}, {node.max_y:.1f}]')
                node.get_logger().info(f'  - 地图保存路径: {node.map_save_path}')
                
                # 如果有中间检查点，显示它们的位置
                if hasattr(node, 'checkpoint_counter') and node.checkpoint_counter > 0:
                    node.get_logger().info(f'  - 中间检查点: {node.checkpoint_counter}个')
                    for i in range(1, node.checkpoint_counter + 1):
                        node.get_logger().info(f'    * 检查点 {i}: map_checkpoint_{i}.png')
            except Exception as final_err:
                node.get_logger().error(f'结束清理时出错: {str(final_err)}')
            finally:
                # 清理资源
                try:
                    node.destroy_node()
                    rclpy.shutdown()
                except:
                    pass
    except Exception as init_err:
        print(f"初始化ROS2时出错: {str(init_err)}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main() 