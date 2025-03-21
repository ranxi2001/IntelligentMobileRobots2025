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
import matplotlib

# 在脚本开头添加函数来检测和设置中文字体
def setup_chinese_font():
    """设置中文字体，尝试多种可能的字体"""
    # 常见的中文字体，按优先级排序
    chinese_fonts = ['WenQuanYi Zen Hei', 'WenQuanYi Micro Hei', 'Noto Sans CJK SC', 
                     'SimSun', 'SimHei', 'Microsoft YaHei', 
                     'AR PL UMing CN', 'AR PL UKai CN', 'NSimSun', 
                     'STSong', 'STFangsong', 'FangSong', 'KaiTi']
    
    # 检测系统中是否存在这些字体
    available_fonts = []
    for font in chinese_fonts:
        if font.lower() in [f.name.lower() for f in matplotlib.font_manager.fontManager.ttflist]:
            available_fonts.append(font)
            print(f"找到中文字体: {font}")
    
    if available_fonts:
        # 使用找到的第一个中文字体
        plt.rcParams['font.sans-serif'] = available_fonts + ['DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
        print(f"设置中文字体为: {available_fonts[0]}")
        return available_fonts[0]
    else:
        # 没有找到中文字体，使用默认设置
        print("未找到中文字体，使用默认设置")
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
        return None

# 设置中文字体
chinese_font = setup_chinese_font()
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号

# 移除固定的地图边界常量，改为函数来动态计算边界
def calculate_map_bounds(min_x, max_x, min_y, max_y, padding=5.0):
    """计算用于显示的地图边界，考虑边距"""
    
    # 确保我们至少包含理想显示区域
    min_x = min(min_x, -15.0)
    max_x = max(max_x, 45.0)
    min_y = min(min_y, -15.0)
    max_y = max(max_y, 50.0)
    
    # 根据实际数据进行有限的边界扩展
    return [
        min_x - padding,
        max_x + padding,
        min_y - padding,
        max_y + padding
    ]

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
    def __init__(self):
        super().__init__('lms_mapper')
        
        # 地图参数
        self.resolution = 0.1  # 每格代表0.1米
        self.size_x = 600  # X方向600格（覆盖60米）
        self.size_y = 650  # Y方向650格（覆盖65米）
        
        # 调整原点位置以优化显示
        self.origin_x = 165  # 世界坐标(-15,?)对应栅格坐标(0,?)
        self.origin_y = 165  # 世界坐标(?,15)对应栅格坐标(?,0)
        
        # 使用概率地图：0-100 (0=空闲, 100=占用, -1=未知)
        self.grid_map = np.ones((self.size_x, self.size_y), dtype=np.int8) * -1
        
        # 原点设在地图中心
        self.origin_x = self.size_x // 2
        self.origin_y = self.size_y // 2
        
        # 追踪地图边界
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        
        # 激光点数量累积计数
        self.hit_map = np.zeros((self.size_x, self.size_y), dtype=np.int16)
        self.miss_map = np.zeros((self.size_x, self.size_y), dtype=np.int16)
    
    def world_to_grid(self, x, y):
        """世界坐标系转栅格坐标系
        
        参数:
            x: 世界坐标系下的X坐标
            y: 世界坐标系下的Y坐标
            
        返回:
            (grid_x, grid_y): 栅格坐标系下的整数坐标
        """
        # 检查输入坐标是否在合理范围内 - 扩大允许范围
        if not -200 < x < 200 or not -200 < y < 200:
            print(f'世界坐标超出扩展范围: x={x}, y={y}')
        
        # 将世界坐标转换为栅格坐标
        # origin_x和origin_y表示地图原点(0,0)在栅格中的位置（通常是地图中心）
        # resolution是每个栅格代表的实际距离（单位：米）
        grid_x = int(self.origin_x + x / self.resolution)
        grid_y = int(self.origin_y + y / self.resolution)
        
        # 移除强制限制，允许坐标超出地图范围
        # 调用此函数的地方应自行检查坐标是否在有效范围内
        
        return grid_x, grid_y
    
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
            
            # 不再限制坐标范围
            # 障碍物点将在绘制地图时被考虑，即使超出当前栅格范围
            try:
                if 0 <= grid_x < self.size_x and 0 <= grid_y < self.size_y:
                    self.hit_map[grid_x, grid_y] += 1
            except:
                pass
        
        # 2. 从机器人到激光点之间的栅格为空闲
        for x, y in points:
            cells = self.ray_trace(robot_x, robot_y, x, y)
            
            # 最后一个栅格是障碍物，不算入空闲
            if cells:
                cells = cells[:-1]
                
            for cx, cy in cells:
                try:
                    if 0 <= cx < self.size_x and 0 <= cy < self.size_y:
                        self.miss_map[cx, cy] += 1
                except:
                    pass
        
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
            plt.plot(traj_x[0], traj_y[0], 'go', markersize=10, label='起点')
            plt.plot(traj_x[-1], traj_y[-1], 'ko', markersize=10, label='终点')  # 改为黑色终点标记
        
        # 添加标题和坐标轴标签
        plt.title('占用栅格图 (白色=空闲, 红色=障碍物, 绿色=起点与轨迹, 黑色=终点)', fontsize=14)
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
        super().__init__('lms_mapper')
        
        # 地图参数
        self.resolution = 0.1  # 每格代表0.1米
        self.size_x = 1000  # X方向1000格（覆盖100米）
        self.size_y = 1000  # Y方向1000格（覆盖100米）
        
        # 调整原点位置以优化显示
        self.origin_x = 150  # 增大原点位置，以便显示更多负坐标区域
        self.origin_y = 150  # 增大原点位置，以便显示更多负坐标区域
        
        # 声明参数
        self.declare_parameter('map_threshold', 0)  # 默认值改为0，使更多点能被识别为障碍物
        self.declare_parameter('filter_points', True)
        self.declare_parameter('filter_threshold', 1.0)
        self.declare_parameter('map_save_path', 'occupancy_grid.png')
        self.declare_parameter('map_reset_enabled', False)  # 添加新参数，默认为False
        self.declare_parameter('clear_expired_obstacles_enabled', False)  # 添加新参数，默认为False
        self.declare_parameter('save_interval', 60.0)  # 新增参数：中间地图保存间隔，默认60秒
        self.declare_parameter('motion_filter_enabled', False)  # 禁用运动过滤，显示所有数据
        self.declare_parameter('min_motion_distance', 0.1)  # 降低最小运动距离（米）
        self.declare_parameter('static_ignore_time', 1.0)  # 降低忽略静止时间（秒）
        self.declare_parameter('adaptive_threshold', False)  # 禁用自适应阈值，使用固定阈值
        self.declare_parameter('base_threshold', 0)  # 设置基础阈值为0
        self.declare_parameter('distance_threshold_factor', 0.1)  # 降低距离阈值因子
        
        # 获取参数
        self.threshold = self.get_parameter('map_threshold').value
        self.filter_points = self.get_parameter('filter_points').value
        self.filter_threshold = self.get_parameter('filter_threshold').value
        self.map_save_path = self.get_parameter('map_save_path').value
        self.map_reset_enabled = self.get_parameter('map_reset_enabled').value  # 读取参数
        self.clear_expired_obstacles_enabled = self.get_parameter('clear_expired_obstacles_enabled').value  # 读取参数
        self.save_interval = self.get_parameter('save_interval').value  # 读取中间地图保存间隔
        self.motion_filter_enabled = self.get_parameter('motion_filter_enabled').value
        self.min_motion_distance = self.get_parameter('min_motion_distance').value
        self.static_ignore_time = self.get_parameter('static_ignore_time').value
        self.adaptive_threshold = self.get_parameter('adaptive_threshold').value
        self.base_threshold = self.get_parameter('base_threshold').value
        self.distance_threshold_factor = self.get_parameter('distance_threshold_factor').value
        
        # 输出地图参数信息
        self.get_logger().info(f'地图参数: 尺寸={self.size_x}x{self.size_y}, 分辨率={self.resolution}米/格')
        
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
            
        # 输出运动过滤状态
        if self.motion_filter_enabled:
            self.get_logger().info(f'运动过滤功能已启用：最小运动距离={self.min_motion_distance}米，静止忽略时间={self.static_ignore_time}秒')
        else:
            self.get_logger().info('运动过滤功能已禁用，将记录所有时间段的激光数据')
            
        # 输出阈值设置
        if self.adaptive_threshold:
            self.get_logger().info(f'使用自适应阈值：基础阈值={self.base_threshold}，距离因子={self.distance_threshold_factor}')
        else:
            self.get_logger().info(f'使用固定阈值：{self.threshold}')
        
        # 创建占用栅格地图
        self.grid_map = np.zeros((self.size_x, self.size_y), dtype=np.int8)
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        self.occupancy_count = {}  # 用于跟踪每个栅格的激光点数
        
        # 存储机器人轨迹
        self.robot_trajectory = []
        
        # 增加轨迹同步变量
        self.traj_lock = threading.Lock()  # 添加线程锁保护轨迹数据
        self.map_lock = threading.Lock()   # 添加地图数据锁
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
        
        # 添加轨迹和运动状态变量
        self.start_pose = None  # 起始位姿
        self.last_moved_time = None  # 上次运动时间
        self.is_moving = False  # 当前是否在运动
        self.total_distance = 0.0  # 总行驶距离
        self.last_position = None  # 上次位置，用于计算移动距离
        self.in_static_start_phase = True  # 是否在起始静止阶段
        self.in_static_end_phase = False  # 是否在结束静止阶段
        self.static_start_time = None  # 开始静止的时间
        self.static_duration_threshold = 2.0  # 判定为静止状态的时间阈值（秒）
        
        # 添加地图重置和更新控制变量
        self.map_reset_interval = 3  # 每隔多少个扫描点重置地图
        self.scan_count_since_reset = 0
        self.map_enabled = True  # 地图构建开关
        self.dynamic_map = True  # 是否使用动态地图（根据机器人位置实时更新）
        self.max_obstacle_age = 10.0  # 障碍物最大存活时间(秒)
        self.obstacle_timestamps = {}  # 记录每个障碍物的时间戳
        
        # 存储障碍物点的原始世界坐标，用于后期处理
        self.obstacle_x = []
        self.obstacle_y = []
        
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
        
        # 将四元数转换为欧拉角（RPY）
        euler = self.quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        
        # 提取机器人位姿信息
        x = position.x
        y = position.y
        theta = euler[2]  # 取偏航角 (yaw)
        
        # 保存或更新机器人位姿
        self.latest_pose = {
            'x': x,
            'y': y,
            'theta': theta
        }
        
        # 记录机器人位置，用于轨迹绘制
        with self.traj_lock:
            self.robot_trajectory.append((x, y))
            self.new_trajectory_since_last_map = True
            
        # 检测是否是第一个里程计消息，用于保存起始位置
        if self.start_pose is None:
            self.start_pose = {'x': x, 'y': y, 'theta': theta}
            self.last_position = (x, y)
            self.last_moved_time = time.time()
            self.static_start_time = time.time()
            self.get_logger().info(f'记录起始位置: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
        
        # 检测机器人运动状态
        if self.motion_filter_enabled and self.last_position is not None:
            # 计算移动距离
            dist = np.sqrt((x - self.last_position[0])**2 + (y - self.last_position[1])**2)
            current_time = time.time()
            
            # 累计总行驶距离
            self.total_distance += dist
            
            # 判断是否在移动
            if dist >= self.min_motion_distance / 10.0:  # 使用较小阈值检测微小移动
                # 从静止变为移动
                if not self.is_moving:
                    self.is_moving = True
                    move_pause = current_time - (self.last_moved_time or current_time)
                    if move_pause > self.static_duration_threshold:
                        if self.in_static_start_phase:
                            self.in_static_start_phase = False
                            self.get_logger().info(f'检测到机器人开始行驶，总静止时间: {move_pause:.2f}秒，当前位置: ({x:.2f}, {y:.2f})')
                        elif not self.in_static_end_phase:
                            self.get_logger().info(f'机器人恢复移动，静止持续了: {move_pause:.2f}秒')
                
                self.last_moved_time = current_time
            else:
                # 记录静止状态持续时间
                if self.is_moving:
                    self.is_moving = False
                    self.static_start_time = current_time
                    
                # 如果已经行驶过，并且静止时间超过阈值，判断为终点静止阶段
                static_duration = current_time - (self.static_start_time or current_time)
                if (not self.in_static_start_phase and 
                    not self.in_static_end_phase and 
                    static_duration > self.static_duration_threshold):
                    self.in_static_end_phase = True
                    self.get_logger().info(f'检测到机器人可能已到达终点，静止时间: {static_duration:.2f}秒，总行驶距离: {self.total_distance:.2f}米')
                
                # 记录长时间静止情况
                if static_duration > 10.0 and (self._odom_count % 200 == 0):
                    self.get_logger().info(f'机器人静止中: {static_duration:.1f}秒, 位置: ({x:.2f}, {y:.2f})')
            
            # 更新上次位置
            self.last_position = (x, y)
    
    def scan_callback(self, msg):
        """处理激光雷达数据"""
        try:
            # 记录处理的帧数
            self.processed_scans += 1
            
            # 获取时间戳信息
            current_time = time.time()
            
            # 提取激光雷达数据信息，用于日志记录
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_increment = msg.angle_increment
            range_min = msg.range_min
            range_max = msg.range_max
            ranges = msg.ranges
            intensities = msg.intensities if hasattr(msg, 'intensities') else []
            
            # 获取激光雷达参考坐标系
            frame_id = msg.header.frame_id
            
            # 周期性地记录激光参数信息，帮助调试
            if self.processed_scans % 100 == 1:
                self.get_logger().info(f'接收到激光数据: frame_id={frame_id}, 点数={len(ranges)}')
            
            # 如果没有有效位姿，跳过此次处理
            if not self.latest_pose:
                if self.processed_scans % 10 == 0:  # 降低日志频率
                    self.get_logger().warn('尚未接收到有效的里程计位姿，无法处理激光数据')
                return
                
            # 记录当前机器人位姿，用于坐标转换
            robot_x = self.latest_pose['x']
            robot_y = self.latest_pose['y']
            robot_theta = self.latest_pose['theta']
            
            # 记录详细位姿信息（降低频率，避免日志过多）
            if self.processed_scans % 100 == 1:
                self.get_logger().info(f'【激光雷达坐标】x={robot_x:.4f}, y={robot_y:.4f}, theta={robot_theta:.4f}')
            
            # 使用简单运动过滤
            if self.motion_filter_enabled and self.last_position:
                # 计算移动距离
                move_dist = math.sqrt((robot_x - self.last_position[0])**2 + (robot_y - self.last_position[1])**2)
                
                # 如果移动距离小于阈值，记录静止开始时间
                if move_dist < self.min_motion_distance:
                    if not self.is_moving:  # 如果已经静止，继续累计时间
                        static_duration = current_time - self.last_moved_time if self.last_moved_time else 0
                        # 如果静止时间超过阈值且不是起始阶段，跳过此帧
                        if static_duration > self.static_ignore_time and not self.in_static_start_phase:
                            if self.processed_scans % 50 == 0:  # 降低日志频率
                                self.get_logger().info(f'机器人静止中 ({static_duration:.1f}秒)，跳过处理激光数据')
                            return
                    else:  # 如果是刚开始静止
                        self.is_moving = False
                        self.last_moved_time = current_time
                        if self.processed_scans % 10 == 0:
                            self.get_logger().info('机器人停止移动')
                else:
                    # 如果移动距离大于阈值，标记为移动状态
                    if not self.is_moving:
                        self.is_moving = True
                        if self.in_static_start_phase:
                            self.in_static_start_phase = False
                            self.get_logger().info('机器人开始移动，退出起始静止阶段')
                    
                    # 更新上次移动时间
                    self.last_moved_time = current_time
                    
                    # 累计总移动距离
                    self.total_distance += move_dist
                    
                    # 每米记录一次累计距离
                    if int(self.total_distance) > int(self.total_distance - move_dist):
                        self.get_logger().info(f'累计行驶距离: {self.total_distance:.2f}米')
            
            # 更新位置记录
            self.last_position = (robot_x, robot_y)
            
            # 如果是第一次收到扫描数据，记录起始位置
            if not self.first_scan_received:
                self.start_pose = self.latest_pose.copy()
                self.get_logger().info(f'记录起始位置: x={robot_x:.2f}, y={robot_y:.2f}, theta={robot_theta:.2f}')
                self.first_scan_received = True
                self.last_scan_time = current_time
            
            # 更新帧处理间隔信息
            if self.last_scan_time:
                scan_interval = current_time - self.last_scan_time
                # 如果间隔明显长于正常，记录警告
                if scan_interval > 0.5:  # 正常应该是0.1秒左右
                    self.get_logger().warn(f'激光扫描处理间隔较长: {scan_interval:.2f}秒')
            self.last_scan_time = current_time
            
            # 检查激光数据点数
            if len(ranges) == 0:
                self.get_logger().warn('接收到空的激光扫描数据')
                return
                
            # 生成对应的角度数组（弧度）
            angles = [angle_min + i * angle_increment for i in range(len(ranges))]
            
            # 记录扫描角度范围，用于诊断
            if self.processed_scans % 100 == 1:
                angle_min_deg = math.degrees(angle_min)
                angle_max_deg = math.degrees(angle_max)
                self.get_logger().info(f'接收到激光数据: {len(ranges)}个点, 角度范围=[{angle_min:.2f}, {angle_max:.2f}]')
            
            # 过滤无效距离数据
            valid_ranges = []
            valid_angles = []
            
            for i, r in enumerate(ranges):
                # 检查距离是否在有效范围内
                if range_min <= r <= range_max:
                    valid_ranges.append(r)
                    valid_angles.append(angles[i])
            
            # 记录有效点数
            valid_points_count = len(valid_ranges)
            total_points_count = len(ranges)
            
            if self.processed_scans % 100 == 1 or valid_points_count < total_points_count * 0.5:
                self.get_logger().info(f'有效激光点: {valid_points_count}/{total_points_count}')
            
            if valid_points_count == 0:
                self.get_logger().warn('没有有效的激光点')
                return
            
            # 计算激光点在世界坐标系中的坐标
            cos_theta = math.cos(robot_theta)
            sin_theta = math.sin(robot_theta)
            
            if self.processed_scans % 100 == 1:
                self.get_logger().info(f'机器人位姿: x={robot_x:.2f}, y={robot_y:.2f}, theta={robot_theta:.2f}, cos_theta={cos_theta:.2f}, sin_theta={sin_theta:.2f}')
            
            # 将激光雷达坐标系下的点转换到世界坐标系
            world_points = []
            for i, (r, theta) in enumerate(zip(valid_ranges, valid_angles)):
                # 计算激光点在激光雷达坐标系中的位置
                laser_x = r * math.cos(theta)
                laser_y = r * math.sin(theta)
                
                # 将点从激光雷达坐标系转换到世界坐标系
                world_x = robot_x + laser_x * cos_theta - laser_y * sin_theta
                world_y = robot_y + laser_x * sin_theta + laser_y * cos_theta
                
                world_points.append((world_x, world_y))
                
                # 保存所有障碍物点的坐标，用于最终地图绘制
                self.obstacle_x.append(world_x)
                self.obstacle_y.append(world_y)
            
            # 分析转换后的点分布
            if self.processed_scans % 100 == 1 and world_points:
                wx = [p[0] for p in world_points]
                wy = [p[1] for p in world_points]
                self.get_logger().info(f'激光点世界坐标范围: x=[{min(wx):.2f}, {max(wx):.2f}], y=[{min(wy):.2f}, {max(wy):.2f}]')
                
                # 添加一些坐标转换的样本点以帮助诊断
                sample_indices = [0, len(world_points)//4, len(world_points)//2, 3*len(world_points)//4, len(world_points)-1]
                samples = []
                for idx in sample_indices:
                    if 0 <= idx < len(world_points):
                        r = valid_ranges[idx]
                        angle = valid_angles[idx]
                        laser_x = r * math.cos(angle)
                        laser_y = r * math.sin(angle)
                        world_x, world_y = world_points[idx]
                        samples.append(f'激光坐标({laser_x:.2f},{laser_y:.2f})→世界坐标({world_x:.2f},{world_y:.2f})')
                
                if samples:
                    self.get_logger().info(f'【坐标转换样本】: {" | ".join(samples)}')
            
            # 更新地图
            if world_points:
                world_points_np = np.array(world_points)
                self.update_map(world_points_np, current_time)
                self.processed_points += len(world_points)
            
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
                
            # 添加日志输出当前使用的阈值设置
            if not hasattr(self, '_threshold_logged') or not self._threshold_logged:
                self.get_logger().info(f'当前使用的障碍物计数阈值: {self.threshold}')
                self._threshold_logged = True
                
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
            
            # 计算点与机器人的距离 - 用于优先处理近距离障碍物和自适应阈值
            def calculate_distance_to_robot(point):
                """计算点到机器人的距离"""
                x, y = point
                dx = x - robot_x
                dy = y - robot_y
                return np.sqrt(dx*dx + dy*dy)
            
            # 对于批量点，优化处理方式
            if isinstance(points, np.ndarray) and points.ndim == 2 and points.shape[1] == 2:
                # 使用向量化操作计算所有点到机器人的距离
                dx = points[:, 0] - robot_x
                dy = points[:, 1] - robot_y
                distances = np.sqrt(dx*dx + dy*dy)
                
                # 记录调试信息
                if len(points) > 100 and hasattr(self, '_map_stats_count') and self._map_stats_count % 50 == 0:
                    # 找出最近的几个点，记录到日志
                    closest_indices = np.argsort(distances)[:5]
                    closest_points = points[closest_indices]
                    closest_distances = distances[closest_indices]
                    
                    # 记录最近的障碍物
                    self.get_logger().info(f'【最近障碍物】距离机器人: {", ".join([f"({p[0]:.2f},{p[1]:.2f})={d:.2f}m" for p,d in zip(closest_points, closest_distances)])}')
                
                # 处理每个世界点
                for i in range(len(points)):
                    x, y = points[i]
                    distance = distances[i]
                    
                    # 将世界坐标转换为栅格坐标
                    grid_x, grid_y = self.world_to_grid(x, y)
                    
                    # 确保栅格坐标在地图范围内
                    if 0 <= grid_x < self.size_x and 0 <= grid_y < self.size_y:
                        # 检查是否使用自适应阈值
                        if self.adaptive_threshold:
                            # 根据距离计算局部阈值 - 远处使用更高阈值，减少噪声
                            local_threshold = self.base_threshold
                            
                            # 根据距离增加阈值，但限制增长幅度使远距离不会太高
                            distance_factor = max(0, min(distance - 5.0, 10.0)) * self.distance_threshold_factor
                            local_threshold += math.floor(distance_factor)
                            
                            # 对特定区域进行特殊处理
                            # 左侧区域（x值较小的区域）使用更高的阈值 - 通常有更多噪声
                            if x < 0:  
                                local_threshold = max(local_threshold + 1, 1)  # 降低最小值为1
                            
                            # 远距离区域使用更高的阈值，但最大不超过2
                            if distance > 20.0:
                                local_threshold = max(local_threshold, min(2, local_threshold + 1))
                            
                            # 获取或初始化该栅格的计数
                            grid_key = (grid_x, grid_y)
                            grid_count = self.occupancy_count.get(grid_key, 0) + 1
                            self.occupancy_count[grid_key] = grid_count
                            
                            # 更新障碍物时间戳
                            self.obstacle_timestamps[grid_key] = current_time
                            
                            # 使用自适应阈值标记栅格
                            if grid_count >= local_threshold:  # 修改为>=，使更多点被识别为障碍物
                                self.grid_map[grid_x, grid_y] = 1
                                points_added += 1
                        else:
                            # 使用固定阈值
                            grid_key = (grid_x, grid_y)
                            grid_count = self.occupancy_count.get(grid_key, 0) + 1
                            self.occupancy_count[grid_key] = grid_count
                            
                            # 更新时间戳
                            self.obstacle_timestamps[grid_key] = current_time
                            
                            # 使用固定阈值标记栅格
                            if grid_count >= self.threshold:  # 修改为>=，使更多点被识别为障碍物
                                self.grid_map[grid_x, grid_y] = 1
                                points_added += 1
                
                # 更新地图边界 - 用于优化显示
                if points_added > 0:
                    self.min_x = min(self.min_x, np.min(points[:, 0]))
                    self.max_x = max(self.max_x, np.max(points[:, 0]))
                    self.min_y = min(self.min_y, np.min(points[:, 1]))
                    self.max_y = max(self.max_y, np.max(points[:, 1]))
            
            # 清除过期的障碍物 - 仅在启用该功能时执行
            if self.clear_expired_obstacles_enabled:
                self._clean_expired_obstacles(current_time)
            
            # 用于监控更新频率
            if not hasattr(self, '_map_stats_count'):
                self._map_stats_count = 0
            self._map_stats_count += 1
            
            # 每100次更新记录一次统计信息
            if self._map_stats_count % 100 == 0:
                occupied_count = np.sum(self.grid_map == 1)
                total_count = self.size_x * self.size_y
                occupied_percent = occupied_count / total_count * 100
                self.get_logger().info(f'地图统计: 总栅格数={total_count}, 占用数={occupied_count}, 占用率={occupied_percent:.2f}%')
                self.get_logger().info(f'地图边界: x=[{self.min_x:.2f}, {self.max_x:.2f}], y=[{self.min_y:.2f}, {self.max_y:.2f}]')
                self.get_logger().info(f'总共处理了{self.processed_scans}个激光帧, {self.processed_points}个有效激光点')
                
                # 添加障碍物点统计
                obstacle_count = len(self.obstacle_x)
                if obstacle_count > 0:
                    self.get_logger().info(f'已记录{obstacle_count}个原始障碍物点坐标，用于最终地图生成')
        
        except Exception as e:
            # 捕获所有异常，防止崩溃
            self.get_logger().error(f'更新地图时发生错误: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
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
        # 检查输入坐标是否在合理范围内 - 扩大允许范围
        if not -200 < x < 200 or not -200 < y < 200:
            print(f'世界坐标超出扩展范围: x={x}, y={y}')
        
        # 将世界坐标转换为栅格坐标
        # origin_x和origin_y表示地图原点(0,0)在栅格中的位置（通常是地图中心）
        # resolution是每个栅格代表的实际距离（单位：米）
        grid_x = int(self.origin_x + x / self.resolution)
        grid_y = int(self.origin_y + y / self.resolution)
        
        # 移除强制限制，允许坐标超出地图范围
        # 调用此函数的地方应自行检查坐标是否在有效范围内
        
        return grid_x, grid_y
    
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
            # 保存所有原始障碍物点（世界坐标系）
            all_obstacle_points = []
            
            # 收集所有观察到的障碍物点（包括可能在地图外的点）
            with self.map_lock:
                for i in range(len(self.obstacle_x)):
                    all_obstacle_points.append((self.obstacle_x[i], self.obstacle_y[i]))
            
            # 如果没有障碍物点，日志提示并使用一个空列表
            if not all_obstacle_points:
                self.get_logger().warn('没有检测到任何障碍物点')
                all_obstacle_points = []
            else:
                self.get_logger().info(f'收集到 {len(all_obstacle_points)} 个原始障碍物点')
                
                # 分析障碍物点分布
                obs_x = [p[0] for p in all_obstacle_points]
                obs_y = [p[1] for p in all_obstacle_points]
                min_obs_x, max_obs_x = min(obs_x), max(obs_x)
                min_obs_y, max_obs_y = min(obs_y), max(obs_y)
                self.get_logger().info(f'障碍物世界坐标范围: X=[{min_obs_x:.2f}, {max_obs_x:.2f}], Y=[{min_obs_y:.2f}, {max_obs_y:.2f}]')
            
            # 设置合适的地图显示范围，确保包含所有障碍物和轨迹
            # 安全获取轨迹副本
            current_trajectory = []
            with self.traj_lock:
                current_trajectory = self.robot_trajectory.copy()
            
            # 如果有轨迹点，分析轨迹范围
            if current_trajectory:
                self.get_logger().info(f'轨迹包含 {len(current_trajectory)} 个点')
                traj_x = [p[0] for p in current_trajectory]
                traj_y = [p[1] for p in current_trajectory]
                min_traj_x, max_traj_x = min(traj_x), max(traj_x)
                min_traj_y, max_traj_y = min(traj_y), max(traj_y)
                self.get_logger().info(f'轨迹世界坐标范围: X=[{min_traj_x:.2f}, {max_traj_x:.2f}], Y=[{min_traj_y:.2f}, {max_traj_y:.2f}]')
            
            # 计算数据的整体范围
            all_points = all_obstacle_points + current_trajectory
            if not all_points:
                # 如果没有任何点，使用默认范围
                map_bounds = [-15.0, 45.0, -15.0, 50.0]
                self.get_logger().warn('没有任何障碍物或轨迹点，使用默认显示范围')
            else:
                # 动态计算范围，包含所有点并添加边距
                all_x = [p[0] for p in all_points]
                all_y = [p[1] for p in all_points]
                margin = 20.0  # 增大边距到20米
                min_x, max_x = min(all_x) - margin, max(all_x) + margin
                min_y, max_y = min(all_y) - margin, max(all_y) + margin
                map_bounds = [min_x, max_x, min_y, max_y]
                self.get_logger().info(f'动态计算的显示范围: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]')
            
            # 创建图像
            fig = plt.figure(figsize=(14, 14))
            
            # 设置背景色为白色
            plt.gca().set_facecolor('#FFFFFF')
            
            # 创建自定义障碍物点大小 - 根据机器人运动距离调整大小
            if all_obstacle_points and current_trajectory:
                # 计算每个障碍物点到起点的距离
                start_point = current_trajectory[0]
                
                # 根据距离过滤和调整障碍物点大小
                filtered_obstacles = []
                obstacle_sizes = []
                
                # 设置合适的起始距离阈值 - 小车开始行驶后记录
                start_distance_threshold = 0.0  # 设置为0，显示所有障碍物点
                
                for point in all_obstacle_points:
                    # 找到离此障碍物点最近的轨迹点
                    min_dist = float('inf')
                    nearest_traj_idx = 0
                    
                    for i, traj_point in enumerate(current_trajectory):
                        dist = np.sqrt((point[0] - traj_point[0])**2 + (point[1] - traj_point[1])**2)
                        if dist < min_dist:
                            min_dist = dist
                            nearest_traj_idx = i
                    
                    # 计算该轨迹点到起点的距离
                    if nearest_traj_idx == 0:
                        traj_dist_from_start = 0
                    else:
                        # 计算轨迹点到起点的累积距离
                        traj_dist_from_start = 0
                        for i in range(1, nearest_traj_idx + 1):
                            prev = current_trajectory[i-1]
                            curr = current_trajectory[i]
                            traj_dist_from_start += np.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
                    
                    # 检查是否达到了起始行驶距离要求
                    if traj_dist_from_start < start_distance_threshold:
                        # 刚开始时的点可能不准确，可选择跳过或使用更高阈值
                        if min_dist > 1.0:  # 过滤掉开始阶段远离轨迹的点
                            continue
                    
                    # 根据点到轨迹的距离调整点大小
                    point_size = max(20, min(80, 50 / (min_dist + 0.5)))  # 增大点大小
                    
                    # 存储过滤后的障碍物点及其大小
                    filtered_obstacles.append(point)
                    obstacle_sizes.append(point_size)
                
                if filtered_obstacles:
                    obs_x = [p[0] for p in filtered_obstacles]
                    obs_y = [p[1] for p in filtered_obstacles]
                    # 使用更鲜明的红色，提高可见性
                    plt.scatter(obs_x, obs_y, color='#FF0000', s=obstacle_sizes, alpha=0.9, label='障碍物')
                    self.get_logger().info(f'过滤后显示 {len(filtered_obstacles)} 个障碍物点')
                else:
                    self.get_logger().warn('过滤后没有障碍物点可显示')
            elif all_obstacle_points:
                # 如果没有轨迹信息，仍然显示所有障碍物
                obs_x = [p[0] for p in all_obstacle_points]
                obs_y = [p[1] for p in all_obstacle_points]
                plt.scatter(obs_x, obs_y, color='#FF0000', s=30, label='障碍物')
            
            # 绘制轨迹 - 使用渐变色显示时间进度
            if current_trajectory and len(current_trajectory) > 1:
                # 创建轨迹的颜色映射，从青蓝色渐变到深蓝色
                traj_points = np.array(current_trajectory)
                traj_segments = np.array([traj_points[:-1], traj_points[1:]]).transpose(1, 0, 2)
                
                # 创建颜色映射
                norm = plt.Normalize(0, len(traj_segments))
                colors = plt.cm.viridis(norm(range(len(traj_segments))))
                
                # 绘制分段线条
                for i, (segment, color) in enumerate(zip(traj_segments, colors)):
                    plt.plot([segment[0][0], segment[1][0]], 
                            [segment[0][1], segment[1][1]], 
                            color=color, linewidth=2.5, solid_capstyle='round')
                
                # 每隔一定间隔标记位置
                marker_interval = max(1, len(current_trajectory) // 10)  # 最多标记10个点
                for i in range(0, len(current_trajectory), marker_interval):
                    if i > 0 and i < len(current_trajectory) - 1:  # 跳过起点和终点
                        plt.plot(current_trajectory[i][0], current_trajectory[i][1], 'bo', markersize=6, alpha=0.7)
                
                # 标记起点和终点
                plt.plot(current_trajectory[0][0], current_trajectory[0][1], 'go', markersize=12, markeredgecolor='black', label='起点')
                plt.plot(current_trajectory[-1][0], current_trajectory[-1][1], 'ro', markersize=12, markeredgecolor='black', label='终点')
                
                # 添加自定义图例
                from matplotlib.lines import Line2D
                legend_elements = [
                    Line2D([0], [0], color='blue', lw=2, label='机器人轨迹'),
                    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='g', markersize=10, label='起点'),
                    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='r', markersize=10, label='终点'),
                    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='障碍物')
                ]
                plt.legend(handles=legend_elements, loc='upper right', fontsize=12)
            else:
                # 简单的图例
                plt.legend(loc='upper right', fontsize=12)
            
            # 设置坐标范围
            plt.xlim(map_bounds[0], map_bounds[1])
            plt.ylim(map_bounds[2], map_bounds[3])
            
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
            
            plt.xlabel('X 坐标 (米)', fontsize=14)
            plt.ylabel('Y 坐标 (米)', fontsize=14)
            plt.title('激光地图 (红色=障碍物, 彩色轨迹=机器人路径)', fontsize=16, fontweight='bold')
            
            # 保存并关闭图像
            try:
                # 确保目标目录存在
                save_dir = os.path.dirname(self.map_save_path)
                if save_dir and not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                
                plt.savefig(self.map_save_path, dpi=300, bbox_inches='tight', facecolor='white')
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
            import traceback
            self.get_logger().error(traceback.format_exc())
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
                plt.plot(traj_x[0], traj_y[0], 'go', markersize=10, label='起点')
                plt.plot(traj_x[-1], traj_y[-1], 'ko', markersize=10, label='终点')  # 改为红色终点标记
                
                # 使用动态计算的边界，不再使用固定值
                margin = 5.0  # 边距5米
                map_bounds = [min(traj_x) - margin, max(traj_x) + margin, 
                             min(traj_y) - margin, max(traj_y) + margin]
                self.get_logger().info(f'轨迹图使用动态显示范围: X=[{map_bounds[0]:.2f}, {map_bounds[1]:.2f}], Y=[{map_bounds[2]:.2f}, {map_bounds[3]:.2f}]')
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
            
            # 绘制机器人轨迹（蓝色）- 使用当前同步的轨迹
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
                                        rgb_map[gy, gx] = [0, 0, 255]  # 蓝色点
                            
                            # 绘制连接线
                            if prev_x is not None and prev_y is not None:
                                # 使用Bresenham算法绘制线段
                                try:
                                    line_points = self.bresenham_line(prev_x, prev_y, grid_x, grid_y)
                                    for lx, ly in line_points:
                                        if 0 <= lx < self.size_x and 0 <= ly < self.size_y:
                                            rgb_map[ly, lx] = [0, 0, 200]  # 稍暗的蓝色线
                                except:
                                    pass
                        
                        prev_x, prev_y = grid_x, grid_y
                    except:
                        pass
                
                # 特别标记起点和终点
                if len(current_trajectory) >= 2:
                    try:
                        # 起点(绿色)
                        start_x, start_y = self.world_to_grid(current_trajectory[0][0], current_trajectory[0][1])
                        for dx in range(-4, 5):
                            for dy in range(-4, 5):
                                gx, gy = start_x + dx, start_y + dy
                                if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                                    # 绘制圆形起点标记
                                    if dx*dx + dy*dy <= 16:
                                        rgb_map[gy, gx] = [0, 255, 0]  # 起点绿色
                        
                        # 终点(红色)
                        end_x, end_y = self.world_to_grid(current_trajectory[-1][0], current_trajectory[-1][1])
                        for dx in range(-4, 5):
                            for dy in range(-4, 5):
                                gx, gy = end_x + dx, end_y + dy
                                if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                                    # 绘制X形终点标记
                                    if abs(dx) == abs(dy):
                                        rgb_map[gy, gx] = [0, 0, 0]  # 终点黑色
                    except Exception as mark_err:
                        self.get_logger().warn(f'标记轨迹起终点时出错: {str(mark_err)}')
            else:
                self.get_logger().warn('没有轨迹点可绘制')
            
            # 保存简化版的地图
            fig = plt.figure(figsize=(12, 12))  # 优化图像尺寸
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
                        min(self.min_x, traj_min_x) - 10.0,  # 减少左侧额外空间
                        max(self.max_x, traj_max_x) + 10.0,  # 减少右侧额外空间
                        min(self.min_y, traj_min_y) - 10.0,  # 减少下方额外空间
                        max(self.max_y, traj_max_y) + 15.0   # 保持上方一定空间
                    )
                
                # 输出当前地图边界以便调试
                self.get_logger().info(f'地图显示边界: X={map_bounds[0]:.1f}到{map_bounds[1]:.1f}, Y={map_bounds[2]:.1f}到{map_bounds[3]:.1f}')
                
                # 只保留一次imshow调用
                plt.imshow(rgb_map, origin='lower', extent=extent)
                
                # 设置动态计算的显示范围
                plt.xlim(map_bounds[0], map_bounds[1])
                plt.ylim(map_bounds[2], map_bounds[3])
                
                # 计算合适的网格线间隔 - 提高网格线密度使地图更清晰
                x_range = map_bounds[1] - map_bounds[0]
                y_range = map_bounds[3] - map_bounds[2]
                grid_step_m = min(5, max(x_range, y_range) / 10)  # 增加网格线数量
                grid_step_m = max(1, round(grid_step_m))  # 至少1米，并取整
                
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
                    plt.plot(traj_x, traj_y, color='blue', linewidth=2.5, alpha=0.8)  # 蓝色轨迹线
                    plt.plot(traj_x[0], traj_y[0], 'go', markersize=10)  # 绿色起点
                    plt.plot(traj_x[-1], traj_y[-1], 'ko', markersize=10)  # 黑色终点
                    
                    # 添加坐标轴标签和图例
                    plt.xlabel('X坐标 (米)', fontsize=12)
                    plt.ylabel('Y坐标 (米)', fontsize=12)
                    plt.legend(['机器人轨迹', '起点', '终点'], loc='upper right', fontsize=10)
                
                # 提高DPI以增加清晰度
                plt.savefig(checkpoint_path, dpi=150)
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