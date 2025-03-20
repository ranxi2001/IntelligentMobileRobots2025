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
        """将世界坐标转换为栅格坐标"""
        grid_x = int(self.origin_x + x / self.resolution)
        grid_y = int(self.origin_y + y / self.resolution)
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
        
        # 保存图像或显示
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"地图已保存至: {save_path}")
        else:
            plt.show()
        
        # 关闭图形以释放资源
        plt.close()


class LmsMapperNode(Node):
    """LMS数据处理和栅格地图生成节点"""
    
    def __init__(self):
        super().__init__('lms_mapper_node')
        
        # 声明参数
        self.declare_parameter('map_resolution', 0.2)
        self.declare_parameter('map_size', 500)
        self.declare_parameter('map_threshold', 1)
        self.declare_parameter('filter_points', True)
        self.declare_parameter('filter_threshold', 1.0)
        self.declare_parameter('map_save_path', 'occupancy_grid.png')
        
        # 获取参数
        self.resolution = self.get_parameter('map_resolution').value
        self.map_size = self.get_parameter('map_size').value
        self.threshold = self.get_parameter('map_threshold').value
        self.filter_points = self.get_parameter('filter_points').value
        self.filter_threshold = self.get_parameter('filter_threshold').value
        self.map_save_path = self.get_parameter('map_save_path').value
        
        # 创建占用栅格地图
        self.size_x = self.map_size
        self.size_y = self.map_size
        self.grid_map = np.zeros((self.size_x, self.size_y), dtype=int)
        self.origin_x = self.size_x // 2
        self.origin_y = self.size_y // 2
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        self.occupancy_count = {}  # 用于跟踪每个栅格的激光点数
        
        # 存储机器人轨迹
        self.robot_trajectory = []
        
        # 记录最近收到的位姿
        self.latest_pose = None
        self.first_scan_received = False
        self.last_scan_time = None
        
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
        
        self.get_logger().info('LMS mapper node已启动')
    
    def odom_callback(self, msg):
        """处理里程计消息，获取机器人位姿"""
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        
        # 打印详细调试信息
        self.get_logger().info(f'接收到里程计消息: frame_id={msg.header.frame_id}, child_frame_id={msg.child_frame_id}')
        self.get_logger().info(f'位置: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}')
        
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
        
        # 记录轨迹点
        self.robot_trajectory.append((position.x, position.y))
        
        # 添加调试信息
        self.get_logger().info(f'添加轨迹点 #{len(self.robot_trajectory)}: ({position.x:.2f}, {position.y:.2f})')
        if len(self.robot_trajectory) % 10 == 0:  # 每10个点打印一次，避免日志过多
            self.get_logger().info(f'轨迹统计: 总点数={len(self.robot_trajectory)}, 开始=({self.robot_trajectory[0][0]:.2f}, {self.robot_trajectory[0][1]:.2f}), 最新=({position.x:.2f}, {position.y:.2f})')
    
    def scan_callback(self, msg):
        """处理激光扫描消息，更新占用栅格地图"""
        if self.latest_pose is None:
            self.get_logger().warn('接收到激光数据，但没有位姿信息，跳过处理')
            return
        
        # 获取当前扫描的时间戳和frame_id
        current_scan_time = self.get_clock().now().nanoseconds // 1000000
        self.get_logger().info(f'接收到激光数据: frame_id={msg.header.frame_id}, 点数={len(msg.ranges)}')
        
        # 如果是第一帧，记录时间
        if not self.first_scan_received:
            self.first_scan_received = True
            self.last_scan_time = current_scan_time
            self.get_logger().info('接收到第一帧激光数据')
        
        # 计算与上一帧的时间差
        time_diff = current_scan_time - self.last_scan_time
        self.last_scan_time = current_scan_time
        
        # 如果时间差过小，可能是重复帧，跳过处理
        if time_diff < 10:  # 小于10毫秒认为是重复帧
            return
        
        # 获取激光扫描数据
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angles = np.arange(len(ranges)) * angle_increment + angle_min
        
        # 添加调试信息
        if len(ranges) > 0:
            self.get_logger().info(f'接收到激光数据: {len(ranges)}个点, 角度范围=[{angle_min:.2f}, {angle_min + angle_increment * (len(ranges)-1):.2f}]')
        
        # 筛选有效的激光点
        valid_indices = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges > 0.1) & (ranges < 30.0)
        
        # 额外的点过滤 - 类似作业2的连续性检查
        if self.filter_points:
            for i in range(1, len(ranges)-1):
                if valid_indices[i]:
                    # 计算与相邻点的差异
                    diff_prev = abs(ranges[i] - ranges[i-1]) if valid_indices[i-1] else float('inf')
                    diff_next = abs(ranges[i] - ranges[i+1]) if valid_indices[i+1] else float('inf')
                    
                    # 如果与相邻点的差异过大，标记为无效
                    if diff_prev > self.filter_threshold and diff_next > self.filter_threshold:
                        valid_indices[i] = False
        
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        if len(valid_ranges) == 0:
            self.get_logger().warn('没有有效的激光点')
            return
        
        self.get_logger().info(f'有效激光点: {len(valid_ranges)}/{len(ranges)}')
        
        # 转换激光点到机器人坐标系
        laser_x = valid_ranges * np.cos(valid_angles)
        laser_y = valid_ranges * np.sin(valid_angles)
        
        # 获取机器人位姿
        robot_x = self.latest_pose['x']
        robot_y = self.latest_pose['y']
        robot_theta = self.latest_pose['theta']
        
        # 转换到世界坐标系
        cos_theta = np.cos(robot_theta)
        sin_theta = np.sin(robot_theta)
        world_x = robot_x + laser_x * cos_theta - laser_y * sin_theta
        world_y = robot_y + laser_x * sin_theta + laser_y * cos_theta
        
        # 更新占用栅格地图
        world_points = np.column_stack((world_x, world_y))
        self.update_map(world_points)
        
        # 添加调试信息
        self.get_logger().info(f'更新地图: 添加了{len(world_points)}个点')
    
    def update_map(self, points):
        """更新占用栅格地图"""
        points_added = 0
        
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
        
        if points_added > 0:
            self.get_logger().info(f'添加了{points_added}个新障碍物点到地图')
        
        # 定期打印地图统计信息
        total_obstacle_cells = np.sum(self.grid_map == 1)
        if total_obstacle_cells % 100 == 0 and total_obstacle_cells > 0:
            self.get_logger().info(f'地图统计: 障碍物单元格={total_obstacle_cells}, 地图边界=[{self.min_x:.1f}, {self.max_x:.1f}]x[{self.min_y:.1f}, {self.max_y:.1f}]')
    
    def world_to_grid(self, x, y):
        """世界坐标系转栅格坐标系"""
        grid_x = int(self.origin_x + x / self.resolution)
        grid_y = int(self.origin_y + y / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """栅格坐标系转世界坐标系"""
        x = (grid_x - self.origin_x) * self.resolution
        y = (grid_y - self.origin_y) * self.resolution
        return x, y
    
    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角"""
        # 计算欧拉角
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        
        return (roll, pitch, yaw)
    
    def publish_map(self):
        """发布占用栅格地图"""
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
        
        # 将二进制地图转换为占用率
        occupancy_values = np.zeros(self.size_x * self.size_y, dtype=np.int8)
        for i in range(self.size_x):
            for j in range(self.size_y):
                if self.grid_map[i, j] == 1:
                    # 占用 (100%)
                    occupancy_values[j * self.size_x + i] = 100
                else:
                    # 未知 (-1) 或空闲 (0)
                    occupancy_values[j * self.size_x + i] = 0
        
        map_msg.data = list(occupancy_values)
        self.map_publisher.publish(map_msg)
    
    def save_map(self):
        """保存占用栅格地图为图像文件"""
        # 创建彩色地图图像
        rgb_map = np.zeros((self.size_y, self.size_x, 3), dtype=np.uint8)
        
        # 白色背景
        rgb_map.fill(255)
        
        # 红色表示障碍物
        obstacle_indices = np.where(self.grid_map == 1)
        if len(obstacle_indices[0]) > 0:
            rgb_map[obstacle_indices[1], obstacle_indices[0]] = [255, 0, 0]  # RGB: 红色
        
        # 添加网格线
        grid_step = max(1, int(10 / self.resolution))  # 每10米一条线
        for i in range(0, self.size_x, grid_step):
            rgb_map[:, i] = [0, 0, 255]  # 蓝色垂直线
        for i in range(0, self.size_y, grid_step):
            rgb_map[i, :] = [0, 0, 255]  # 蓝色水平线
        
        # 绘制机器人轨迹（绿色）
        if self.robot_trajectory:
            self.get_logger().info(f'绘制轨迹: {len(self.robot_trajectory)}个点')
            
            # 计算轨迹的边界
            traj_x = [p[0] for p in self.robot_trajectory]
            traj_y = [p[1] for p in self.robot_trajectory]
            traj_min_x = min(traj_x)
            traj_max_x = max(traj_x)
            traj_min_y = min(traj_y)
            traj_max_y = max(traj_y)
            
            # 绘制所有轨迹点连线
            prev_x, prev_y = None, None
            for x, y in self.robot_trajectory:
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
                    line_points = self.bresenham_line(prev_x, prev_y, grid_x, grid_y)
                    for lx, ly in line_points:
                        if 0 <= lx < self.size_x and 0 <= ly < self.size_y:
                            rgb_map[ly, lx] = [0, 200, 0]  # 稍暗的绿色线
                
                prev_x, prev_y = grid_x, grid_y
            
            # 特别标记起点和终点
            if len(self.robot_trajectory) >= 2:
                # 起点(蓝色)
                start_x, start_y = self.world_to_grid(self.robot_trajectory[0][0], self.robot_trajectory[0][1])
                for dx in range(-4, 5):
                    for dy in range(-4, 5):
                        gx, gy = start_x + dx, start_y + dy
                        if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                            # 绘制圆形起点标记
                            if dx*dx + dy*dy <= 16:
                                rgb_map[gy, gx] = [0, 0, 255]  # 起点蓝色
                
                # 终点(红色)
                end_x, end_y = self.world_to_grid(self.robot_trajectory[-1][0], self.robot_trajectory[-1][1])
                for dx in range(-4, 5):
                    for dy in range(-4, 5):
                        gx, gy = end_x + dx, end_y + dy
                        if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                            # 绘制X形终点标记
                            if abs(dx) == abs(dy):
                                rgb_map[gy, gx] = [255, 0, 0]  # 终点红色
        else:
            self.get_logger().warn('没有轨迹点可绘制')
        
        # 保存图像
        plt.figure(figsize=(12, 12))
        
        # 设置固定显示范围，借鉴作业二的设置
        world_min_x = -10
        world_max_x = 45
        world_min_y = -14
        world_max_y = 50
        
        # 转换世界坐标为栅格坐标
        grid_min_x, grid_min_y = self.world_to_grid(world_min_x, world_min_y)
        grid_max_x, grid_max_y = self.world_to_grid(world_max_x, world_max_y)
        
        # 确保坐标在地图范围内
        grid_min_x = max(0, min(grid_min_x, self.size_x-1))
        grid_max_x = max(0, min(grid_max_x, self.size_x-1))
        grid_min_y = max(0, min(grid_min_y, self.size_y-1))
        grid_max_y = max(0, min(grid_max_y, self.size_y-1))
        
        # 裁剪显示范围
        display_map = rgb_map[grid_min_y:grid_max_y, grid_min_x:grid_max_x]
        
        plt.imshow(display_map, origin='lower', 
                  extent=[world_min_x, world_max_x, world_min_y, world_max_y])
        
        # 设置网格线
        grid_step_m = 10  # 10米一条网格线
        plt.grid(True, color='gray', linestyle='-', linewidth=0.5, alpha=0.3)
        plt.xticks(np.arange(world_min_x, world_max_x+1, grid_step_m))
        plt.yticks(np.arange(world_min_y, world_max_y+1, grid_step_m))
        
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('占用栅格图 (白色=空闲, 红色=障碍物)', fontsize=14)
        
        # 保存并关闭图像
        plt.savefig(self.map_save_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'地图已保存到: {self.map_save_path}')
        
        # 如果轨迹点数量适中，额外保存一个只包含轨迹的图
        if len(self.robot_trajectory) > 0:
            plt.figure(figsize=(12, 12))
            traj_x = [p[0] for p in self.robot_trajectory]
            traj_y = [p[1] for p in self.robot_trajectory]
            plt.plot(traj_x, traj_y, 'g-', linewidth=2, markersize=2)
            
            # 标记起点和终点
            plt.plot(traj_x[0], traj_y[0], 'bo', markersize=10, label='起点')
            plt.plot(traj_x[-1], traj_y[-1], 'ro', markersize=10, label='终点')
            
            # 设置固定的显示范围
            plt.xlim(world_min_x, world_max_x)
            plt.ylim(world_min_y, world_max_y)
            
            # 添加网格线
            plt.grid(True, color='gray', linestyle='-', linewidth=0.5, alpha=0.5)
            plt.xticks(np.arange(world_min_x, world_max_x+1, grid_step_m))
            plt.yticks(np.arange(world_min_y, world_max_y+1, grid_step_m))
            
            plt.xlabel('X (m)', fontsize=12)
            plt.ylabel('Y (m)', fontsize=12)
            plt.title('机器人轨迹', fontsize=14)
            plt.legend(loc='upper right')
            
            plt.savefig('robot_trajectory.png', dpi=300, bbox_inches='tight')
            plt.close()
            self.get_logger().info('轨迹图已保存到: robot_trajectory.png')

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


def main(args=None):
    rclpy.init(args=args)
    
    node = LmsMapperNode()
    node.get_logger().info('LMS mapper node已启动，等待数据...')
    
    try:
        # 主动处理一段时间的消息
        timeout = 0
        max_timeout = 60  # 最大等待时间（秒）
        
        while rclpy.ok() and timeout < max_timeout:
            rclpy.spin_once(node, timeout_sec=1.0)
            
            # 检查是否已接收到数据
            if len(node.robot_trajectory) > 0:
                node.get_logger().info(f'已接收轨迹点: {len(node.robot_trajectory)}')
                if timeout == 0:
                    node.get_logger().info('开始接收数据，继续处理...')
            else:
                timeout += 1
                if timeout % 5 == 0:  # 每5秒提示一次
                    node.get_logger().warn(f'等待数据中... {timeout}/{max_timeout}秒')
        
        # 主循环处理
        if timeout < max_timeout:
            node.get_logger().info('进入主处理循环')
            rclpy.spin(node)
        else:
            node.get_logger().error('超时未接收到数据，退出')
    
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，退出')
    except Exception as e:
        node.get_logger().error(f'发生错误: {str(e)}')
    finally:
        # 保存最终地图
        node.get_logger().info('保存地图...')
        node.save_map()
        node.get_logger().info(f'保存最终地图到 {node.map_save_path}')
        
        # 打印最终统计信息
        total_obstacle_cells = np.sum(node.grid_map == 1)
        node.get_logger().info(f'最终地图统计: 轨迹点={len(node.robot_trajectory)}, 障碍物单元格={total_obstacle_cells}')
        if len(node.robot_trajectory) > 0:
            node.get_logger().info(f'轨迹起点: ({node.robot_trajectory[0][0]:.2f}, {node.robot_trajectory[0][1]:.2f})')
            node.get_logger().info(f'轨迹终点: ({node.robot_trajectory[-1][0]:.2f}, {node.robot_trajectory[-1][1]:.2f})')
        
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 