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
    """ROS2激光雷达占用栅格地图生成节点"""
    
    def __init__(self):
        super().__init__('lms_mapper_node')
        
        # 声明参数
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_size_x', 1000)
        self.declare_parameter('map_size_y', 1000)
        self.declare_parameter('map_update_rate', 5.0)  # 地图更新频率 (Hz)
        self.declare_parameter('map_save_path', 'occupancy_grid.png')
        
        # 获取参数
        resolution = self.get_parameter('map_resolution').value
        size_x = self.get_parameter('map_size_x').value
        size_y = self.get_parameter('map_size_y').value
        self.update_rate = self.get_parameter('map_update_rate').value
        self.map_save_path = self.get_parameter('map_save_path').value
        
        # 初始化占用栅格地图
        self.mapper = OccupancyGridMapper(resolution, size_x, size_y)
        
        # 初始化订阅者
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        # 初始化发布者
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10)
            
        # 初始化定时器，定期发布地图
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_map)
        
        # 存储数据
        self.latest_scan = None
        self.latest_pose = None
        self.robot_trajectory = []
        
        # 同步锁，防止数据竞争
        self.lock = threading.Lock()
        
        # 记录开始时间
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('LMS mapper node已启动')
        
    def odom_callback(self, msg):
        """处理里程计消息"""
        with self.lock:
            self.latest_pose = msg.pose.pose
            
            # 记录轨迹点
            position = msg.pose.pose.position
            self.robot_trajectory.append((position.x, position.y))
    
    def scan_callback(self, msg):
        """处理激光扫描消息"""
        with self.lock:
            self.latest_scan = msg
    
    def publish_map(self):
        """定期发布地图并更新"""
        with self.lock:
            # 检查是否有数据可用
            if self.latest_scan is None or self.latest_pose is None:
                return
                
            # 提取激光数据
            scan = self.latest_scan
            ranges = np.array(scan.ranges)
            angle_min = scan.angle_min
            angle_increment = scan.angle_increment
            angles = np.arange(len(ranges)) * angle_increment + angle_min
            
            # 转换激光点到世界坐标系
            world_points = laser_to_world(self.latest_pose, ranges, angles)
            
            # 更新地图
            self.mapper.update_map(self.latest_pose, world_points)
            
            # 发布地图
            map_msg = self.mapper.get_ros_occupancy_grid()
            self.map_pub.publish(map_msg)
            
            # 检查是否需要保存地图
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if int(elapsed) % 10 == 0:  # 每10秒保存一次地图
                self.mapper.visualize_map(self.robot_trajectory, self.map_save_path)


def main(args=None):
    rclpy.init(args=args)
    
    node = LmsMapperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 保存最终地图
        node.mapper.visualize_map(node.robot_trajectory, node.map_save_path)
        node.get_logger().info(f'保存最终地图到 {node.map_save_path}')
        
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 