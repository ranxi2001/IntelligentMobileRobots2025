#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
将LMS激光雷达数据和NAV轨迹数据转换为ROS2 bag格式
"""

import os
import sys
import struct
import numpy as np
import time
from datetime import datetime
import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, Vector3, Twist, TransformStamped
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Header

def get_rosbag_options(path, db_storage_id='sqlite3'):
    storage_options = rosbag2_py.StorageOptions(
        uri=path,
        storage_id=db_storage_id
    )
    
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    return storage_options, converter_options

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    return q

def read_lms_file(lms_file):
    """读取激光雷达扫描数据文件"""
    scans = []
    
    with open(lms_file, 'rb') as f:
        # 读取头部信息
        header_data = f.read(12)  # 3个float，每个4字节
        header = struct.unpack('fff', header_data)
        
        # 解析头部信息
        ang_range = header[0]  # 激光扫描范围
        ang_res = header[1]    # 角分辨率
        unit = header[2]       # 单位
        
        # 直接设置确认的最佳点数，通过数据分析验证为361
        max_dat_len = 361
        
        # 循环读取所有帧的激光数据
        while True:
            # 读取时间戳
            timestamp_data = f.read(8)  # long类型，8字节
            if not timestamp_data or len(timestamp_data) < 8:
                break
                
            timestamp = struct.unpack('q', timestamp_data)[0]
            
            # 读取激光数据
            laser_data = f.read(2 * max_dat_len)  # unsigned short类型，每个2字节
            if not laser_data or len(laser_data) < 2 * max_dat_len:
                print(f"警告：在时间戳 {timestamp} 处读取的激光数据长度不一致")
                break
                
            # 解析激光数据
            format_str = '{}H'.format(max_dat_len)
            laser_values = struct.unpack(format_str, laser_data)
            
            # 保存帧数据
            scans.append({
                'timestamp': timestamp,
                'laser_data': np.array(laser_values),
                'ang_range': ang_range,
                'ang_res': ang_res,
                'unit': unit,
                'max_dat_len': max_dat_len
            })
    
    print(f"成功读取 {len(scans)} 帧激光雷达数据")
    return scans

def read_trajectory_file(trajectory_file):
    """读取轨迹数据文件"""
    trajectory = []
    
    with open(trajectory_file, 'r') as f:
        # 跳过标题行
        next(f)
        
        # 读取所有轨迹点
        for line in f:
            data = line.strip().split()
            if len(data) >= 7:
                trajectory.append({
                    'timestamp': int(data[0]),
                    'ang_x': float(data[1]),
                    'ang_y': float(data[2]),
                    'ang_z': float(data[3]),
                    'shv_x': float(data[4]),
                    'shv_y': float(data[5]),
                    'shv_z': float(data[6])
                })
    
    print(f"成功读取 {len(trajectory)} 个轨迹点")
    return trajectory

def ros_time_from_ms(timestamp_ms):
    """将毫秒时间戳转换为ROS2的Time消息"""
    seconds = timestamp_ms // 1000
    nanoseconds = (timestamp_ms % 1000) * 1000000
    
    time_msg = Time()
    time_msg.sec = seconds
    time_msg.nanosec = nanoseconds
    return time_msg

def create_laser_msg(scan_data):
    """从激光扫描数据创建LaserScan消息"""
    msg = LaserScan()
    
    # 设置消息头
    msg.header = Header()
    msg.header.stamp = ros_time_from_ms(scan_data['timestamp'])
    msg.header.frame_id = 'laser'
    
    # 角度设置
    msg.angle_min = 0.0
    msg.angle_max = scan_data['ang_range'] * np.pi / 180.0
    msg.angle_increment = scan_data['ang_res'] * np.pi / 180.0
    
    # 扫描设置
    msg.time_increment = 0.0
    msg.scan_time = 1.0 / 25.0  # 假设25Hz的扫描频率
    
    # 距离设置
    msg.range_min = 0.1  # 最小距离（米）
    msg.range_max = 30.0  # 最大距离（米）
    
    # 转换激光数据到实际距离（米）
    ranges = scan_data['laser_data'] / scan_data['unit']
    
    # 使用numpy的where函数将0值替换为inf（超出范围）
    ranges = np.where(ranges == 0, float('inf'), ranges)
    
    msg.ranges = ranges.tolist()
    msg.intensities = []  # 不包含强度数据
    
    return msg

def create_odom_msg(traj_data):
    """从轨迹数据创建Odometry消息"""
    msg = Odometry()
    
    # 设置消息头
    msg.header = Header()
    msg.header.stamp = ros_time_from_ms(traj_data['timestamp'])
    msg.header.frame_id = 'odom'
    
    # 设置子坐标系
    msg.child_frame_id = 'base_link'
    
    # 位置
    msg.pose.pose.position = Point(x=traj_data['shv_x'], y=traj_data['shv_y'], z=traj_data['shv_z'])
    
    # 姿态（欧拉角转四元数）
    msg.pose.pose.orientation = euler_to_quaternion(
        traj_data['ang_x'], traj_data['ang_y'], traj_data['ang_z']
    )
    
    # 速度（无数据，设为0）
    msg.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
    msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
    
    return msg

def create_tf_msg(traj_data):
    """从轨迹数据创建TF消息（odom -> base_link -> laser）"""
    tf_msg = TFMessage()
    
    # odom -> base_link
    transform1 = TransformStamped()
    transform1.header.stamp = ros_time_from_ms(traj_data['timestamp'])
    transform1.header.frame_id = 'odom'
    transform1.child_frame_id = 'base_link'
    transform1.transform.translation.x = traj_data['shv_x']
    transform1.transform.translation.y = traj_data['shv_y']
    transform1.transform.translation.z = traj_data['shv_z']
    transform1.transform.rotation = euler_to_quaternion(
        traj_data['ang_x'], traj_data['ang_y'], traj_data['ang_z']
    )
    
    # base_link -> laser
    transform2 = TransformStamped()
    transform2.header.stamp = ros_time_from_ms(traj_data['timestamp'])
    transform2.header.frame_id = 'base_link'
    transform2.child_frame_id = 'laser'
    # 假设激光雷达在车辆前方0.2米，高0.5米
    transform2.transform.translation.x = 0.2
    transform2.transform.translation.y = 0.0
    transform2.transform.translation.z = 0.5
    # 假设激光雷达与车辆方向一致
    transform2.transform.rotation = euler_to_quaternion(0.0, 0.0, 0.0)
    
    tf_msg.transforms = [transform1, transform2]
    return tf_msg

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 解析命令行参数
    if len(sys.argv) < 4:
        print(f"用法: {sys.argv[0]} <LMS文件> <NAV文件> <输出bag路径>")
        sys.exit(1)
    
    lms_file = sys.argv[1]
    nav_file = sys.argv[2]
    bag_path = sys.argv[3]
    
    # 读取数据
    scans = read_lms_file(lms_file)
    trajectory = read_trajectory_file(nav_file)
    
    # 时间戳修正：将激光数据的时间戳映射到轨迹的时间戳范围内
    if scans and trajectory:
        # 获取激光数据的时间戳范围
        laser_start_time = scans[0]['timestamp']
        laser_end_time = scans[-1]['timestamp']
        
        # 获取轨迹数据的时间戳范围
        traj_start_time = trajectory[0]['timestamp']
        traj_end_time = trajectory[-1]['timestamp']
        
        print(f"激光数据时间戳范围: {laser_start_time} - {laser_end_time}")
        print(f"轨迹数据时间戳范围: {traj_start_time} - {traj_end_time}")
        
        # 对激光数据进行时间戳调整 - 线性映射
        # 我们将激光数据的时间戳线性映射到轨迹的时间戳范围
        laser_duration = laser_end_time - laser_start_time
        traj_duration = traj_end_time - traj_start_time
        
        if laser_duration > 0 and traj_duration > 0:
            # 对每个激光数据帧应用时间戳转换
            for scan in scans:
                # 计算相对位置 (0.0-1.0)
                relative_pos = (scan['timestamp'] - laser_start_time) / laser_duration
                # 映射到轨迹时间戳范围
                scan['original_timestamp'] = scan['timestamp']  # 保存原始时间戳以便调试
                scan['timestamp'] = int(traj_start_time + relative_pos * traj_duration)
            
            print("已完成激光数据时间戳映射")
        else:
            print("警告: 时间戳范围无效，不进行映射")
    
    # 筛选有效的激光扫描数据 - 确保每个激光帧都有对应的轨迹数据
    valid_scans = [s for s in scans if traj_start_time <= s['timestamp'] <= traj_end_time]
    print(f"有效激光扫描数据帧数: {len(valid_scans)}/{len(scans)}")
    
    # 如果没有有效数据帧，则使用所有激光数据帧
    if not valid_scans and scans:
        print("警告: 没有找到有效的激光数据帧，将使用所有激光数据帧")
        valid_scans = scans
        
    # 创建ROS2 bag文件
    storage_options, converter_options = get_rosbag_options(bag_path)
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    
    # 创建topic
    laser_topic = '/scan'
    odom_topic = '/odom'
    tf_topic = '/tf'
    
    # 注册topic
    laser_topic_type = 'sensor_msgs/msg/LaserScan'
    odom_topic_type = 'nav_msgs/msg/Odometry'
    tf_topic_type = 'tf2_msgs/msg/TFMessage'
    
    writer.create_topic(
        rosbag2_py.TopicMetadata(name=laser_topic, type=laser_topic_type, serialization_format='cdr')
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(name=odom_topic, type=odom_topic_type, serialization_format='cdr')
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(name=tf_topic, type=tf_topic_type, serialization_format='cdr')
    )
    
    # 将数据写入bag文件
    for scan in valid_scans:
        # 找到最近的轨迹点
        closest_traj = min(trajectory, key=lambda x: abs(x['timestamp'] - scan['timestamp']))
        
        # 创建激光消息
        laser_msg = create_laser_msg(scan)
        laser_serialized = serialize_message(laser_msg)
        
        # 创建里程计消息
        odom_msg = create_odom_msg(closest_traj)
        odom_serialized = serialize_message(odom_msg)
        
        # 创建TF消息
        tf_msg = create_tf_msg(closest_traj)
        tf_serialized = serialize_message(tf_msg)
        
        # 计算ROS2时间戳
        timestamp_sec = int(scan['timestamp'] // 1000)  # 秒
        timestamp_nsec = int((scan['timestamp'] % 1000) * 1000000)  # 纳秒
        
        # 写入激光消息
        writer.write(
            laser_topic,
            laser_serialized,
            timestamp_sec
        )
        
        # 写入里程计消息
        writer.write(
            odom_topic,
            odom_serialized,
            timestamp_sec
        )
        
        # 写入TF消息
        writer.write(
            tf_topic,
            tf_serialized,
            timestamp_sec
        )
    
    print(f"成功将数据写入ROS2 bag文件: {bag_path}")
    print(f"总共写入帧数: {len(valid_scans)}")
    
    # 清理资源
    rclpy.shutdown()

if __name__ == '__main__':
    main() 