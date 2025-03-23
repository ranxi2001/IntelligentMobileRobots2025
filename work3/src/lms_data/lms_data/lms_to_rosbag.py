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
    frame_count = 0
    
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
            # 读取时间戳 - 修正：使用4字节而不是8字节
            timestamp_data = f.read(4)  # long类型，4字节
            if not timestamp_data or len(timestamp_data) < 4:
                break
                
            # 使用小端序解析时间戳（按照作业二的数据格式）
            timestamp = struct.unpack('<l', timestamp_data)[0]
            
            # 将时间戳作为毫秒级时间戳使用
            # 根据作业二的数据分析，时间戳在71000000到72000000之间
            if timestamp < 0:
                print(f"警告：检测到负数时间戳 {timestamp}，使用帧计数生成时间戳")
                timestamp = 71400000 + frame_count * 50  # 每帧间隔约50ms
            elif timestamp < 70000000 or timestamp > 73000000:
                print(f"警告：检测到不合理的时间戳 {timestamp}，使用帧计数生成时间戳")
                timestamp = 71400000 + frame_count * 50
            
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
            
            frame_count += 1
    
    if len(scans) > 0:
        print(f"成功读取 {len(scans)} 帧激光雷达数据")
        print(f"时间戳范围: {scans[0]['timestamp']} - {scans[-1]['timestamp']}")
        print(f"平均时间间隔: {(scans[-1]['timestamp'] - scans[0]['timestamp']) / (len(scans)-1):.2f}ms")
    else:
        print("警告：未读取到任何激光数据")
    
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

def ros_time_from_ms(timestamp_ms, start_time=None):
    """将毫秒时间戳转换为ROS2的Time消息，使用相对时间"""
    # 简化时间计算逻辑，避免数值溢出
    if start_time is None:
        # 不使用相对时间，直接从0开始计时，避免大数值
        relative_ms = timestamp_ms % 10000000  # 限制在合理范围内
    else:
        # 计算相对时间，但限制在合理范围内
        relative_ms = (timestamp_ms - start_time) % 10000000
    
    # 确保时间值在有效范围内
    seconds = int(relative_ms / 1000)
    nanoseconds = int((relative_ms % 1000) * 1000000)
    
    time_msg = Time()
    time_msg.sec = seconds
    time_msg.nanosec = nanoseconds
    return time_msg

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
    
    # 检查命令行参数
    if len(sys.argv) != 4:
        print("用法: ros2 run lms_data lms_to_rosbag <lms_file> <nav_file> <bag_path>")
        sys.exit(1)
    
    lms_file = sys.argv[1]
    nav_file = sys.argv[2]
    bag_path = sys.argv[3]
    
    print(f"开始处理: LMS文件: {lms_file}, NAV文件: {nav_file}, BAG输出: {bag_path}")
    
    # 添加处理计时器
    start_time = time.time()
    
    try:
        # 读取数据
        print("读取LMS激光数据...")
        scans = read_lms_file(lms_file)
        
        print("读取NAV轨迹数据...")
        trajectory = read_trajectory_file(nav_file)
        
        if not scans or not trajectory:
            print("错误：无法读取数据文件")
            sys.exit(1)
        
        # 获取时间范围
        scan_times = [scan['timestamp'] for scan in scans]
        traj_times = [traj['timestamp'] for traj in trajectory]
        
        if not scan_times or not traj_times:
            print("错误：没有有效的时间戳数据")
            sys.exit(1)
        
        print(f"激光数据原始时间戳范围: {min(scan_times)} - {max(scan_times)}")
        print(f"轨迹数据时间戳范围: {min(traj_times)} - {max(traj_times)}")
        
        # 重要：使用轨迹数据的时间戳范围作为标准
        traj_start_time = min(traj_times)
        traj_end_time = max(traj_times)
        traj_time_range = traj_end_time - traj_start_time
        print(f"使用轨迹数据作为标准时间范围: {traj_start_time} - {traj_end_time}")
        
        # 激光数据范围
        scan_start_time = min(scan_times)
        scan_end_time = max(scan_times)
        scan_time_range = scan_end_time - scan_start_time
        
        if scan_time_range <= 0 or traj_time_range <= 0:
            print("错误：时间戳范围无效")
            sys.exit(1)
        
        # 创建更精确的时间戳映射函数
        print("创建时间戳映射...")
        def map_timestamp(t):
            # 计算激光数据在原始范围内的归一化位置 (0-1)
            normalized = (t - scan_start_time) / scan_time_range
            # 映射到轨迹时间范围内
            mapped_time = traj_start_time + normalized * traj_time_range
            # 确保时间戳在有效范围内
            mapped_time = max(traj_start_time, min(traj_end_time, mapped_time))
            return int(mapped_time)
        
        # 应用时间戳映射
        print("映射激光数据时间戳到轨迹时间范围...")
        for i, scan in enumerate(scans):
            original_ts = scan['timestamp']
            mapped_ts = map_timestamp(original_ts)
            scan['timestamp'] = mapped_ts
            scan['original_timestamp'] = original_ts  # 保留原始时间戳供参考
            
            # 打印进度
            if i % 100 == 0 or i == len(scans) - 1:
                print(f"处理进度: {i+1}/{len(scans)} ({(i+1)/len(scans)*100:.1f}%)")
                if i < 5 or i > len(scans) - 5:  # 打印前5个和后5个的映射情况
                    print(f"  样本映射: {original_ts} -> {mapped_ts}")
        
        print("已完成激光数据时间戳映射")
        mapped_scan_times = [scan['timestamp'] for scan in scans]
        print(f"映射后激光数据时间戳范围: {min(mapped_scan_times)} - {max(mapped_scan_times)}")
        
        # 验证映射质量
        in_range_count = sum(1 for t in mapped_scan_times if traj_start_time <= t <= traj_end_time)
        print(f"在轨迹时间范围内的激光帧数: {in_range_count}/{len(scans)} ({in_range_count/len(scans)*100:.1f}%)")
        
        # 过滤有效的激光扫描数据
        print("过滤有效的激光扫描数据...")
        valid_scans = []
        total_valid_points = 0
        # 简化过滤逻辑，只保留在轨迹时间范围内的激光帧
        for i, scan in enumerate(scans):
            # 判断该帧是否在轨迹时间戳范围内
            if traj_start_time <= scan['timestamp'] <= traj_end_time:
                try:
                    # 创建激光消息，简化过滤逻辑
                    laser_msg = LaserScan()
                    
                    # 设置消息头
                    laser_msg.header = Header()
                    laser_msg.header.stamp = ros_time_from_ms(scan['timestamp'])
                    laser_msg.header.frame_id = 'laser'
                    
                    # 角度设置
                    laser_msg.angle_min = 0.0
                    laser_msg.angle_max = scan['ang_range'] * np.pi / 180.0
                    laser_msg.angle_increment = scan['ang_res'] * np.pi / 180.0
                    
                    # 扫描设置
                    laser_msg.time_increment = 0.0
                    laser_msg.scan_time = 1.0 / 25.0  # 假设25Hz的扫描频率
                    
                    # 距离设置
                    laser_msg.range_min = 0.1  # 最小距离（米）
                    laser_msg.range_max = 30.0  # 最大距离（米）
                    
                    # 转换激光数据到实际距离（米）并应用简单过滤
                    ranges = scan['laser_data'] / scan['unit']
                    ranges = np.array(ranges)
                    
                    # 简单过滤：将小于最小范围或大于最大范围的值设为inf
                    ranges[ranges <= laser_msg.range_min] = float('inf')
                    ranges[ranges >= laser_msg.range_max] = float('inf')
                    ranges[ranges == 0] = float('inf')
                    
                    laser_msg.ranges = ranges.tolist()
                    laser_msg.intensities = []  # 不包含强度数据
                    
                    # 统计有效点数
                    valid_count = np.sum(~np.isinf(ranges))
                    
                    if valid_count > 0:  # 只保留有效点数大于0的帧
                        valid_scans.append((scan, laser_msg, valid_count))
                        total_valid_points += valid_count
                    
                    # 打印进度
                    if i % 100 == 0 or i == len(scans) - 1:
                        percent = (i+1) / len(scans) * 100
                        print(f"过滤进度: {i+1}/{len(scans)} ({percent:.1f}%), 当前有效帧: {len(valid_scans)}")
                except Exception as e:
                    print(f"警告: 处理第{i}帧时出错: {e}")
                    continue
        
        print(f"有效激光扫描数据帧数: {len(valid_scans)}/{len(scans)}")
        print(f"在轨迹时间范围内 ({traj_start_time} - {traj_end_time}) 的有效帧占总帧数的 {len(valid_scans)/len(scans)*100:.1f}%")
        
        if len(valid_scans) == 0:
            print("错误: 没有有效的激光扫描数据帧，无法创建rosbag")
            sys.exit(1)
        
        # 对轨迹点进行插值，确保每个激光帧都有对应的轨迹点
        print("创建插值轨迹点...")
        scan_timestamps = sorted([scan[0]['timestamp'] for scan in valid_scans])
        traj_timestamps = sorted([t['timestamp'] for t in trajectory])
        
        # 创建完整的轨迹点列表（包含插值点）
        full_trajectory = []
        for i, t in enumerate(scan_timestamps):
            # 找到最近的两个轨迹点
            idx = np.searchsorted(traj_timestamps, t)
            if idx == 0:
                full_trajectory.append(trajectory[0])
            elif idx >= len(trajectory):
                full_trajectory.append(trajectory[-1])
            else:
                # 线性插值
                t1, t2 = trajectory[idx-1], trajectory[idx]
                # 避免除零错误
                time_diff = t2['timestamp'] - t1['timestamp']
                if time_diff > 0:
                    alpha = (t - t1['timestamp']) / time_diff
                else:
                    alpha = 0.5  # 如果时间戳相同，使用中点
                interp_traj = {
                    'timestamp': t,
                    'ang_x': t1['ang_x'] + alpha * (t2['ang_x'] - t1['ang_x']),
                    'ang_y': t1['ang_y'] + alpha * (t2['ang_y'] - t1['ang_y']),
                    'ang_z': t1['ang_z'] + alpha * (t2['ang_z'] - t1['ang_z']),
                    'shv_x': t1['shv_x'] + alpha * (t2['shv_x'] - t1['shv_x']),
                    'shv_y': t1['shv_y'] + alpha * (t2['shv_y'] - t1['shv_y']),
                    'shv_z': t1['shv_z'] + alpha * (t2['shv_z'] - t1['shv_z'])
                }
                full_trajectory.append(interp_traj)
            
            # 打印进度
            if i % 100 == 0 or i == len(scan_timestamps) - 1:
                percent = (i+1) / len(scan_timestamps) * 100
                print(f"插值进度: {i+1}/{len(scan_timestamps)} ({percent:.1f}%)")
        
        print(f"扩充轨迹点: 原始={len(trajectory)}, 扩充后={len(full_trajectory)}")
        
        # 检查时间戳匹配
        scan_times = set([scan[0]['timestamp'] for scan in valid_scans])
        traj_times = set([t['timestamp'] for t in full_trajectory])
        matching_times = scan_times.intersection(traj_times)
        print(f"时间戳完全匹配的激光帧数: {len(matching_times)}/{len(valid_scans)} ({len(matching_times)/len(valid_scans)*100:.1f}%)")
        
        # 创建rosbag
        print(f"创建rosbag文件: {bag_path}")
        storage_options, converter_options = get_rosbag_options(bag_path)
        writer = rosbag2_py.SequentialWriter()
        
        try:
            writer.open(storage_options, converter_options)
            
            # 创建话题
            topic_types = [
                ('tf', 'tf2_msgs/msg/TFMessage'),
                ('odom', 'nav_msgs/msg/Odometry'),
                ('scan', 'sensor_msgs/msg/LaserScan'),
                ('/clock', 'rosgraph_msgs/msg/Clock')  # 修改：添加前导斜杠，确保绝对话题名称
            ]
            
            for topic, msg_type in topic_types:
                topic_info = rosbag2_py.TopicMetadata(
                    name=topic,
                    type=msg_type,
                    serialization_format='cdr'
                )
                writer.create_topic(topic_info)
            
            # 使用从轨迹开始时间为基准的相对时间戳
            print("生成rosbag时间戳...")
            # 轨迹起始时间的纳秒表示（用于ROS2时间）
            traj_start_time_ns = traj_start_time * 1000000  # 毫秒转纳秒
            
            # 创建一个起始偏移，确保所有时间戳为正值
            time_offset_ns = 1000000000  # 1秒的纳秒数
            
            print("写入所有轨迹点...")
            # 写入轨迹和TF数据
            for i, traj_data in enumerate(full_trajectory):
                try:
                    # 计算消息时间戳（相对于轨迹起始时间）
                    relative_time_ms = traj_data['timestamp'] - traj_start_time
                    timestamp_ns = time_offset_ns + relative_time_ms * 1000000  # 毫秒转纳秒
                    
                    # 创建并写入Odometry消息
                    odom_msg = create_odom_msg(traj_data)
                    # 使用与轨迹时间戳对应的时间
                    odom_msg.header.stamp.sec = int(timestamp_ns / 1000000000)
                    odom_msg.header.stamp.nanosec = int(timestamp_ns % 1000000000)
                    writer.write(
                        'odom',
                        serialize_message(odom_msg),
                        timestamp_ns
                    )
                    
                    # 创建并写入TF消息
                    tf_msg = create_tf_msg(traj_data)
                    # 更新TF消息的时间戳
                    for transform in tf_msg.transforms:
                        transform.header.stamp.sec = int(timestamp_ns / 1000000000)
                        transform.header.stamp.nanosec = int(timestamp_ns % 1000000000)
                    writer.write(
                        'tf',
                        serialize_message(tf_msg),
                        timestamp_ns
                    )
                    
                    # 添加时钟消息，确保时间同步
                    from rosgraph_msgs.msg import Clock
                    clock_msg = Clock()
                    clock_msg.clock.sec = int(timestamp_ns / 1000000000)
                    clock_msg.clock.nanosec = int(timestamp_ns % 1000000000)
                    writer.write(
                        '/clock',
                        serialize_message(clock_msg),
                        timestamp_ns
                    )
                    
                    # 打印进度
                    if i % 100 == 0 or i == len(full_trajectory) - 1:
                        percent = (i+1) / len(full_trajectory) * 100
                        print(f"轨迹写入进度: {i+1}/{len(full_trajectory)} ({percent:.1f}%)")
                except Exception as e:
                    print(f"警告: 写入轨迹点{i}时出错: {e}")
                    continue
            
            print("写入所有激光帧...")
            # 写入激光数据
            for i, (scan_data, laser_msg, valid_count) in enumerate(valid_scans):
                try:
                    # 计算相对时间戳
                    relative_time_ms = scan_data['timestamp'] - traj_start_time
                    timestamp_ns = time_offset_ns + relative_time_ms * 1000000  # 毫秒转纳秒
                    
                    # 更新激光消息的时间戳
                    laser_msg.header.stamp.sec = int(timestamp_ns / 1000000000)
                    laser_msg.header.stamp.nanosec = int(timestamp_ns % 1000000000)
                    
                    writer.write(
                        'scan',
                        serialize_message(laser_msg),
                        timestamp_ns
                    )
                    
                    # 打印进度
                    if i % 100 == 0 or i == len(valid_scans) - 1:
                        percent = (i+1) / len(valid_scans) * 100
                        print(f"激光帧写入进度: {i+1}/{len(valid_scans)} ({percent:.1f}%)")
                except Exception as e:
                    print(f"警告: 写入激光帧{i}时出错: {e}")
                    continue
            
            end_time = time.time()
            elapsed = end_time - start_time
            
            print(f"成功将数据写入ROS2 bag文件: {bag_path}")
            print(f"总共写入帧数: {len(valid_scans)}")
            print(f"总有效激光点数: {total_valid_points}, 平均每帧: {total_valid_points/len(valid_scans):.1f}")
            print(f"使用的轨迹时间戳范围: {traj_start_time} - {traj_end_time}")
            print(f"处理用时: {elapsed:.2f}秒")
            
            # 修正：不再调用close方法，ROS2 Humble中的SequentialWriter没有close方法
            # 让Python的垃圾回收机制自动清理
            print("ROS2 Bag文件写入完成")
            
        except Exception as e:
            print(f"❌ 错误: 创建或写入rosbag时出错: {e}")
            # 不再尝试调用close方法
            sys.exit(1)
        
    except Exception as e:
        print(f"❌ 致命错误: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        # 确保关闭ROS2
        rclpy.shutdown()

if __name__ == '__main__':
    main() 