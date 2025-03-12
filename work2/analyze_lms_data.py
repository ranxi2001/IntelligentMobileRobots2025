#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LMS激光雷达数据文件分析工具
用于检测和分析二进制LMS文件中的异常数据
"""

import numpy as np
import matplotlib.pyplot as plt
import struct
import os
import time
from collections import defaultdict

class LMSDataAnalyzer:
    """LMS激光雷达数据分析类"""
    def __init__(self, lms_file):
        self.lms_file = lms_file
        self.header = None
        self.file_size = 0
        self.scans = []
        self.point_count = None
        self.frames_count = 0
        self.timestamp_diffs = []
        self.distance_stats = {}
        
    def analyze_file_structure(self):
        """分析文件结构，尝试确定实际的激光点数"""
        with open(self.lms_file, 'rb') as f:
            # 获取文件总大小
            f.seek(0, 2)
            self.file_size = f.tell()
            f.seek(0, 0)
            
            # 读取头部信息 (3个float：角度范围、角分辨率、单位)
            header_size = 12  # 3 个 float (4字节 * 3)
            header_data = f.read(header_size)
            self.header = struct.unpack('fff', header_data)
            
            ang_range = self.header[0]  # 激光扫描范围
            ang_res = self.header[1]    # 角分辨率
            unit = self.header[2]       # 单位
            
            # 计算理论上的激光点数
            theoretical_points = int(ang_range / ang_res) + 1
            
            print(f"=== 文件基本信息 ===")
            print(f"文件大小: {self.file_size} 字节")
            print(f"头部信息:")
            print(f"  - 角度范围: {ang_range} 度")
            print(f"  - 角分辨率: {ang_res} 度")
            print(f"  - 单位: {unit}")
            print(f"理论激光点数: {theoretical_points}")
            
            # 尝试各种可能的点数配置
            remaining_size = self.file_size - header_size
            possible_configs = []
            
            # 测试点数范围 (从理论点数附近开始探索)
            min_test = max(1, theoretical_points - 10)
            max_test = theoretical_points + 10
            
            for points in range(min_test, max_test + 1):
                # 每一帧的大小 = 时间戳(4字节) + 点数据(每点2字节)
                frame_size = 4 + 2 * points
                
                # 检查剩余文件大小是否能被帧大小整除
                if remaining_size % frame_size == 0:
                    frames = remaining_size // frame_size
                    possible_configs.append((points, frames, frame_size))
                    print(f"可能的配置: {points}点/帧 - {frames}帧 - 每帧{frame_size}字节")
            
            # 如果没有找到可能的配置，尝试更广的范围
            if not possible_configs:
                print("在初始范围内未找到有效配置，尝试更广的范围...")
                
                # 尝试其他可能的点数配置
                additional_tests = [
                    int(360 / ang_res) + 1,  # 全圆扫描
                    int(360 / ang_res),      # 全圆扫描（不加1）
                    int(270 / ang_res) + 1,  # 270度扫描
                    int(180 / ang_res) + 1,  # 180度扫描
                ]
                
                for points in additional_tests:
                    frame_size = 4 + 2 * points
                    if remaining_size % frame_size == 0:
                        frames = remaining_size // frame_size
                        possible_configs.append((points, frames, frame_size))
                        print(f"可能的配置: {points}点/帧 - {frames}帧 - 每帧{frame_size}字节")
            
            # 如果仍没有找到配置，尝试暴力搜索合理范围内的所有可能
            if not possible_configs:
                print("标准配置未找到，尝试广范围搜索...")
                for points in range(100, 2000):  # 合理范围内的激光点数
                    frame_size = 4 + 2 * points
                    if remaining_size % frame_size == 0:
                        frames = remaining_size // frame_size
                        if frames > 10:  # 至少应该有几十帧数据
                            possible_configs.append((points, frames, frame_size))
                            print(f"可能的配置: {points}点/帧 - {frames}帧 - 每帧{frame_size}字节")
                
                # 限制输出过多的配置
                if len(possible_configs) > 10:
                    print(f"找到了{len(possible_configs)}种可能的配置，仅显示前10种")
                    possible_configs = possible_configs[:10]
            
            # 如果找到了可能的配置，选择帧数最多的（最可能是正确的）
            if possible_configs:
                possible_configs.sort(key=lambda x: x[1], reverse=True)
                self.point_count = possible_configs[0][0]
                self.frames_count = possible_configs[0][1]
                print(f"\n选择最可能的配置: {self.point_count}点/帧，共{self.frames_count}帧")
                return self.point_count
            else:
                print("警告: 无法确定有效的点数配置！")
                return theoretical_points  # 退回到理论值
    
    def read_data(self):
        """使用确定的点数读取所有数据帧"""
        if not self.point_count:
            self.analyze_file_structure()
        
        print(f"\n=== 读取数据帧 ===")
        with open(self.lms_file, 'rb') as f:
            # 跳过头部
            f.seek(12, 0)
            
            # 逐帧读取数据
            frame_count = 0
            last_timestamp = None
            invalid_frames = 0
            distance_values = []
            
            while True:
                frame_start_pos = f.tell()
                
                # 读取时间戳
                timestamp_data = f.read(4)
                if not timestamp_data or len(timestamp_data) < 4:
                    break
                
                timestamp = struct.unpack('l', timestamp_data)[0]
                
                # 记录时间戳差异
                if last_timestamp is not None:
                    time_diff = timestamp - last_timestamp
                    self.timestamp_diffs.append(time_diff)
                last_timestamp = timestamp
                
                # 读取激光点数据
                laser_data = f.read(2 * self.point_count)
                if not laser_data or len(laser_data) < 2 * self.point_count:
                    print(f"  警告: 帧 #{frame_count+1} 数据不完整，已读取 {len(laser_data)} 字节")
                    invalid_frames += 1
                    break
                
                # 解析激光点数据
                try:
                    format_str = '{}H'.format(self.point_count)
                    distances = struct.unpack(format_str, laser_data)
                    
                    # 保存帧数据
                    self.scans.append({
                        'timestamp': timestamp,
                        'distances': np.array(distances),
                        'file_position': frame_start_pos
                    })
                    
                    # 收集距离值用于统计
                    distance_values.extend(distances)
                    
                    frame_count += 1
                    
                    # 显示进度
                    if frame_count % 1000 == 0:
                        print(f"  已读取 {frame_count} 帧...")
                        
                except struct.error as e:
                    print(f"  错误: 帧 #{frame_count+1} 解析失败: {str(e)}")
                    invalid_frames += 1
                    break
            
            print(f"读取完成: 共 {frame_count} 帧，{invalid_frames} 帧无效")
            
            # 将距离值转换为numpy数组，便于统计分析
            distance_values = np.array(distance_values)
            
            # 计算基本统计信息
            self.distance_stats = {
                'min': np.min(distance_values),
                'max': np.max(distance_values),
                'mean': np.mean(distance_values),
                'median': np.median(distance_values),
                'std': np.std(distance_values),
                'zero_count': np.sum(distance_values == 0),
                'max_count': np.sum(distance_values == 65535)  # 最大值通常表示无效测量
            }
            
            return frame_count
    
    def analyze_timestamps(self):
        """分析时间戳的连续性和异常"""
        if not self.timestamp_diffs:
            print("没有时间戳差异数据可供分析")
            return
        
        print(f"\n=== 时间戳分析 ===")
        diffs = np.array(self.timestamp_diffs)
        
        # 基本统计
        mean_diff = np.mean(diffs)
        median_diff = np.median(diffs)
        std_diff = np.std(diffs)
        min_diff = np.min(diffs)
        max_diff = np.max(diffs)
        
        print(f"时间戳差异统计:")
        print(f"  - 平均值: {mean_diff:.2f}")
        print(f"  - 中位数: {median_diff:.2f}")
        print(f"  - 标准差: {std_diff:.2f}")
        print(f"  - 最小值: {min_diff:.2f}")
        print(f"  - 最大值: {max_diff:.2f}")
        
        # 检测异常时间戳
        # 定义异常：与中位数差距超过3个标准差
        anomaly_threshold = 3 * std_diff
        anomalies = np.where(np.abs(diffs - median_diff) > anomaly_threshold)[0]
        
        if len(anomalies) > 0:
            print(f"检测到 {len(anomalies)} 个异常时间戳差异:")
            for i, idx in enumerate(anomalies[:10]):  # 只显示前10个
                print(f"  #{idx+1}: {diffs[idx]:.2f} (帧 {idx+1} 和 {idx+2} 之间)")
            
            if len(anomalies) > 10:
                print(f"  ... 还有 {len(anomalies)-10} 个异常未显示")
        else:
            print("未检测到明显的时间戳异常")
        
        # 检测静止区域（时间戳差异较小的连续区域）
        static_threshold = mean_diff * 0.5
        static_regions = []
        i = 0
        while i < len(diffs):
            if diffs[i] < static_threshold:
                start = i
                while i < len(diffs) and diffs[i] < static_threshold:
                    i += 1
                end = i
                if end - start > 5:  # 至少5帧才认为是静止区域
                    static_regions.append((start, end))
            else:
                i += 1
        
        if static_regions:
            print(f"检测到 {len(static_regions)} 个可能的静止区域:")
            for i, (start, end) in enumerate(static_regions[:5]):  # 只显示前5个
                print(f"  区域 #{i+1}: 帧 {start+1} 到 {end+1} (共 {end-start+1} 帧)")
            
            if len(static_regions) > 5:
                print(f"  ... 还有 {len(static_regions)-5} 个区域未显示")
        else:
            print("未检测到明显的静止区域")
    
    def analyze_distances(self):
        """分析距离数据的分布和异常"""
        if not self.scans:
            print("没有扫描数据可供分析")
            return
        
        print(f"\n=== 距离数据分析 ===")
        
        # 显示统计信息
        print(f"距离值统计:")
        print(f"  - 最小值: {self.distance_stats['min']}")
        print(f"  - 最大值: {self.distance_stats['max']}")
        print(f"  - 平均值: {self.distance_stats['mean']:.2f}")
        print(f"  - 中位数: {self.distance_stats['median']:.2f}")
        print(f"  - 标准差: {self.distance_stats['std']:.2f}")
        print(f"  - 零值数量: {self.distance_stats['zero_count']} ({self.distance_stats['zero_count']/len(self.scans)/self.point_count*100:.2f}%)")
        print(f"  - 最大值数量: {self.distance_stats['max_count']} ({self.distance_stats['max_count']/len(self.scans)/self.point_count*100:.2f}%)")
        
        # 分析每个角度位置的数据
        angle_stats = []
        for angle_idx in range(self.point_count):
            angle_values = [scan['distances'][angle_idx] for scan in self.scans]
            angle_values = np.array(angle_values)
            
            # 计算该角度的统计数据
            angle_stats.append({
                'min': np.min(angle_values),
                'max': np.max(angle_values),
                'mean': np.mean(angle_values),
                'median': np.median(angle_values),
                'std': np.std(angle_values),
                'cv': np.std(angle_values) / np.mean(angle_values) if np.mean(angle_values) > 0 else 0,
                'zero_count': np.sum(angle_values == 0),
                'max_count': np.sum(angle_values == 65535)
            })
        
        # 找出异常角度（变异系数高的角度）
        cv_threshold = 0.5  # 变异系数阈值
        anomaly_angles = []
        for i, stats in enumerate(angle_stats):
            if stats['cv'] > cv_threshold:
                anomaly_angles.append((i, stats['cv']))
        
        anomaly_angles.sort(key=lambda x: x[1], reverse=True)
        
        if anomaly_angles:
            print(f"\n检测到 {len(anomaly_angles)} 个可能存在问题的角度:")
            for i, (angle_idx, cv) in enumerate(anomaly_angles[:10]):  # 只显示前10个
                ang_res = self.header[1]  # 角分辨率
                angle_deg = angle_idx * ang_res
                print(f"  角度 #{angle_idx} ({angle_deg:.1f}°): 变异系数 = {cv:.2f}, "
                      f"零值率 = {angle_stats[angle_idx]['zero_count']/len(self.scans)*100:.2f}%, "
                      f"最大值率 = {angle_stats[angle_idx]['max_count']/len(self.scans)*100:.2f}%")
            
            if len(anomaly_angles) > 10:
                print(f"  ... 还有 {len(anomaly_angles)-10} 个异常角度未显示")
        else:
            print("未检测到明显的角度异常")
    
    def visualize_data(self, output_dir=None):
        """可视化分析结果"""
        if not self.scans or not self.timestamp_diffs:
            print("没有足够的数据用于可视化")
            return
        
        print(f"\n=== 生成可视化结果 ===")
        
        # 创建输出目录
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        
        # 1. 时间戳差异图
        plt.figure(figsize=(12, 6))
        plt.plot(self.timestamp_diffs)
        plt.title('激光雷达扫描帧之间的时间戳差异')
        plt.xlabel('帧索引')
        plt.ylabel('时间戳差异')
        plt.grid(True)
        if output_dir:
            plt.savefig(os.path.join(output_dir, 'timestamp_diffs.png'), dpi=150)
            plt.close()
        else:
            plt.show()
        
        # 2. 时间戳差异直方图
        plt.figure(figsize=(12, 6))
        plt.hist(self.timestamp_diffs, bins=50)
        plt.title('时间戳差异分布')
        plt.xlabel('时间戳差异')
        plt.ylabel('频率')
        plt.grid(True)
        if output_dir:
            plt.savefig(os.path.join(output_dir, 'timestamp_hist.png'), dpi=150)
            plt.close()
        else:
            plt.show()
        
        # 3. 角度-距离异常分析
        angle_values = []
        for angle_idx in range(self.point_count):
            values = [scan['distances'][angle_idx] for scan in self.scans]
            values = np.array(values)
            
            # 过滤掉无效值
            valid_values = values[(values > 0) & (values < 65535)]
            if len(valid_values) > 0:
                cv = np.std(valid_values) / np.mean(valid_values) if np.mean(valid_values) > 0 else 0
                angle_values.append((angle_idx, cv, len(values) - len(valid_values)))
        
        # 按变异系数排序
        angle_values.sort(key=lambda x: x[1], reverse=True)
        
        # 绘制角度变异系数图
        plt.figure(figsize=(12, 6))
        angles = [x[0] * self.header[1] for x in angle_values]  # 转换为角度
        cvs = [x[1] for x in angle_values]
        plt.bar(angles, cvs, width=self.header[1]*0.8)
        plt.title('各角度激光点的变异系数')
        plt.xlabel('角度 (度)')
        plt.ylabel('变异系数')
        plt.grid(True)
        if output_dir:
            plt.savefig(os.path.join(output_dir, 'angle_cv.png'), dpi=150)
            plt.close()
        else:
            plt.show()
        
        # 4. 无效点比例图
        plt.figure(figsize=(12, 6))
        angles = [x[0] * self.header[1] for x in angle_values]
        invalid_rates = [x[2] / len(self.scans) for x in angle_values]
        plt.bar(angles, invalid_rates, width=self.header[1]*0.8)
        plt.title('各角度激光点的无效比例')
        plt.xlabel('角度 (度)')
        plt.ylabel('无效点比例')
        plt.grid(True)
        if output_dir:
            plt.savefig(os.path.join(output_dir, 'angle_invalid.png'), dpi=150)
            plt.close()
        else:
            plt.show()
        
        # 5. 可视化几个典型帧
        # 选择几个均匀分布的帧进行可视化
        frame_indices = [0, len(self.scans)//4, len(self.scans)//2, 3*len(self.scans)//4, len(self.scans)-1]
        
        plt.figure(figsize=(15, 12))
        for i, idx in enumerate(frame_indices):
            plt.subplot(2, 3, i+1)
            
            scan = self.scans[idx]
            distances = scan['distances']
            angles = np.arange(self.point_count) * self.header[1] * np.pi / 180.0
            
            # 过滤掉无效值
            valid_mask = (distances > 0) & (distances < 65535)
            valid_distances = distances[valid_mask] / self.header[2]  # 转换为实际距离单位
            valid_angles = angles[valid_mask]
            
            # 转换为笛卡尔坐标
            x = valid_distances * np.cos(valid_angles)
            y = valid_distances * np.sin(valid_angles)
            
            plt.scatter(x, y, s=10, alpha=0.7)
            plt.title(f'帧 #{idx+1} (时间戳: {scan["timestamp"]})')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.grid(True)
            plt.axis('equal')
        
        plt.tight_layout()
        if output_dir:
            plt.savefig(os.path.join(output_dir, 'sample_frames.png'), dpi=150)
            plt.close()
        else:
            plt.show()
        
        print(f"可视化结果{'已保存到 '+output_dir if output_dir else '已显示'}")
        
    def detect_dynamic_point_count(self):
        """
        尝试动态检测每帧的实际激光点数
        这种方法适用于不同帧可能有不同点数的情况
        """
        print(f"\n=== 尝试检测可变的激光点数 ===")
        
        with open(self.lms_file, 'rb') as f:
            # 跳过头部
            f.seek(12, 0)
            remaining_size = self.file_size - 12
            
            # 存储每个位置的时间戳
            timestamps = []
            positions = []
            
            # 初始位置
            pos = 12
            
            # 寻找时间戳
            while pos + 4 <= self.file_size:
                f.seek(pos, 0)
                timestamp_data = f.read(4)
                if len(timestamp_data) < 4:
                    break
                    
                timestamp = struct.unpack('l', timestamp_data)[0]
                
                # 只存储合理范围内的时间戳
                if timestamp > 0 and timestamp < 10000000000:  # 合理的时间戳范围
                    timestamps.append(timestamp)
                    positions.append(pos)
                
                # 移动到下一个可能的时间戳位置 (尝试2字节递增)
                pos += 2
            
            # 分析连续时间戳之间的差异
            if len(timestamps) < 2:
                print("未找到足够的时间戳来分析")
                return False
            
            # 计算时间戳差异
            diffs = np.diff(timestamps)
            
            # 查找频繁出现的差异值（可能代表正常的扫描频率）
            from collections import Counter
            diff_counter = Counter(diffs)
            common_diffs = diff_counter.most_common(5)
            
            print(f"最常见的时间戳差异值:")
            for diff, count in common_diffs:
                print(f"  差异值 {diff}: 出现 {count} 次")
            
            # 根据常见的时间戳差异，尝试识别真正的帧结构
            if common_diffs:
                most_common_diff = common_diffs[0][0]
                print(f"最常见的时间戳差异: {most_common_diff}")
                
                # 寻找符合这个差异的连续时间戳
                consistent_frames = []
                for i in range(len(timestamps) - 1):
                    if abs(timestamps[i+1] - timestamps[i] - most_common_diff) < most_common_diff * 0.1:  # 允许10%的误差
                        consistent_frames.append((positions[i], positions[i+1]))
                
                if consistent_frames:
                    print(f"找到 {len(consistent_frames)} 对可能连续的帧")
                    
                    # 计算这些帧之间的间隔
                    frame_sizes = [end - start for start, end in consistent_frames]
                    average_size = np.mean(frame_sizes)
                    
                    # 推断可能的点数
                    possible_points = int((average_size - 4) / 2)  # 减去时间戳的4字节，每点2字节
                    
                    print(f"基于时间戳分析，推断每帧可能包含 {possible_points} 个激光点")
                    print(f"平均帧大小: {average_size:.2f} 字节")
                    
                    # 验证这个点数是否合理
                    theoretical_points = int(self.header[0] / self.header[1]) + 1
                    if abs(possible_points - theoretical_points) > theoretical_points * 0.5:
                        print(f"警告: 推断的点数 ({possible_points}) 与理论点数 ({theoretical_points}) 差异较大")
                    
                    return possible_points
                else:
                    print("未找到符合常见时间戳差异的连续帧")
            else:
                print("未找到常见的时间戳差异模式")
                
            return False
    
    def run_full_analysis(self, output_dir=None):
        """运行完整的分析流程"""
        print(f"开始分析LMS激光雷达数据文件: {self.lms_file}")
        
        # 1. 分析文件结构
        self.analyze_file_structure()
        
        # 2. 尝试动态检测点数
        dynamic_point_count = self.detect_dynamic_point_count()
        if dynamic_point_count and dynamic_point_count != self.point_count:
            print(f"发现可能的动态点数: {dynamic_point_count}，与初始分析结果不同")
            use_dynamic = input("是否使用动态检测的点数？(y/n): ").strip().lower() == 'y'
            if use_dynamic:
                self.point_count = dynamic_point_count
                print(f"使用动态检测的点数: {self.point_count}")
        
        # 3. 读取数据
        self.read_data()
        
        # 4. 分析时间戳
        self.analyze_timestamps()
        
        # 5. 分析距离数据
        self.analyze_distances()
        
        # 6. 可视化结果
        self.visualize_data(output_dir)
        
        print(f"分析完成！")
        
        return {
            'file_size': self.file_size,
            'header': self.header,
            'point_count': self.point_count,
            'frames_count': len(self.scans),
            'distance_stats': self.distance_stats
        }


def main():
    print("=== LMS激光雷达数据文件分析工具 ===")
    
    # 设置默认文件路径
    default_lms_file = "./data/URG_X_20130903_195003.lms"
    default_output_dir = "./analysis_results"
    
    # 获取用户输入
    print(f"默认激光雷达数据文件: {default_lms_file}")
    lms_file = input(f"请输入激光雷达数据文件路径 (直接回车使用默认值): ").strip()
    if not lms_file:
        lms_file = default_lms_file
    
    # 检查文件是否存在
    if not os.path.exists(lms_file):
        print(f"错误: 文件 {lms_file} 不存在")
        return
    
    # 询问是否需要保存可视化结果
    save_viz = input("是否保存可视化结果? (y/n, 默认y): ").strip().lower() != 'n'
    output_dir = None
    if save_viz:
        print(f"默认输出目录: {default_output_dir}")
        output_dir = input(f"请输入输出目录路径 (直接回车使用默认值): ").strip()
        if not output_dir:
            output_dir = default_output_dir
    
    # 创建分析器并运行分析
    analyzer = LMSDataAnalyzer(lms_file)
    results = analyzer.run_full_analysis(output_dir)
    
    # 显示分析结果摘要
    print("\n=== 分析结果摘要 ===")
    print(f"文件大小: {results['file_size']} 字节")
    print(f"实际激光点数: {results['point_count']} 点/帧")
    print(f"总帧数: {results['frames_count']} 帧")
    print(f"平均距离值: {results['distance_stats']['mean']:.2f}")
    print(f"距离标准差: {results['distance_stats']['std']:.2f}")
    
    # 结论提示
    if save_viz and output_dir:
        print(f"\n分析完成! 可视化结果已保存到 {output_dir}")
    else:
        print("\n分析完成!")


if __name__ == "__main__":
    main() 