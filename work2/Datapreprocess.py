import struct
import numpy as np
import matplotlib.pyplot as plt
import os
import time
import datetime

class LaserDataPreprocessor:
    """激光数据预处理类"""
    def __init__(self, lms_file, log_file=None):
        self.lms_file = lms_file
        self.header = None
        self.scans = []
        self.file_size = 0
        self.data_start_pos = 0
        
        # 设置日志文件
        if log_file is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file = f"laser_data_analysis_{timestamp}.log"
        else:
            self.log_file = log_file
        
        # 创建日志文件
        with open(self.log_file, 'w', encoding='utf-8') as f:
            f.write(f"===== 激光雷达数据分析日志 =====\n")
            f.write(f"开始时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"数据文件: {os.path.abspath(lms_file)}\n\n")
    
    def log(self, message):
        """将消息写入日志文件并打印到控制台"""
        print(message)
        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write(message + "\n")

    def read_header(self):
        """读取并分析文件头部信息"""
        with open(self.lms_file, 'rb') as f:
            # 获取文件大小
            f.seek(0, 2)  # 移动到文件末尾
            self.file_size = f.tell()
            f.seek(0, 0)  # 回到文件开头
            
            # 读取头部信息
            header_data = f.read(12)  # 3个float，每个4字节
            self.header = struct.unpack('fff', header_data)
            
            # 解析头部信息
            ang_range = self.header[0]  # 激光扫描范围
            ang_res = self.header[1]    # 角分辨率
            unit = self.header[2]       # 单位
            
            # 计算每帧激光数据的长度
            max_dat_len = int(ang_range / ang_res) + 1
            
            self.log("===== 激光雷达数据文件分析 =====")
            self.log(f"文件路径: {self.lms_file}")
            self.log(f"文件大小: {self.file_size / 1024:.2f} KB")
            self.log("\n头部信息:")
            self.log(f"  角度范围: {ang_range} 度")
            self.log(f"  角分辨率: {ang_res} 度")
            self.log(f"  距离单位: {unit}")
            self.log(f"  计算的每帧数据点数: {max_dat_len}")
            
            # 理论上每帧数据的字节数（时间戳4字节 + 每个点2字节）
            bytes_per_frame = 4 + 2 * max_dat_len
            estimated_frames = (self.file_size - 12) / bytes_per_frame
            
            self.log(f"  每帧数据理论字节数: {bytes_per_frame}")
            self.log(f"  估计的总帧数: {int(estimated_frames)}")
            
            # 保存数据起始位置
            self.data_start_pos = 12
            
            return {
                'ang_range': ang_range,
                'ang_res': ang_res,
                'unit': unit,
                'max_dat_len': max_dat_len
            }

    def read_lms_file(self, debug_mode=True, num_frames_to_check=10, max_frames=None):
        """读取激光雷达扫描数据文件并进行预处理"""
        header_info = self.read_header()
        max_dat_len = header_info['max_dat_len']
        
        with open(self.lms_file, 'rb') as f:
            # 跳过头部
            f.seek(self.data_start_pos)
            
            # 统计数据
            frame_sizes = []
            weird_data_frames = []
            frame_index = 0
            
            # 循环读取所有帧的激光数据
            while True:
                # 如果设置了最大帧数限制，则检查是否达到限制
                if max_frames is not None and frame_index >= max_frames:
                    self.log(f"已达到设置的最大帧数限制 ({max_frames})，停止读取")
                    break
                
                start_pos = f.tell()
                
                # 读取时间戳
                timestamp_data = f.read(4)  # long类型，4字节
                if not timestamp_data or len(timestamp_data) < 4:
                    break
                
                timestamp = struct.unpack('l', timestamp_data)[0]
                
                if debug_mode and frame_index < num_frames_to_check:
                    self.log(f"\n帧 #{frame_index}:")
                    self.log(f"  时间戳: {timestamp}")
                    self.log(f"  文件位置: {start_pos}")
                
                # 尝试读取激光数据
                laser_data = f.read(2 * max_dat_len)  # unsigned short类型，每个2字节
                actual_data_size = len(laser_data)
                
                if not laser_data:
                    self.log(f"警告：无法读取激光数据，在时间戳 {timestamp} 处文件结束")
                    break
                
                if actual_data_size < 2 * max_dat_len:
                    self.log(f"警告：在时间戳 {timestamp} 处读取的激光数据长度不一致")
                    self.log(f"  预期长度: {2 * max_dat_len} 字节，实际长度: {actual_data_size} 字节")
                    self.log(f"  文件剩余字节数: {self.file_size - f.tell()}")
                    break
                
                # 解析激光数据
                format_str = '{}H'.format(max_dat_len)
                try:
                    laser_values = struct.unpack(format_str, laser_data)
                    
                    # 统计数据点的基本信息
                    min_val = min(laser_values)
                    max_val = max(laser_values)
                    num_zeros = sum(1 for v in laser_values if v == 0)
                    
                    if debug_mode and frame_index < num_frames_to_check:
                        self.log(f"  数据点数: {len(laser_values)}")
                        self.log(f"  数据范围: {min_val} - {max_val}")
                        self.log(f"  零值数量: {num_zeros} ({num_zeros/len(laser_values)*100:.1f}%)")
                        
                        # 输出前10个和后10个数据点的值
                        self.log(f"  前10个点: {laser_values[:10]}")
                        self.log(f"  后10个点: {laser_values[-10:]}")
                    
                    # 检查异常值
                    if max_val > 10000 or num_zeros > len(laser_values) * 0.8:
                        weird_data_frames.append(frame_index)
                        if debug_mode and frame_index < num_frames_to_check:
                            self.log(f"  警告：帧 {frame_index} 包含可能的异常数据!")
                    
                    # 保存帧数据
                    self.scans.append({
                        'timestamp': timestamp,
                        'laser_data': np.array(laser_values),
                        'frame_index': frame_index
                    })
                    
                except struct.error as e:
                    self.log(f"错误：无法解析帧 {frame_index} 的激光数据: {str(e)}")
                    break
                
                frame_sizes.append(actual_data_size)
                frame_index += 1
            
            self.log(f"\n成功读取 {len(self.scans)} 帧激光雷达数据")
            
            if weird_data_frames:
                self.log(f"检测到 {len(weird_data_frames)} 帧可能包含异常数据:")
                self.log(f"  帧索引: {weird_data_frames[:10]}...")
            
            if len(set(frame_sizes)) > 1:
                self.log(f"警告：不同帧的数据大小不一致!")
                self.log(f"  数据大小分布: {set(frame_sizes)}")
    
    def visualize_scan(self, frame_index=0, save_fig=False):
        """可视化某一帧的激光扫描数据"""
        if not self.scans or frame_index >= len(self.scans):
            self.log(f"错误：无法找到帧 {frame_index}，总帧数为 {len(self.scans)}")
            return
        
        scan = self.scans[frame_index]
        laser_data = scan['laser_data']
        
        # 计算角度（弧度）
        header_info = self.read_header() if self.header is None else {
            'ang_range': self.header[0],
            'ang_res': self.header[1],
            'unit': self.header[2],
            'max_dat_len': int(self.header[0] / self.header[1]) + 1
        }
        
        angles = np.arange(0, len(laser_data)) * header_info['ang_res'] * np.pi / 180.0
        
        # 将激光数据转换为距离（米）
        distances = laser_data / header_info['unit']
        
        # 过滤掉异常值（例如为0的值）
        valid_indices = distances > 0
        valid_angles = angles[valid_indices]
        valid_distances = distances[valid_indices]
        
        # 转换为笛卡尔坐标
        x = valid_distances * np.cos(valid_angles)
        y = valid_distances * np.sin(valid_angles)
        
        # 创建图形
        plt.figure(figsize=(10, 10))
        
        # 绘制极坐标图
        ax1 = plt.subplot(211, polar=True)
        ax1.scatter(valid_angles, valid_distances, s=2, c='r')
        ax1.set_title(f'帧 #{frame_index} 极坐标视图（极角vs距离）')
        ax1.set_theta_zero_location('N')
        ax1.set_theta_direction(-1)  # 顺时针方向
        
        # 绘制笛卡尔坐标图
        ax2 = plt.subplot(212)
        ax2.scatter(x, y, s=2, c='b')
        ax2.set_title(f'帧 #{frame_index} 笛卡尔坐标视图')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.grid(True)
        ax2.axis('equal')
        
        # 设置适当的显示范围
        max_range = np.max(valid_distances) * 1.1
        ax2.set_xlim([-max_range, max_range])
        ax2.set_ylim([-max_range, max_range])
        
        # 显示额外信息
        plt.figtext(0.1, 0.01, f"时间戳: {scan['timestamp']}", fontsize=10)
        plt.figtext(0.5, 0.01, f"总点数: {len(laser_data)}, 有效点数: {np.sum(valid_indices)}", fontsize=10)
        
        plt.tight_layout()
        
        # 保存图像
        if save_fig:
            fig_name = f"laser_scan_frame_{frame_index}.png"
            plt.savefig(fig_name, dpi=300)
            self.log(f"已保存可视化图像到 {fig_name}")
        
        plt.show()
        
        # 记录分析到日志
        self.log(f"\n帧 #{frame_index} 数据分析:")
        self.log(f"  总点数: {len(laser_data)}")
        self.log(f"  有效点数: {np.sum(valid_indices)}")
        self.log(f"  有效数据比例: {np.sum(valid_indices)/len(laser_data)*100:.1f}%")
        
        return {
            'x': x,
            'y': y,
            'angles': valid_angles,
            'distances': valid_distances
        }
    
    def visualize_multiple_scans(self, frame_indices=None, num_frames=5, save_fig=False):
        """可视化多帧激光扫描数据以进行比较"""
        if not self.scans:
            self.log("错误：没有可用的扫描数据")
            return
        
        if frame_indices is None:
            # 如果没有指定帧索引，则选择均匀分布的几帧
            total_frames = len(self.scans)
            step = max(1, total_frames // num_frames)
            frame_indices = range(0, total_frames, step)[:num_frames]
        
        self.log(f"\n比较多帧扫描数据:")
        self.log(f"  选择的帧索引: {list(frame_indices)}")
        
        # 计算每帧的笛卡尔坐标
        scan_points = []
        
        header_info = self.read_header() if self.header is None else {
            'ang_range': self.header[0],
            'ang_res': self.header[1],
            'unit': self.header[2],
            'max_dat_len': int(self.header[0] / self.header[1]) + 1
        }
        
        for idx in frame_indices:
            if idx >= len(self.scans):
                continue
                
            scan = self.scans[idx]
            laser_data = scan['laser_data']
            
            # 计算角度（弧度）
            angles = np.arange(0, len(laser_data)) * header_info['ang_res'] * np.pi / 180.0
            
            # 将激光数据转换为距离（米）
            distances = laser_data / header_info['unit']
            
            # 过滤掉异常值（例如为0的值）
            valid_indices = distances > 0
            valid_angles = angles[valid_indices]
            valid_distances = distances[valid_indices]
            
            # 记录帧信息
            self.log(f"  帧 #{idx}:")
            self.log(f"    总点数: {len(laser_data)}")
            self.log(f"    有效点数: {np.sum(valid_indices)}")
            
            # 转换为笛卡尔坐标
            x = valid_distances * np.cos(valid_angles)
            y = valid_distances * np.sin(valid_angles)
            
            scan_points.append({
                'frame_index': idx,
                'timestamp': scan['timestamp'],
                'x': x,
                'y': y
            })
        
        # 创建图形
        plt.figure(figsize=(12, 8))
        
        # 使用不同颜色绘制每帧数据
        colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
        for i, points in enumerate(scan_points):
            color = colors[i % len(colors)]
            plt.scatter(points['x'], points['y'], s=2, c=color, 
                       label=f"帧 #{points['frame_index']} (时间戳:{points['timestamp']})")
        
        plt.title('多帧激光扫描数据比较')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.grid(True)
        plt.axis('equal')
        plt.legend(loc='upper right')
        
        # 设置适当的显示范围
        all_x = np.concatenate([points['x'] for points in scan_points])
        all_y = np.concatenate([points['y'] for points in scan_points])
        max_range = max(np.max(np.abs(all_x)), np.max(np.abs(all_y))) * 1.1
        plt.xlim([-max_range, max_range])
        plt.ylim([-max_range, max_range])
        
        plt.tight_layout()
        
        # 保存图像
        if save_fig:
            fig_name = f"laser_scan_multiple_frames.png"
            plt.savefig(fig_name, dpi=300)
            self.log(f"已保存多帧比较图像到 {fig_name}")
        
        plt.show()
    
    def analyze_data_structure(self):
        """分析数据结构，尝试确定正确的数据格式"""
        if not self.lms_file or not os.path.exists(self.lms_file):
            self.log(f"错误：无法找到文件 {self.lms_file}")
            return
        
        # 获取文件大小
        file_size = os.path.getsize(self.lms_file)
        
        self.log("===== 激光雷达数据结构分析 =====")
        self.log(f"文件大小: {file_size} 字节 ({file_size/1024:.2f} KB)")
        
        # 读取头部信息
        with open(self.lms_file, 'rb') as f:
            # 读取头部信息
            header_data = f.read(12)  # 3个float，每个4字节
            header = struct.unpack('fff', header_data)
            
            # 解析头部信息
            ang_range = header[0]  # 激光扫描范围
            ang_res = header[1]    # 角分辨率
            unit = header[2]       # 单位
            
            self.log(f"\n头部信息:")
            self.log(f"  角度范围: {ang_range} 度")
            self.log(f"  角分辨率: {ang_res} 度")
            self.log(f"  距离单位: {unit}")
            
            # 尝试不同的数据点数
            possible_point_counts = []
            
            # 计算理论上的点数
            theoretical_count = int(ang_range / ang_res) + 1
            possible_point_counts.append(theoretical_count)
            
            # 尝试其他可能的值
            possible_point_counts.extend([
                theoretical_count - 1,  # 可能不包括最后一个点
                int(ang_range / ang_res),  # 不加1
                int(360 / ang_res),  # 如果是全范围
                int(270 / ang_res),  # 常见的270度扫描
                541,  # URG-04LX常见数据点数
                683,  # UTM-30LX常见数据点数
                721,  # SICK LMS-100常见数据点数
                1081  # SICK LMS-200常见数据点数
            ])
            
            # 去重并排序
            possible_point_counts = sorted(set(possible_point_counts))
            
            self.log(f"\n可能的每帧点数:")
            for count in possible_point_counts:
                frame_size = 4 + 2 * count  # 时间戳4字节 + 每点2字节
                estimated_frames = (file_size - 12) / frame_size
                remaining_bytes = (file_size - 12) % frame_size
                
                self.log(f"  点数 {count}:")
                self.log(f"    每帧大小: {frame_size} 字节")
                self.log(f"    估计帧数: {estimated_frames:.2f}")
                self.log(f"    剩余字节: {remaining_bytes}")
                self.log(f"    剩余分析: {'完美匹配' if remaining_bytes == 0 else '不完全匹配'}")
            
            # 尝试读取前几帧，检查一致性
            self.log("\n尝试读取前几帧检查一致性:")
            
            best_points_count = None
            best_points_confidence = 0
            
            for count in possible_point_counts:
                f.seek(12)  # 重置到数据部分开始
                
                try:
                    # 读取前10帧
                    frames_read = 0
                    timestamps = []
                    
                    for _ in range(10):
                        # 读取时间戳
                        timestamp_data = f.read(4)
                        if len(timestamp_data) < 4:
                            break
                            
                        timestamp = struct.unpack('l', timestamp_data)[0]
                        timestamps.append(timestamp)
                        
                        # 跳过激光数据
                        f.seek(2 * count, 1)  # 从当前位置向前移动
                        frames_read += 1
                    
                    # 检查时间戳是否递增及合理性
                    is_monotonic = all(t1 <= t2 for t1, t2 in zip(timestamps[:-1], timestamps[1:]))
                    all_positive = all(t > 0 for t in timestamps)
                    time_diffs = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
                    avg_diff = sum(time_diffs) / len(time_diffs) if time_diffs else 0
                    
                    self.log(f"  点数 {count}:")
                    self.log(f"    成功读取帧数: {frames_read}")
                    self.log(f"    时间戳: {timestamps}")
                    self.log(f"    时间戳单调递增: {is_monotonic}")
                    self.log(f"    时间戳均为正: {all_positive}")
                    
                    if time_diffs:
                        self.log(f"    相邻帧时间差: {time_diffs}")
                        self.log(f"    平均时间差: {avg_diff}")
                    
                    # 计算这种点数配置的可信度
                    confidence = 0
                    if frames_read >= 10:
                        confidence += 3
                    if is_monotonic:
                        confidence += 3
                    if all_positive:
                        confidence += 2
                    if time_diffs and avg_diff > 0 and avg_diff < 1000000:  # 合理的帧间隔
                        confidence += 2
                    
                    # 如果文件大小正好是整数个帧，额外加分
                    remaining_bytes = (file_size - 12) % (4 + 2 * count)
                    if remaining_bytes == 0:
                        confidence += 5
                    
                    self.log(f"    配置可信度: {confidence}/15")
                    
                    # 更新最佳点数配置
                    if confidence > best_points_confidence:
                        best_points_confidence = confidence
                        best_points_count = count
                    
                except Exception as e:
                    self.log(f"  点数 {count} 测试失败: {str(e)}")
            
            # 报告最佳点数配置
            if best_points_count is not None:
                self.log(f"\n推荐点数配置: {best_points_count} (可信度: {best_points_confidence}/15)")
                self.log(f"建议使用此点数进行数据读取")
                return best_points_count
            else:
                self.log(f"\n无法确定最佳点数配置，请手动检查数据")
                return theoretical_count
    
    def fix_laser_data_loading(self):
        """根据数据结构分析，修复激光数据加载问题并保存结果"""
        # 首先分析数据结构
        best_point_count = self.analyze_data_structure()
        
        self.log(f"\n===== 使用优化后的点数 {best_point_count} 重新加载数据 =====")
        
        # 使用分析得到的最佳点数重新读取数据
        self.scans = []  # 清空之前的数据
        
        with open(self.lms_file, 'rb') as f:
            # 读取头部信息
            header_data = f.read(12)  # 3个float，每个4字节
            self.header = struct.unpack('fff', header_data)
            
            # 仅更新max_dat_len，保持其他头部信息不变
            ang_range = self.header[0]
            ang_res = self.header[1]
            unit = self.header[2]
            
            # 使用分析得到的最佳点数
            max_dat_len = best_point_count
            
            # 循环读取所有帧的激光数据
            frame_index = 0
            
            while True:
                # 读取时间戳
                timestamp_data = f.read(4)  # long类型，4字节
                if not timestamp_data or len(timestamp_data) < 4:
                    break
                
                timestamp = struct.unpack('l', timestamp_data)[0]
                
                # 读取激光数据
                laser_data = f.read(2 * max_dat_len)  # unsigned short类型，每个2字节
                if not laser_data or len(laser_data) < 2 * max_dat_len:
                    self.log(f"警告：在帧 {frame_index} (时间戳 {timestamp}) 处文件结束")
                    break
                
                # 解析激光数据
                format_str = '{}H'.format(max_dat_len)
                try:
                    laser_values = struct.unpack(format_str, laser_data)
                    
                    # 保存帧数据
                    self.scans.append({
                        'timestamp': timestamp,
                        'laser_data': np.array(laser_values),
                        'frame_index': frame_index
                    })
                    
                    frame_index += 1
                    
                except struct.error as e:
                    self.log(f"错误：无法解析帧 {frame_index} 的激光数据: {str(e)}")
                    break
        
        self.log(f"使用优化后的点数成功读取 {len(self.scans)} 帧激光雷达数据")
        
        # 对比原始计算方法与优化后的点数
        original_points = int(ang_range / ang_res) + 1
        self.log(f"\n数据点数对比:")
        self.log(f"  原始计算方法: {original_points} 点")
        self.log(f"  优化后的点数: {best_point_count} 点")
        self.log(f"  点数差异: {best_point_count - original_points} 点")
        
        if best_point_count != original_points:
            self.log(f"\n建议更新 work2.py 中的数据读取代码，使用 {best_point_count} 作为每帧的数据点数")
        
        return best_point_count
    
    def export_analysis_summary(self):
        """导出分析总结"""
        if not self.header or not self.scans:
            self.log("错误：没有可用的分析数据")
            return
        
        self.log("\n===== 激光雷达数据分析总结 =====")
        
        # 头部信息
        self.log(f"数据文件: {os.path.basename(self.lms_file)}")
        self.log(f"角度范围: {self.header[0]} 度")
        self.log(f"角分辨率: {self.header[1]} 度")
        self.log(f"距离单位: {self.header[2]}")
        
        # 数据统计
        self.log(f"总帧数: {len(self.scans)}")
        
        if len(self.scans) > 0:
            # 计算时间范围
            start_time = self.scans[0]['timestamp']
            end_time = self.scans[-1]['timestamp']
            duration = end_time - start_time
            
            self.log(f"起始时间戳: {start_time}")
            self.log(f"结束时间戳: {end_time}")
            self.log(f"持续时间: {duration} (时间单位)")
            
            # 计算平均帧率
            if duration > 0:
                frame_rate = len(self.scans) / (duration / 1000000.0)  # 假设时间戳单位是微秒
                self.log(f"估计帧率: {frame_rate:.2f} 帧/秒")
            
            # 随机抽样几帧分析数据分布
            sample_indices = np.linspace(0, len(self.scans)-1, 5, dtype=int)
            self.log("\n随机抽样帧分析:")
            
            for idx in sample_indices:
                scan = self.scans[idx]
                laser_data = scan['laser_data']
                
                min_val = min(laser_data)
                max_val = max(laser_data)
                num_zeros = sum(1 for v in laser_data if v == 0)
                
                self.log(f"  帧 #{idx}:")
                self.log(f"    数据点数: {len(laser_data)}")
                self.log(f"    数据范围: {min_val} - {max_val}")
                self.log(f"    零值比例: {num_zeros/len(laser_data)*100:.1f}%")
        
        self.log("\n分析完成时间: " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))


# 示例用法
if __name__ == "__main__":
    # 替换为实际的激光雷达数据文件路径
    laser_file = "./data/URG_X_20130903_195003.lms"
    
    # 创建数据预处理器
    preprocessor = LaserDataPreprocessor(laser_file)
    
    # 分析数据结构并修复数据加载
    best_point_count = preprocessor.fix_laser_data_loading()
    
    # 读取较少的帧数进行分析（避免处理过多数据）
    preprocessor.read_lms_file(debug_mode=True, num_frames_to_check=5, max_frames=100)
    
    # 可视化部分帧并保存图像
    if len(preprocessor.scans) > 0:
        # 可视化第一帧
        preprocessor.visualize_scan(0, save_fig=True)
        
        # 可视化最后一帧
        if len(preprocessor.scans) > 1:
            preprocessor.visualize_scan(len(preprocessor.scans) - 1, save_fig=True)
        
        # 可视化多帧比较
        preprocessor.visualize_multiple_scans(num_frames=3, save_fig=True)
    
    # 导出分析总结
    preprocessor.export_analysis_summary()
    
    # 输出日志文件路径
    print(f"\n分析日志已保存至: {preprocessor.log_file}")
