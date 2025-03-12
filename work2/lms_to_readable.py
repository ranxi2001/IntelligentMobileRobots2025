#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LMS文件转换工具 - 将激光雷达数据转换为可读格式
"""

import os
import sys
import argparse
from Datapreprocess import LaserDataPreprocessor

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='将LMS激光雷达数据文件转换为可读格式')
    parser.add_argument('input_file', help='输入的LMS文件路径')
    parser.add_argument('-o', '--output', help='输出文件路径 (默认：与输入文件同名，扩展名为.csv)')
    parser.add_argument('-f', '--format', choices=['csv', 'txt_simple', 'txt_readable'], 
                        default='csv', help='输出格式 (默认: csv)')
    parser.add_argument('--cartesian', action='store_true', help='是否包含笛卡尔坐标 (仅CSV格式)')
    parser.add_argument('--frames', type=int, help='最大读取帧数 (默认：全部)')
    parser.add_argument('--visualize', action='store_true', help='是否可视化显示前几帧数据')
    parser.add_argument('--viz-frames', type=int, default=1, help='可视化帧数 (默认：1)')
    
    args = parser.parse_args()
    
    # 检查输入文件是否存在
    if not os.path.exists(args.input_file):
        print(f"错误：找不到输入文件 '{args.input_file}'")
        return 1
    
    # 初始化数据预处理器
    preprocessor = LaserDataPreprocessor(args.input_file)
    
    # 分析文件头部信息
    header_info = preprocessor.read_header()
    
    # 读取并修复LMS数据
    print("正在优化数据结构分析...")
    preprocessor.fix_laser_data_loading()
    
    # 读取数据
    print(f"正在读取LMS文件数据...")
    preprocessor.read_lms_file(debug_mode=True, max_frames=args.frames)
    
    # 导出数据
    if args.format == 'csv':
        output_file = args.output
        print(f"正在将数据导出为CSV格式...")
        preprocessor.export_to_csv(output_file, include_cartesian=args.cartesian)
    elif args.format == 'txt_simple':
        output_file = args.output
        print(f"正在将数据导出为简单TXT格式...")
        preprocessor.export_to_txt(output_file, format_type="simple")
    elif args.format == 'txt_readable':
        output_file = args.output
        print(f"正在将数据导出为可读TXT格式...")
        preprocessor.export_to_txt(output_file, format_type="readable")
    
    # 可视化数据（如果需要）
    if args.visualize and preprocessor.scans:
        print("正在可视化数据...")
        num_frames = min(args.viz_frames, len(preprocessor.scans))
        
        # 可视化第一帧
        preprocessor.visualize_scan(0, save_fig=True)
        
        # 如果要显示多帧，则显示最后一帧和中间帧
        if num_frames > 1:
            # 选择中间帧
            mid_frame = len(preprocessor.scans) // 2
            preprocessor.visualize_scan(mid_frame, save_fig=True)
            
            # 最后一帧
            preprocessor.visualize_scan(len(preprocessor.scans) - 1, save_fig=True)
            
            # 多帧比较
            preprocessor.visualize_multiple_scans(num_frames=min(5, num_frames), save_fig=True)
    
    print("处理完成！")
    return 0

if __name__ == "__main__":
    sys.exit(main()) 