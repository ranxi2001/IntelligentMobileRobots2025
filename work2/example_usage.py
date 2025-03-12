#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
示例脚本 - 展示如何使用LaserDataPreprocessor类解析LMS文件
"""

from Datapreprocess import LaserDataPreprocessor

# 替换为实际的LMS文件路径
lms_file_path = "./data/URG_X_20130903_195003.lms"  # 根据实际情况修改此路径

def main():
    print("=== LMS文件解析示例 ===")
    print(f"输入文件: {lms_file_path}")
    
    # 1. 创建LaserDataPreprocessor对象
    preprocessor = LaserDataPreprocessor(lms_file_path)
    
    # 2. 读取文件头部信息
    header_info = preprocessor.read_header()
    
    # 3. 优化数据结构分析
    best_point_count = preprocessor.fix_laser_data_loading()
    
    # 4. 读取LMS文件数据
    # 为了提高速度，这里只读取前100帧
    preprocessor.read_lms_file(debug_mode=True, max_frames=100)
    
    # 5. 导出为CSV格式
    csv_file = preprocessor.export_to_csv()
    print(f"已导出CSV格式数据: {csv_file}")
    
    # 6. 导出为简单TXT格式
    txt_simple_file = preprocessor.export_to_txt(format_type="simple")
    print(f"已导出简单TXT格式数据: {txt_simple_file}")
    
    # 7. 导出为可读TXT格式
    txt_readable_file = preprocessor.export_to_txt(format_type="readable")
    print(f"已导出可读TXT格式数据: {txt_readable_file}")
    
    # 8. 可视化第一帧数据
    print("正在可视化第一帧数据...")
    preprocessor.visualize_scan(0, save_fig=True)
    
    print("\n处理完成！")

if __name__ == "__main__":
    main() 