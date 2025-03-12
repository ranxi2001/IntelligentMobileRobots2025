<think>用户想要修改当前代码，使用终端选择器来选择文件，而不是使用命令行参数。他们还提到文件路径应该参考 work2.py。我需要创建一个新的脚本，使用终端交互方式让用户选择文件和选项，而不是使用命令行参数。

首先，我需要查找 work2.py 文件，了解它的文件路径结构：</think>

Ran tool

Read file: work2/work2.py

Read file: work2/work2.py

```python:work2/lms_to_txt.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LMS文件转换工具 - 将激光雷达数据转换为可读格式
使用终端交互式界面
"""

import os
import sys
from Datapreprocess import LaserDataPreprocessor

def main():
    """主函数，实现激光雷达数据文件转换为可读格式"""
    print("====== 激光雷达LMS文件转换工具 ======")
    
    # 允许用户指定文件路径或使用默认路径
    laser_file = input("请输入激光雷达数据文件路径 (默认为'./data/URG_X_20130903_195003.lms'): ").strip()
    if not laser_file:
        laser_file = "./data/URG_X_20130903_195003.lms"
    
    # 检查文件是否存在
    if not os.path.exists(laser_file):
        print(f"错误：激光数据文件 {laser_file} 不存在")
        return 1
    
    # 选择输出格式
    print("\n请选择输出格式：")
    print("1. CSV格式 (每行一帧，逗号分隔)")
    print("2. 简单文本格式 (每行一帧，空格分隔)")
    print("3. 可读文本格式 (详细格式，每帧多行)")
    
    format_choice = input("请输入选项编号 [1-3]，默认为1: ").strip()
    if not format_choice:
        format_choice = "1"
    
    if format_choice not in ["1", "2", "3"]:
        print("无效选项，使用默认的CSV格式")
        format_choice = "1"
    
    # 是否包含笛卡尔坐标（仅CSV格式）
    include_cartesian = False
    if format_choice == "1":
        cartesian_choice = input("是否在CSV中包含笛卡尔坐标 (x,y)？[y/n]，默认为n: ").strip().lower()
        include_cartesian = cartesian_choice == "y"
    
    # 选择处理帧数
    frames_choice = input("\n请输入要处理的最大帧数 (默认处理全部帧): ").strip()
    max_frames = None
    if frames_choice and frames_choice.isdigit():
        max_frames = int(frames_choice)
    
    # 是否可视化数据
    visualize_choice = input("是否可视化显示激光扫描数据？[y/n]，默认为n: ").strip().lower()
    visualize = visualize_choice == "y"
    
    viz_frames = 1
    if visualize:
        viz_frames_choice = input("请输入要可视化的帧数 (默认为1): ").strip()
        if viz_frames_choice and viz_frames_choice.isdigit():
            viz_frames = int(viz_frames_choice)
    
    # 输出文件设置
    base_name = os.path.splitext(laser_file)[0]
    output_file = None
    
    print("\n输出文件将保存在与输入文件相同的目录下")
    custom_output = input("是否指定自定义输出文件路径？[y/n]，默认为n: ").strip().lower()
    
    if custom_output == "y":
        output_file = input("请输入输出文件路径: ").strip()
    else:
        if format_choice == "1":
            output_file = f"{base_name}.csv"
        elif format_choice == "2":
            output_file = f"{base_name}_simple.txt"
        else:
            output_file = f"{base_name}_readable.txt"
    
    print("\n=== 处理参数摘要 ===")
    print(f"输入文件: {laser_file}")
    print(f"输出文件: {output_file}")
    print(f"输出格式: {['CSV', '简单文本', '可读文本'][int(format_choice)-1]}")
    if format_choice == "1":
        print(f"包含笛卡尔坐标: {include_cartesian}")
    print(f"最大处理帧数: {max_frames if max_frames else '全部'}")
    print(f"可视化显示: {visualize}")
    if visualize:
        print(f"可视化帧数: {viz_frames}")
    
    proceed = input("\n是否继续处理？[y/n]，默认为y: ").strip().lower()
    if proceed == "n":
        print("操作已取消")
        return 0
    
    # 初始化数据预处理器
    print("\n开始处理...")
    preprocessor = LaserDataPreprocessor(laser_file)
    
    # 分析文件头部信息
    print("1. 正在读取文件头部信息...")
    header_info = preprocessor.read_header()
    
    # 读取并修复LMS数据
    print("2. 正在优化数据结构分析...")
    best_point_count = preprocessor.fix_laser_data_loading()
    
    # 读取数据
    print(f"3. 正在读取LMS文件数据...")
    preprocessor.read_lms_file(debug_mode=True, max_frames=max_frames)
    
    # 导出数据
    print("4. 正在导出数据...")
    if format_choice == "1":
        preprocessor.export_to_csv(output_file, include_cartesian=include_cartesian)
    elif format_choice == "2":
        preprocessor.export_to_txt(output_file, format_type="simple")
    else:
        preprocessor.export_to_txt(output_file, format_type="readable")
    
    # 可视化数据（如果需要）
    if visualize and preprocessor.scans:
        print("5. 正在可视化数据...")
        num_frames = min(viz_frames, len(preprocessor.scans))
        
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
    
    print("\n处理完成！")
    print(f"结果已保存到: {output_file}")
    print(f"分析日志已保存至: {preprocessor.log_file}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
```

```bash
cd work2 && python lms_to_txt.py
```
## LMS文件解析工具

我已经为您创建了一个新的交互式工具 `lms_to_txt.py`，该工具可以将LMS文件解析成易于理解的数据格式，每行代表一次采样帧。与之前不同的是，这个工具使用终端交互方式而非命令行参数，让您能够更直观地选择文件和设置导出选项。

### 主要功能

1. **交互式文件选择**：
   - 使用类似于work2.py的方式让用户输入文件路径或使用默认路径
   - 自动检查文件是否存在

2. **多种导出格式**：
   - CSV格式（逗号分隔，可选包含笛卡尔坐标）
   - 简单文本格式（空格分隔）
   - 可读文本格式（详细多行格式）

3. **灵活的处理选项**：
   - 可指定最大处理帧数
   - 可选择是否进行数据可视化
   - 可定制可视化的帧数

4. **自定义输出位置**：
   - 默认将输出文件保存在与输入文件相同的目录
   - 允许用户指定自定义输出路径

### 使用方法

只需运行脚本，然后按照提示进行操作：

```bash
python work2/lms_to_txt.py
```

程序会引导您完成以下步骤：
1. 输入激光雷达数据文件路径
2. 选择输出格式
3. 设置各种处理选项
4. 查看并确认处理参数
5. 执行转换过程

### 导出格式说明

根据您的选择，工具可以生成三种不同格式的文件：

1. **CSV格式**：每行代表一次采样帧
   ```
   帧索引,时间戳,距离值1,距离值2,...
   ```
   如果包含笛卡尔坐标，则会添加额外的x,y值：
   ```
   帧索引,时间戳,距离值1,距离值2,...,x1,y1,x2,y2,...
   ```

2. **简单文本格式**：空格分隔的数据
   ```
   帧索引 时间戳 距离值1 距离值2 ...
   ```

3. **可读文本格式**：详细的多行格式
   ```
   帧索引: 0
   时间戳: 1234567890
   点索引 角度(度) 距离值
   0 0.00 123
   1 0.25 456
   ...
   ```

这个工具完全基于您提供的LaserDataPreprocessor类，同时采用了与work2.py相似的交互方式，使其更加易于使用。

需要注意的是，为了确保数据质量，工具会自动分析数据结构并优化读取方式，然后再进行数据导出。处理完成后，会生成包含详细信息的日志文件，方便您进行后续分析和处理。
