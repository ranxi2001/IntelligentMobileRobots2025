# 机器人运动轨迹估计实验

## 题目

### 作业1：基于航位推算的机器人运动估计

#### 目的
- 理解运动传感器和基于航位推算的机器人运动估计方法

#### 使用数据
1. 车轮编码器数据：`COMPort_X_20130903_195003.txt`
2. IMU（惯性导航）数据：`InterSense_X_20130903_195003.txt`

##### 数据说明

###### 车轮编码器数据

- 文件名：`COMPort_X_20130903_195003.txt`

- 数据格式（文本）：

  ```
  E Millisecond 1 Count
  E 71403642 1 13832
  E 71403656 1 13832
  E 71403667 1 13832
  ```

  - Count：累计计数，范围[1,30000]
  - Millisecond：第一个LONG型数据是时钟戳，单位是毫秒; 
  - Count：最后数据是脉冲数，可以代表行驶距离， 一个脉冲为0.003846154 米，注意： 30000溢出。
  - 距离换算：1 Count ≈ 0.003846154 meter

###### IMU数据

- 文件名：`InterSense_X_20130903_195003.txt`

- 数据格式（文本）：

  ```
  IMU Millisecond 0 0 横滚角 俯仰角 航向角
  IMU 71407079 0 0 -1.73278 3.73778 -137.976
  IMU 71407089 39.8195 181 -1.73086 3.73267 -137.988
  IMU 71407109 39.8195 181 -1.74338 3.74885 -137.959
  ```

  | 列名          | 解释                                                         |
  | ------------- | ------------------------------------------------------------ |
  | Millisecond   | LONG型数据是时钟戳，单位是毫秒                               |
  | 0 0           | 第二个数据和第三个数据分别代表帧率和数据是否有效（数据>=180有效） |
  | 横滚角 俯仰角 | 第四个数据和第五个数据代表横滚角，俯仰角。                   |
  | 航向角        | 朝向角度-180°到 180°                                         |

- 角度单位：度

- 注：作业1、2仅使用 Millisecond 和航向角数据


#### 基本要求

- 利用车轮编码器数据、IMU数据和航位推算，计算小车行驶轨迹
- 编程语言：Python/C/C++

#### 提交要求
- 提交内容：代码与报告（1页以上）
- 截止时间：3月23日

## 题解

首先我注意到时间单位`Millisecond`在两个文件中初始值并不一致，所以选择从相同的时间戳开始计算小车的行驶轨迹。

由于作业一仅使用 Millisecond 、 count 和航向角数据，我们可以首先把两个数据合并为一个文件，包括时间戳、累计计数和航向角即可。

### 数据处理

在数据处理过程中，我们需要解决以下几个问题：

1. **时间同步问题**：车轮编码器数据和IMU数据的采样时间不一致，需要找到共同的时间范围并进行匹配。
2. **计数溢出问题**：车轮编码器的计数值在达到30000后会溢出，需要检测并修正这种情况。
3. **位移计算**：根据计数值的变化计算小车的位移。

为了解决这些问题，我实现了以下几个关键函数：

1. `find_common_time_range`：找到两个数据集共同的时间范围，确保我们只处理两个传感器都有数据的时间段。
2. `calculate_displacement`：计算位移增量，并处理计数溢出的情况。
3. `calculate_trajectory`：根据位移和航向角计算小车的轨迹。

### 编写小车行驶代码

以下是实现小车轨迹计算的核心代码：

```python
def calculate_displacement(counts, count_to_meter=0.003846154):
    """计算位移增量，处理溢出情况"""
    # 创建一个新数组来存储修正后的计数值
    corrected_counts = np.copy(counts)
    
    # 检测并修正溢出
    for i in range(1, len(counts)):
        if counts[i] < counts[i-1] and counts[i-1] - counts[i] > 15000:  # 溢出检测阈值
            # 溢出发生，修正当前及后续所有计数
            corrected_counts[i:] += 30000
    
    # 计算相邻点之间的差值作为位移增量
    displacements = np.zeros(len(corrected_counts))
    for i in range(1, len(corrected_counts)):
        displacements[i] = (corrected_counts[i] - corrected_counts[i-1]) * count_to_meter
    
    return displacements

def calculate_trajectory(wheel_time, wheel_counts, imu_time, imu_heading):
    """计算机器人轨迹"""
    # 找到共同的时间范围
    (wheel_start, wheel_end), (imu_start, imu_end) = find_common_time_range(wheel_time, imu_time)
    
    # 截取共同时间范围内的数据
    wheel_time = wheel_time[wheel_start:wheel_end]
    wheel_counts = wheel_counts[wheel_start:wheel_end]
    imu_time = imu_time[imu_start:imu_end]
    imu_heading = imu_heading[imu_start:imu_end]
    
    # 计算位移增量
    displacements = calculate_displacement(wheel_counts)
    
    # 初始化轨迹数组
    x = np.zeros(len(wheel_time))
    y = np.zeros(len(wheel_time))
    
    # 初始位置
    current_x = 0
    current_y = 0
    
    # 对每个时间点计算位置
    for i in range(len(wheel_time)):
        # 找到最接近当前车轮编码器时间的IMU数据索引
        imu_idx = np.argmin(np.abs(imu_time - wheel_time[i]))
        
        # 获取当前航向角（弧度）
        heading_rad = degrees_to_radians(imu_heading[imu_idx])
        
        # 计算位置增量
        dx = displacements[i] * np.cos(heading_rad)
        dy = displacements[i] * np.sin(heading_rad)
        
        # 更新位置
        current_x += dx
        current_y += dy
        
        x[i] = current_x
        y[i] = current_y
    
    return x, y
```

这段代码的核心思想是：

1. 首先找到两个数据集共同的时间范围，确保数据的一致性。
2. 计算车轮编码器数据的位移增量，并处理可能的溢出情况。
3. 对于每个时间点，找到最接近的IMU数据，获取当前的航向角。
4. 根据位移增量和航向角，计算小车在x和y方向上的位移。
5. 累加位移，得到小车的轨迹。

### 绘制小车行驶轨迹

最后，我们使用matplotlib库绘制小车的行驶轨迹：

```python
def plot_trajectory(x, y):
    """绘制轨迹"""
    plt.figure(figsize=(10, 10))
    plt.plot(x, y, 'b-', label='机器人轨迹')
    plt.scatter(x[0], y[0], color='g', s=100, label='起点')
    plt.scatter(x[-1], y[-1], color='r', s=100, label='终点')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('机器人运动轨迹')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.savefig('trajectory.png')
    plt.show()
```

通过运行程序，我们可以得到小车的行驶轨迹图，直观地展示小车的运动情况。

![](D:\OneDrive\课程\计算机\智能移动机器人\作业代码\trajectory.png)

### 结果分析

通过实验，我们成功地利用车轮编码器数据和IMU数据，基于航位推算方法计算了小车的行驶轨迹。这种方法的优点是实现简单，计算效率高；缺点是误差会随着时间累积，长时间运行可能导致较大的位置偏差。

在实际应用中，可以通过融合多种传感器数据（如GPS、视觉里程计等）来提高定位精度，减少误差累积。