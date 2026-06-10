# 野生动物巡查系统 - ROS航点传输

## 系统概述

本系统实现了地面站与ROS系统之间的航点数据传输功能：

1. **地面站** (`run_wildlife_survey.py`) - 规划路径并发布航点数据到ROS topic
2. **航点接收器** (`waypoint_receiver.py`) - 接收ROS topic数据并保存为YAML文件

## 文件说明

### 核心文件
- `run_wildlife_survey.py` - 地面站主程序（已增强ROS发布功能）
- `waypoint_receiver.py` - 航点接收器程序
- `start_waypoint_receiver.sh` - 接收器启动脚本

### 数据流程
```
地面站规划路径 → 保存YAML文件 → 发布到ROS topic → 接收器接收 → 转换回YAML → 保存到~/catkin_ws
```

## 使用方法

### 1. 启动ROS环境

```bash
# 启动ROS Master
roscore
```

### 2. 启动航点接收器

```bash
# 方法1：使用启动脚本
./start_waypoint_receiver.sh

# 方法2：直接运行Python脚本
python3 waypoint_receiver.py
```

### 3. 运行地面站

```bash
python3 run_wildlife_survey.py
```

### 4. 操作流程

1. 在地面站中规划路径
2. 点击"保存航线"按钮
3. 地面站会：
   - 保存YAML文件到本地
   - 发布数据到ROS topic `/wildlife_survey/waypoints`
4. 航点接收器会：
   - 自动接收topic数据
   - 转换回YAML格式
   - 保存到 `~/catkin_ws/received_waypoints_YYYYMMDD_HHMMSS.yaml`

## ROS Topic信息

### Topic名称
- `/wildlife_survey/waypoints`

### 消息类型
- `std_msgs/String`

### 数据格式
- JSON字符串（包含完整的航点和元数据信息）

## 输出文件格式

### 地面站保存的文件
- 位置：当前目录
- 文件名：`waypoints.yaml`

### 接收器保存的文件
- 位置：`~/catkin_ws/`
- 文件名：`waypoints_N.yaml` (N为递增序号)
- 命名规则：
  - 第一次接收：`waypoints_1.yaml`
  - 第二次接收：`waypoints_2.yaml`
  - 以此类推...
- 格式：与地面站保存的格式完全相同

## 文件命名机制

### 智能递增命名
接收器使用智能的文件命名机制，确保不会覆盖已有文件：

1. **基础名称**: `waypoints`
2. **递增后缀**: `_1`, `_2`, `_3`, ...
3. **文件扩展名**: `.yaml`

### 命名示例
```
~/catkin_ws/
├── waypoints_1.yaml    # 第一次接收
├── waypoints_2.yaml    # 第二次接收
├── waypoints_3.yaml    # 第三次接收
└── ...
```

### 自动检测机制
- 启动时扫描目录中已有的 `waypoints_*.yaml` 文件
- 自动确定下一个可用的序号
- 显示已有文件列表和统计信息
- 预告下一个文件名

### 防覆盖保护
- 绝不覆盖已有文件
- 序号自动递增到可用值
- 支持最多9999个文件（理论上）

## 数据内容

保存的YAML文件包含：

```yaml
waypoints:
  - id: 1
    coordinate: "B1 A9"
    grid_position: {col: 8, row: 6}
    global_position: {x: 0.0, y: 0.0, z: 0.0}
    action: "takeoff"
    height_info: {is_takeoff_landing: true, height_m: 0.0}

metadata:
  total_points: 63
  grid_size: "9x7"
  total_cells: 63
  cell_size: {width_m: 0.5, height_m: 0.5, width_cm: 50, height_cm: 50}
  coordinate_system:
    origin: "red_point"
    x_axis: "forward (B1->B7)"
    y_axis: "left (A9->A1)"
    z_axis: "up"
    units: "meters"
  height_settings:
    takeoff_landing_height_m: 0.0
    survey_height_m: 1.22
```

## 故障排除

### 1. ROS连接问题
```bash
# 检查ROS Master状态
rostopic list

# 检查topic是否存在
rostopic info /wildlife_survey/waypoints

# 监听topic数据
rostopic echo /wildlife_survey/waypoints
```

### 2. 权限问题
```bash
# 给启动脚本添加执行权限
chmod +x start_waypoint_receiver.sh
```

### 3. 保存目录问题
```bash
# 检查保存目录是否存在
ls -la ~/catkin_ws/

# 手动创建目录
mkdir -p ~/catkin_ws/
```

### 4. Python依赖问题
```bash
# 安装必要的Python包
pip3 install pyyaml
```

## 日志信息

### 地面站日志
- 发布成功：`航点数据已发布到ROS topic: /wildlife_survey/waypoints`
- 发布失败：`发布ROS消息失败: [错误信息]`

### 接收器日志
- 启动信息：显示监听的topic和保存路径
- 接收信息：显示接收到的数据大小和详细信息
- 保存信息：显示保存的文件路径

## 注意事项

1. **ROS环境**：确保ROS环境正确设置
2. **网络连接**：确保ROS Master可访问
3. **文件权限**：确保有写入~/catkin_ws目录的权限
4. **数据完整性**：接收器会验证数据格式的完整性
5. **时间戳**：每次接收的文件都有唯一的时间戳，避免覆盖

## 扩展功能

系统支持以下扩展：
- 多个接收器同时运行
- 自定义保存路径
- 数据格式验证
- 错误恢复机制
