#!/bin/bash

# 航点接收器启动脚本

echo "=== 野生动物巡查系统 - 航点接收器 ==="
echo "启动时间: $(date)"
echo ""

# 检查ROS环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "警告: ROS环境未设置，尝试source setup.bash..."
    if [ -f ~/catkin_ws/devel/setup.bash ]; then
        source ~/catkin_ws/devel/setup.bash
        echo "已加载 ~/catkin_ws/devel/setup.bash"
    elif [ -f /opt/ros/noetic/setup.bash ]; then
        source /opt/ros/noetic/setup.bash
        echo "已加载 /opt/ros/noetic/setup.bash"
    else
        echo "错误: 找不到ROS setup.bash文件"
        exit 1
    fi
fi

# 检查roscore是否运行
echo "检查ROS Master状态..."
if ! rostopic list > /dev/null 2>&1; then
    echo "错误: ROS Master未运行，请先启动roscore"
    echo "在另一个终端运行: roscore"
    exit 1
fi

echo "ROS Master运行正常"
echo ""

# 检查保存目录
SAVE_DIR="$HOME/catkin_ws"
if [ ! -d "$SAVE_DIR" ]; then
    echo "创建保存目录: $SAVE_DIR"
    mkdir -p "$SAVE_DIR"
fi

echo "保存目录: $SAVE_DIR"
echo ""

echo "文件命名规则:"
echo "  • 基础名称: waypoints"
echo "  • 递增后缀: _1, _2, _3, ..."
echo "  • 文件格式: waypoints_N.yaml"
echo "  • 自动检测已有文件，避免覆盖"
echo ""

# 启动航点接收器
echo "启动航点接收器..."
echo "监听topic: /wildlife_survey/waypoints"
echo "按 Ctrl+C 停止接收器"
echo ""

# 运行Python脚本
python3 waypoint_receiver.py

echo ""
echo "航点接收器已停止"
echo "结束时间: $(date)"
