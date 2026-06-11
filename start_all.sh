#!/bin/bash

# ── 狗机全系统启动脚本 ──
# 用法: ./start_all.sh
# 支持 systemd 服务调用

if [ -n "${JOURNAL_STREAM:-}" ] || [ -n "${INVOCATION_ID:-}" ]; then
    IN_SYSTEMD=1
else
    IN_SYSTEMD=0
fi

LOCKFILE="/tmp/dog_system_start.lock"
if [ -f "$LOCKFILE" ]; then
    OLD_PID=$(cat "$LOCKFILE" 2>/dev/null || echo "")
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo ">>> 旧启动脚本仍在运行 (pid=$OLD_PID)，正在终止..."
        kill "$OLD_PID" 2>/dev/null || true
        # 杀死旧进程组（包括其所有子进程）
        pkill -P "$OLD_PID" 2>/dev/null || true
        sleep 1
        # 如果还没死就强杀
        kill -9 "$OLD_PID" 2>/dev/null || true
        echo ">>> 旧进程已终止，重新启动。"
    fi
fi
echo $$ > "$LOCKFILE"

WS_DIR="/home/orangepi/dog_ws"
cd "$WS_DIR" || { echo ">>> 错误：无法进入 $WS_DIR"; exit 1; }

cleanup() {
    echo ""
    echo ">>> 收到终止信号，清理所有后台进程..."
    kill $(jobs -p) 2>/dev/null || true
    wait $(jobs -p) 2>/dev/null || true
    rm -f "$LOCKFILE"
    echo ">>> 已全部终止。"
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── 清理上次运行的残留进程 ──
echo ""
echo "[清理] 清除上次残留进程..."

# 本工作空间下的 roslaunch/rosrun 进程
pkill -f "roslaunch.*dog_ws" 2>/dev/null && echo "  已终止旧 roslaunch" || true
pkill -f "rosrun.*dog_ws" 2>/dev/null && echo "  已终止旧 rosrun" || true
pkill -f "rosrun.*dog_bringup" 2>/dev/null || true
pkill -f "rosrun.*dog_web_ops" 2>/dev/null || true

# 残留的 /camera 相关
pkill -f "astra_camera" 2>/dev/null || true

sleep 1
echo "[清理] 完成。"
echo ""

echo "============================================"
echo "  狗机全系统启动"
echo "  模式: $( [ "$IN_SYSTEMD" = "1" ] && echo 'systemd' || echo '手动' )"
echo "============================================"

# ── 0. ROS 环境 ──
echo ""
echo "[0/9] 加载 ROS 环境..."

# 读取 .bashrc 中的主从配置 (ROS_MASTER_URI / ROS_HOSTNAME)
eval "$(grep -E '^export ROS_(MASTER_URI|HOSTNAME)=' /home/orangepi/.bashrc 2>/dev/null || true)"
echo "  ROS_MASTER_URI = ${ROS_MASTER_URI:-未设置}"
echo "  ROS_HOSTNAME  = ${ROS_HOSTNAME:-未设置}"

if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo "  -> /opt/ros/noetic"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
    echo "  -> /opt/ros/melodic"
else
    echo "  -> 警告：未找到 ROS setup.bash，尝试继续..."
fi

if [ -f "$WS_DIR/devel/setup.bash" ]; then
    source "$WS_DIR/devel/setup.bash"
    echo "  -> $WS_DIR/devel"
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
echo "  ROS_MASTER_URI = ${ROS_MASTER_URI}"

if ! command -v roslaunch &>/dev/null; then
    echo ">>> 致命错误：roslaunch 不可用，中止启动。"
    rm -f "$LOCKFILE"
    exit 1
fi

# ── 1. L1 LiDAR ──
echo ""
echo "[1/9] 启动 L1 LiDAR 驱动..."
roslaunch unitree_lidar_ros run_without_rviz.launch &
sleep 3

# ── 2. 点云滤波 ──
echo ""
echo "[2/9] 启动点云滤波器..."
rosrun dog_bringup front_rear_cloud_filter.py &
sleep 2

# ── 3. Point-LIO SLAM ──
echo ""
echo "[3/9] 启动 Point-LIO SLAM..."
roslaunch point_lio_unilidar mapping_unilidar_l1.launch rviz:=false &
sleep 4

# ── 4. Octomap ──
echo ""
echo "[4/9] 启动 Octomap 后端..."
roslaunch jie_octomap octomap_web_backend.launch &
sleep 3

# ── 5. 导航规划 ──
echo ""
echo "[5/9] 启动导航规划..."
roslaunch octo_planner nav.launch &
sleep 3

# ── 6. 深度相机 ──
echo ""
echo "[6/9] 启动深度相机..."
roslaunch astra_camera gemini.launch &
sleep 4

# ── 7. 宇树高层桥 ──
echo ""
echo "[7/9] 启动宇树高层控制桥..."
roslaunch fdsc_utils unitree_highlevel_bridge.launch &
sleep 3

# ── 8. Web 地面站 ──
echo ""
echo "[8/9] 启动 Web 地面站..."
roslaunch dog_web_ops web_ops.launch &
sleep 3

echo ""
echo "============================================"
echo "  全部 8 个模块已启动"
echo "  Web 地面站: http://$(hostname -I 2>/dev/null | awk '{print $1}' || echo 'localhost'):8080"
echo "============================================"

wait