# 点云地面站 功能演进记录

## 项目概览

- **包名**: `pointcloud_ground_station`
- **框架**: ROS Noetic + Qt5 + OpenGL
- **平台**: 宇树机械狗，Docker 容器 `ros-noetic`
- **核心文件**: `/home/pi/dog_ws/src/pointcloud_ground_station/src/main.cpp`
- **配置**: `config/view_settings.yaml`

## 数据流

```
LiDAR → /pointlio/cloud_registered → Qt5 GUI
                                       ├── 最近 8 帧 → 内存（实时渲染）
                                       ├── 关键帧 → 空间哈希表（3cm 格子, 150k 预算）
                                       └── 旧帧 → .ply 分段落盘
```

里程计 `/pointlio/odom` → 机器人位姿，驱动相机跟随。

---

## 功能列表

### 1. 相机系统

| 特性 | 实现 |
|------|------|
| 跟随模式 | 默认开启，相机指数平滑跟踪机器人位置（Z 轴 alpha=0.05） |
| 自由模式 | 锁定跟随→自由时，冻结当前机器人位置作为 `free_center_`，绕其旋转 |
| 俯仰角 | -10°（从 20° 调整，略微俯视狗背 +30cm） |
| Z 偏移 | 相机中心在机器人上方 0.3m |
| 鼠标拖拽 | 左键旋转（`orb_pitch_ -= d.y()`），右键/滚轮缩放 |
| 无限缩放 | 最小距离 0.5m，近裁剪面 0.01m，滑块范围 5-500 |
| 保存/恢复视角 | "Save View" 按钮写 `view_state.txt`（yaw/pitch/dist/follow），启动时自动加载 |

### 2. 点云渲染 —— 体素立方体（当前方案）

**完全去除点精灵渲染，只显示 10cm 体素方格。**

| 项目 | 值 |
|------|-----|
| 体素尺寸 | 10cm × 10cm × 10cm |
| 哈希编码 | 3×21bit 打包到 uint64，符号扩展支持负坐标 |
| 色块 | `GL_QUADS` 六面，深度热力图配色，alpha 0.55 |
| 线框 | `GL_LINES`，深灰半透明（alpha 0.35），线宽 1.3 |
| 剔除规则 | Z > 3.5m 丢弃；`dot(P-robotPos, robotForward) <= 0` 丢弃（仅显示机头前方 180°） |
| 适用范围 | 所有帧（普通帧 + 关键帧）统一走体素 |

**深度热力图配色**：蓝(0) → 青(0.25) → 绿(0.5) → 黄(0.75) → 红(1.0)，按体素内点云平均 Z 值映射。

### 3. 静止过滤

机器人 3 秒内位移 < 10cm 时，跳过帧累积，不增加点云。

### 4. 坐标轴

- 长度 0.15m，线宽 7.0
- RGB 对应 XYZ（Z 朝上为蓝色）
- 底部绘制 2m×2m 参考网格

### 5. UI 布局

| 区域 | 内容 |
|------|------|
| 左侧 | QOpenGLWidget 3D 视图 |
| 右上 | 实时统计（帧数、点数） |
| 右中 | 相机控制面板：Save View / Reset View / 跟随距离滑块 |
| 右下 | 点云显示滑块：点大小 / KF 点大小 / 渲染预算 / 透明度 |
| HUD 叠加 | 左上角模式标签、右下角位置坐标（1 位小数，10cm 精度） |

### 6. YAML 配置项

```yaml
point_size: 3.0
kf_point_size: 4.0
render_budget: 150000
point_alpha: 0.7
kf_point_alpha: 0.55
fov: 60.0
follow_distance: 2.0
pitch: -10.0
```

---

## 演进时间线

| 序号 | 改动 | 说明 |
|------|------|------|
| 1 | 相机俯仰角 20°→-10° | 俯视狗背 |
| 2 | Z 偏移 +0.3m | 相机位于机器人上方 |
| 3 | 静止过滤 | 3s/10cm 阈值，修复 `last_move_stamp_` 初始化 bug |
| 4 | 自由模式冻结中心 | `free_center_` = 切离跟随时狗的位置 |
| 5 | 鼠标反转修复 | `orb_pitch_ -= d.y()` |
| 6 | 保存/恢复视角 | `view_state.txt` 读写 |
| 7 | 删除狗模型 | 移除 `drawRobotEnvelope` |
| 8 | 坐标轴缩小 | 1.5m→1.0m→0.5m→0.15m |
| 9 | 位置精度 | 3 位小数→1 位（10cm） |
| 10 | 无限缩放 | 最小距离 0.5m，近平面 0.01m |
| 11 | 背面剔除 | `dot(P-robotPos, robotForward) <= 0` 剔除身后点 |
| 12 | 点精灵纹理 | sphere→EGO-Planner 风格膨胀立方体 |
| 13 | 黑边消除 | `GL_ALPHA_TEST` + `GL_GREATER 0.05` |
| 14 | 20cm 体素渲染 | 叠加在点精灵之上的半透明立方体 |
| 15 | **10cm 体素纯渲染** | 完全去除点精灵，只画 10cm 体素方格 + 线框 |

---

## 编译命令

```bash
docker exec ros-noetic bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/dog_ws && catkin_make --only-pkg-with-deps pointcloud_ground_station"
```