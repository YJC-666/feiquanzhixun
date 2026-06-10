# 点云建图 Qt5 GUI 方案讨论记录

## 话题翻译

| ROS Topic | 含义 |
|---|---|
| `/pointlio/cloud_effected` | 受效果影响的点云（位姿修正/投影后） |
| `/pointlio/cloud_registered` | 已配准的点云（全局坐标系，扫描匹配对齐后） |
| `/pointlio/cloud_registered_body` | 机体坐标系下已配准的点云（body frame） |

## 技术决策

### 订阅话题

使用 **`/pointlio/cloud_registered`**，因为点云已配准到全局坐标系，Qt5 GUI 只需追加累积到容器即可显示地图效果。

`/pointlio/cloud_registered_body` 是机体坐标系，累积需要里程计位姿做坐标变换，多一层麻烦。

### 存储策略：按时间分代

- **新点云**：保留在内存中，实时显示
- **旧点云**：落盘存储，防止内存爆炸
- Qt5 GUI 从磁盘按需加载旧数据

### 分块落盘方案（备用）

如果地图范围扩大，可采用瓦片方案：

- 将世界坐标系切成网格，每个格子的点云存为一个二进制文件
- 渲染时按视野范围 LRU 加载瓦片
- `tile_size` = 网格边长（米），室内 2~5m，室外 10~20m

### 当前场景

宇树机械狗，建图范围约 **5m × 5m**。

## 数据流

```
LiDAR → ROS → /pointlio/cloud_registered → Qt5 GUI
                                              ├── 新帧 → 内存（实时渲染）
                                              └── 旧帧 → 磁盘文件（持久化，按需回读）
```

## 待实现

- Qt5 点云可视化程序（subscriber + 累积渲染 + 分部落盘）
- 存储目录结构设计
- 旧数据按需加载机制