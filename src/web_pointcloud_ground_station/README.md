# Web PointCloud Ground Station

ROS Noetic 浏览器地面站。机器人端提供静态页面和 rosbridge，其他设备用浏览器访问即可显示点云、里程计和相机画面。

## 启动

```bash
cd /home/orangepi/dog_ws
catkin_make --only-pkg-with-deps web_pointcloud_ground_station
source devel/setup.bash
roslaunch web_pointcloud_ground_station web_ground_station.launch
```

当前启动文件只负责 Web 静态服务。ROSBridge 需要单独启动：

```bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

如果机器没有 rosbridge：

```bash
sudo apt install ros-noetic-rosbridge-server
```

## 访问

```text
http://<robot-ip>:8080
```

默认 ROSBridge：

```text
ws://<robot-ip>:9090
```

## 默认话题

| 类型 | Topic |
|---|---|
| 点云 | `/pointlio/cloud_registered` |
| 里程计 | `/pointlio/odom` |
| 压缩相机 | `/camera/image/compressed` |

## 显示策略

- 最近 8 帧实时显示。
- 旧帧合并到关键帧池。
- 点云按 10cm 体素聚合。
- 平均高度映射热力图颜色。
- 有里程计时可仅显示机器人前方 180°。
- 浏览器端使用 Three.js `InstancedMesh` 降低绘制开销。