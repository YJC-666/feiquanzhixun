#!/usr/bin/env python3
import json
import math
import os
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from jie_map_msgs.srv import (
    ExportNavigationSnapshot,
    ExportNavigationSnapshotResponse,
    GetNavigationMapMeta,
    GetNavigationMapMetaResponse,
    LoadNavigationMapPackage,
    LoadNavigationMapPackageResponse,
    SaveNavigationMapPackage,
    SaveNavigationMapPackageResponse,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String


class OctomapWebBackend:
    def __init__(self):
        self.lock = threading.Lock()

        self.cloud_topic = rospy.get_param("~topics/cloud", "/pointlio/cloud_registered")
        self.odom_topic = rospy.get_param("~topics/odom", "/pointlio/odom")
        self.global_cloud_topic = rospy.get_param("~topics/global_cloud", "/navigation_map_cloud")
        self.local_cloud_topic = rospy.get_param("~topics/local_cloud", "/local_navigation_map_cloud")
        self.local_obstacle_cloud_topic = rospy.get_param("~topics/local_obstacle_cloud", "/local_navigation_obstacle_cloud")
        self.status_topic = rospy.get_param("~topics/status", "/web_selection_status")

        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.map_id = rospy.get_param("~map_id", "dog_l1_global_pointcloud_map")
        self.resolution = float(rospy.get_param("~grid/resolution", 0.20))
        self.z_min = float(rospy.get_param("~grid/z_min", -1.0))
        self.z_max = float(rospy.get_param("~grid/z_max", 2.0))
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.28))

        base_window = max(0.2, float(rospy.get_param("~local_map/window_size", 3.0)))
        self.local_window_x = max(0.2, float(rospy.get_param("~local_map/window_x", base_window)))
        self.local_window_y = max(0.2, float(rospy.get_param("~local_map/window_y", base_window)))
        self.local_window_z = max(0.2, float(rospy.get_param("~local_map/window_z", base_window)))
        self.publish_rate = max(0.2, float(rospy.get_param("~local_map/publish_rate", 5.0)))
        self.clear_stale_global_in_local_box = bool(rospy.get_param("~local_map/clear_stale_global_in_local_box", True))
        self.local_clear_margin = max(0.0, float(rospy.get_param("~local_map/clear_margin", 0.0)))
        self.insert_local_points_into_global = bool(rospy.get_param("~local_map/insert_local_points_into_global", False))
        self.local_global_exclusion_margin = max(
            self.local_clear_margin,
            float(rospy.get_param("~local_map/global_model_exclusion_margin", 0.20)),
        )

        self.point_sample_step = max(1, int(rospy.get_param("~cloud/point_sample_step", 1)))
        self.max_input_points = max(1000, int(rospy.get_param("~cloud/max_input_points", 180000)))
        self.retained_voxel_size = max(0.02, float(rospy.get_param("~cloud/retained_voxel_size", self.resolution)))
        self.max_retained_points = max(1000, int(rospy.get_param("~cloud/max_retained_points", 260000)))
        self.max_retained_publish_points = max(1000, int(rospy.get_param("~cloud/max_retained_publish_points", 90000)))

        self.last_publish = rospy.Time(0)
        self.last_status = rospy.Time(0)
        self.global_bounds = None
        self.local_bounds = None
        self.global_points = {}
        self.local_points = []
        self.local_obstacle_points = []
        self.occupied_cells = set()
        self.latest_odom_pose = None

        self.global_cloud_pub = rospy.Publisher(self.global_cloud_topic, PointCloud2, queue_size=1, latch=True)
        self.local_cloud_pub = rospy.Publisher(self.local_cloud_topic, PointCloud2, queue_size=1, latch=True)
        self.local_obstacle_cloud_pub = rospy.Publisher(self.local_obstacle_cloud_topic, PointCloud2, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)

        rospy.Service("~get_meta", GetNavigationMapMeta, self.handle_get_meta)
        rospy.Service("~export_snapshot", ExportNavigationSnapshot, self.handle_export_snapshot)
        rospy.Service("~save_map_package", SaveNavigationMapPackage, self.handle_save_map_package)
        rospy.Service("~load_map_package", LoadNavigationMapPackage, self.handle_load_map_package)
        rospy.Service("get_meta", GetNavigationMapMeta, self.handle_get_meta)
        rospy.Service("export_snapshot", ExportNavigationSnapshot, self.handle_export_snapshot)
        rospy.Service("save_map_package", SaveNavigationMapPackage, self.handle_save_map_package)
        rospy.Service("load_map_package", LoadNavigationMapPackage, self.handle_load_map_package)

        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.on_cloud, queue_size=1)
        rospy.loginfo(
            "jie_octomap_backend started. cloud=%s odom=%s global=%s local=%s local_obstacle=%s frame=%s res=%.2f local_box=%.1fx%.1fx%.1fm rate=%.1fHz",
            self.cloud_topic,
            self.odom_topic,
            self.global_cloud_topic,
            self.local_cloud_topic,
            self.local_obstacle_cloud_topic,
            self.frame_id,
            self.resolution,
            self.local_window_x,
            self.local_window_y,
            self.local_window_z,
            self.publish_rate,
        )

    def on_odom(self, msg):
        with self.lock:
            self.latest_odom_pose = msg.pose.pose
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id.strip("/")

    def on_cloud(self, msg):
        now = rospy.Time.now()
        if self.last_publish.to_sec() > 0.0 and (now - self.last_publish).to_sec() < 1.0 / self.publish_rate:
            return
        self.last_publish = now

        with self.lock:
            odom_pose = self.latest_odom_pose
        if odom_pose is None:
            rospy.logwarn_throttle(2.0, "jie_octomap_backend: waiting for odom before refreshing local pointcloud box")
            self.publish_status("等待 /pointlio/odom，暂不刷新 3m×3m×3m 局部点云盒子", now, 2.0)
            return

        center = (
            float(odom_pose.position.x),
            float(odom_pose.position.y),
            float(odom_pose.position.z),
        )
        odom_yaw = self.yaw_from_quaternion(odom_pose.orientation)
        half_extents = (
            self.local_window_x * 0.5,
            self.local_window_y * 0.5,
            self.local_window_z * 0.5,
        )
        source_frame = msg.header.frame_id.strip("/") or self.frame_id
        frame_voxels = {}
        local_voxels = {}
        accepted = 0

        for index, point in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % self.point_sample_step != 0:
                continue
            x, y, z = point
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            x = float(x)
            y = float(y)
            z = float(z)
            if z < self.z_min or z > self.z_max:
                continue
            key = self.point_to_voxel_key(x, y, z)
            frame_voxels[key] = (x, y, z)
            if self.point_in_local_box(x, y, z, center, half_extents):
                local_voxels[key] = (x, y, z)
            accepted += 1
            if accepted >= self.max_input_points:
                break

        non_local_frame_voxels = {
            key: point
            for key, point in frame_voxels.items()
            if not self.point_in_local_box(point[0], point[1], point[2], center, half_extents, self.local_global_exclusion_margin)
        }
        local_points = list(local_voxels.values())
        local_obstacle_points = [
            point for point in local_points if self.point_in_front_of_pose(point, center, odom_yaw)
        ]
        occupied = self.cells_from_points(local_obstacle_points)

        with self.lock:
            self.frame_id = source_frame
            cleared_count = self.remove_global_points_in_local_box(center, half_extents)
            self.global_points.update(non_local_frame_voxels)
            if self.insert_local_points_into_global:
                self.global_points.update(local_voxels)
            if len(self.global_points) > self.max_retained_points:
                overflow = len(self.global_points) - self.max_retained_points
                for key in list(self.global_points.keys())[:overflow]:
                    self.global_points.pop(key, None)
            self.local_points = local_points
            self.local_obstacle_points = local_obstacle_points
            self.occupied_cells = occupied
            global_snapshot = list(self.global_points.values())
            self.global_bounds = self.compute_bounds(global_snapshot)
            self.local_bounds = self.compute_bounds(local_points)

        self.publish_layers()
        self.publish_status(
            "pointcloud_map: global=%d local_display=%d local_obstacle=%d cleared_local_stale=%d local_global_insert=%s exclusion_margin=%.2f box=%.1fx%.1fx%.1fm %.1fHz"
            % (
                len(global_snapshot),
                len(local_points),
                len(local_obstacle_points),
                cleared_count,
                "on" if self.insert_local_points_into_global else "off",
                self.local_global_exclusion_margin,
                self.local_window_x,
                self.local_window_y,
                self.local_window_z,
                self.publish_rate,
            ),
            now,
            2.0,
        )

    def point_in_local_box(self, x, y, z, center, half_extents, margin=0.0):
        return (
            abs(x - center[0]) <= half_extents[0] + margin
            and abs(y - center[1]) <= half_extents[1] + margin
            and abs(z - center[2]) <= half_extents[2] + margin
        )

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def point_in_front_of_pose(point, center, yaw):
        x, y, _z = point
        dx = x - center[0]
        dy = y - center[1]
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        return forward >= 0.0

    def remove_global_points_in_local_box(self, center, half_extents):
        if not self.clear_stale_global_in_local_box:
            return 0
        stale_keys = [
            key
            for key, (x, y, z) in self.global_points.items()
            if self.point_in_local_box(x, y, z, center, half_extents, self.local_clear_margin)
        ]
        for key in stale_keys:
            self.global_points.pop(key, None)
        return len(stale_keys)

    def point_to_voxel_key(self, x, y, z):
        return (
            int(round(x / self.retained_voxel_size)),
            int(round(y / self.retained_voxel_size)),
            int(round(z / self.retained_voxel_size)),
        )

    def cells_from_points(self, points):
        occupied = set()
        for x, y, _z in points:
            occupied.add((int(round(x / self.resolution)), int(round(y / self.resolution))))
        return occupied

    def compute_bounds(self, points):
        if not points:
            return None
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        zs = [point[2] for point in points]
        return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))

    def publish_layers(self):
        with self.lock:
            frame = self.frame_id
            global_points = list(self.global_points.values())
            local_points = list(self.local_points)
            local_obstacle_points = list(self.local_obstacle_points)
        self.global_cloud_pub.publish(self.make_cloud(frame, global_points))
        self.local_cloud_pub.publish(self.make_cloud(frame, local_points))
        self.local_obstacle_cloud_pub.publish(self.make_cloud(frame, local_obstacle_points))

    def publish_status(self, text, stamp=None, interval=2.0):
        now = stamp or rospy.Time.now()
        if self.last_status.to_sec() > 0.0 and (now - self.last_status).to_sec() < interval:
            return
        self.last_status = now
        self.status_pub.publish(String(data=text))

    def make_cloud(self, frame, points):
        header = Header(stamp=rospy.Time.now(), frame_id=frame)
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        if len(points) > self.max_retained_publish_points:
            step = int(math.ceil(float(len(points)) / float(self.max_retained_publish_points)))
            points = points[::step]
        return pc2.create_cloud(header, fields, [(x, y, z, 1.0) for x, y, z in points])

    def handle_get_meta(self, _request):
        response = GetNavigationMapMetaResponse()
        with self.lock:
            bounds = self.global_bounds
            frame = self.frame_id
            ready = bool(self.global_points)
        response.success = ready
        response.message = "ok" if ready else "global pointcloud map not ready"
        response.map_id = self.map_id
        response.frame_id = frame
        response.resolution = self.resolution
        response.robot_radius = self.robot_radius
        response.snap_search_radius_cells = int(math.ceil(1.2 / self.resolution))
        response.require_ground_support = False
        response.strict_direct_ground_support = False
        response.ground_support_xy_radius_cells = 0
        response.ground_support_depth_cells = 0
        response.enable_preblocked_costmap = False
        response.preblocked_costmap_radius_cells = 0
        response.preblocked_costmap_weight = 0.0
        response.source_world_file = self.global_cloud_topic
        if bounds:
            response.min_bound.x, response.min_bound.y, response.min_bound.z = bounds[0], bounds[1], bounds[2]
            response.max_bound.x, response.max_bound.y, response.max_bound.z = bounds[3], bounds[4], bounds[5]
        return response

    def handle_export_snapshot(self, _request):
        self.publish_layers()
        response = ExportNavigationSnapshotResponse()
        response.success = True
        response.message = "global and local pointcloud snapshots published"
        response.snapshot_stamp = rospy.Time.now()
        return response

    def handle_save_map_package(self, request):
        response = SaveNavigationMapPackageResponse()
        package_path = os.path.abspath(os.path.expanduser(request.package_path))
        if os.path.exists(package_path) and not request.overwrite:
            response.success = False
            response.message = "package path exists"
            return response
        os.makedirs(package_path, exist_ok=True)
        manifest_path = os.path.join(package_path, "navigation_map.json")
        with self.lock:
            payload = {
                "map_id": self.map_id,
                "frame_id": self.frame_id,
                "resolution": self.resolution,
                "retained_voxel_size": self.retained_voxel_size,
                "local_window_x": self.local_window_x,
                "local_window_y": self.local_window_y,
                "local_window_z": self.local_window_z,
                "global_points": [[x, y, z] for x, y, z in self.global_points.values()],
                "local_points": [[x, y, z] for x, y, z in self.local_points],
                "local_obstacle_points": [[x, y, z] for x, y, z in self.local_obstacle_points],
                "occupied": [[x, y] for x, y in sorted(self.occupied_cells)],
            }
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
        response.success = True
        response.message = "saved"
        response.manifest_path = manifest_path
        return response

    def handle_load_map_package(self, request):
        response = LoadNavigationMapPackageResponse()
        manifest_path = os.path.join(os.path.abspath(os.path.expanduser(request.package_path)), "navigation_map.json")
        if not os.path.exists(manifest_path):
            response.success = False
            response.message = "navigation_map.json not found"
            return response
        with open(manifest_path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        with self.lock:
            self.map_id = str(payload.get("map_id", self.map_id))
            self.frame_id = str(payload.get("frame_id", self.frame_id))
            self.resolution = float(payload.get("resolution", self.resolution))
            self.retained_voxel_size = float(payload.get("retained_voxel_size", self.retained_voxel_size))
            global_points = payload.get("global_points", payload.get("retained_points", payload.get("local_points", [])))
            self.global_points = {
                self.point_to_voxel_key(float(x), float(y), float(z)): (float(x), float(y), float(z))
                for x, y, z in global_points
            }
            local_points = payload.get("local_points", [])
            self.local_points = [(float(x), float(y), float(z)) for x, y, z in local_points]
            local_obstacle_points = payload.get("local_obstacle_points", local_points)
            self.local_obstacle_points = [(float(x), float(y), float(z)) for x, y, z in local_obstacle_points]
            self.occupied_cells = self.cells_from_points(self.local_obstacle_points)
            self.global_bounds = self.compute_bounds(list(self.global_points.values()))
            self.local_bounds = self.compute_bounds(self.local_points)
        self.publish_layers()
        response.success = True
        response.message = "loaded"
        response.map_id = self.map_id
        return response


def main():
    rospy.init_node("jie_octomap_backend")
    OctomapWebBackend()
    rospy.spin()


if __name__ == "__main__":
    main()