#!/usr/bin/env python3
import json
import math
import threading

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class SafetyMuxNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.tf_listener = tf.TransformListener()

        self.manual_cmd_topic = rospy.get_param("~topics/manual_cmd", "/web_cmd_vel")
        self.nav_cmd_topic = rospy.get_param("~topics/nav_cmd", "/nav_cmd_vel")
        self.cloud_topic = rospy.get_param("~topics/cloud", "/pointlio/cloud_registered")
        self.odom_topic = rospy.get_param("~topics/odom", "/pointlio/odom")
        self.output_cmd_topic = rospy.get_param("~topics/output_cmd", "/cmd_vel")
        self.status_topic = rospy.get_param("~topics/status", "/dog_safety_mux/status")
        self.manual_prediction_path_topic = rospy.get_param("~topics/manual_prediction_path", "/manual_prediction_path")
        self.local_avoidance_config_topic = rospy.get_param("~topics/local_avoidance_config", "/dog_safety_mux/local_avoidance_config")

        self.base_frame = rospy.get_param("~frames/base_frame", "base_link")
        self.assume_cloud_in_base_frame = bool(rospy.get_param("~frames/assume_cloud_in_base_frame", False))
        self.use_odom_fallback = bool(rospy.get_param("~frames/use_odom_fallback", True))

        self.rate = float(rospy.get_param("~control/rate", 20.0))
        self.command_timeout = rospy.Duration(float(rospy.get_param("~control/command_timeout", 0.35)))
        self.cloud_timeout = rospy.Duration(float(rospy.get_param("~control/cloud_timeout", 0.8)))
        self.stop_without_cloud = bool(rospy.get_param("~control/stop_without_cloud", True))
        self.max_linear_x = abs(float(rospy.get_param("~control/max_linear_x", 0.35)))
        self.max_linear_y = abs(float(rospy.get_param("~control/max_linear_y", 0.25)))
        self.max_angular_z = abs(float(rospy.get_param("~control/max_angular_z", 0.8)))

        self.footprint_length = float(rospy.get_param("~footprint/length", 0.70))
        self.footprint_width = float(rospy.get_param("~footprint/width", 0.42))
        self.z_min = float(rospy.get_param("~footprint/z_min", -0.35))
        self.z_max = float(rospy.get_param("~footprint/z_max", 0.85))
        self.lateral_margin = float(rospy.get_param("~footprint/lateral_margin", 0.12))

        self.preview_time = float(rospy.get_param("~avoidance/preview_time", 1.2))
        self.stop_distance = float(rospy.get_param("~avoidance/stop_distance", 0.45))
        self.warning_distance = float(rospy.get_param("~avoidance/warning_distance", 1.20))
        self.rotate_stop_radius = float(rospy.get_param("~avoidance/rotate_stop_radius", 0.45))
        self.rotate_warning_radius = float(rospy.get_param("~avoidance/rotate_warning_radius", 0.85))
        self.point_sample_step = max(1, int(rospy.get_param("~avoidance/point_sample_step", 8)))
        self.max_points = max(100, int(rospy.get_param("~avoidance/max_points", 3000)))
        self.front_only_obstacles = bool(rospy.get_param("~avoidance/front_only_obstacles", True))
        self.front_min_x = float(rospy.get_param("~avoidance/front_min_x", 0.0))
        self.local_planner_enabled = bool(rospy.get_param("~avoidance/local_planner_enabled", True))
        self.trajectory_horizon = max(0.4, float(rospy.get_param("~avoidance/trajectory_horizon", 1.8)))
        self.trajectory_dt = max(0.05, float(rospy.get_param("~avoidance/trajectory_dt", 0.12)))
        self.candidate_linear_scales = [
            float(value)
            for value in rospy.get_param("~avoidance/candidate_linear_scales", [1.0, 0.75, 0.50, 0.25, 0.0])
        ]
        self.candidate_angular_offsets = [
            float(value)
            for value in rospy.get_param("~avoidance/candidate_angular_offsets", [-0.70, -0.45, -0.25, 0.0, 0.25, 0.45, 0.70])
        ]
        self.trajectory_collision_margin = float(rospy.get_param("~avoidance/trajectory_collision_margin", 0.08))
        self.clearance_weight = float(rospy.get_param("~avoidance/clearance_weight", 4.0))
        self.heading_weight = float(rospy.get_param("~avoidance/heading_weight", 2.2))
        self.speed_weight = float(rospy.get_param("~avoidance/speed_weight", 1.4))
        self.smoothness_weight = float(rospy.get_param("~avoidance/smoothness_weight", 1.2))
        self.rotation_escape_weight = float(rospy.get_param("~avoidance/rotation_escape_weight", 0.6))
        self.prediction_horizon = max(0.2, float(rospy.get_param("~prediction/horizon", 2.5)))
        self.prediction_steps = max(4, int(rospy.get_param("~prediction/steps", 28)))

        self.manual_cmd = Twist()
        self.nav_cmd = Twist()
        self.last_manual_stamp = rospy.Time(0)
        self.last_nav_stamp = rospy.Time(0)
        self.last_cloud_stamp = rospy.Time(0)
        self.points_base = []
        self.odom_matrix = None
        self.odom_pose = None
        self.odom_frame = ""
        self.last_odom_stamp = rospy.Time(0)
        self.last_status = "waiting"

        self.cmd_pub = rospy.Publisher(self.output_cmd_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        self.manual_prediction_pub = rospy.Publisher(self.manual_prediction_path_topic, Path, queue_size=1)

        rospy.Subscriber(self.manual_cmd_topic, Twist, self.on_manual_cmd, queue_size=1)
        rospy.Subscriber(self.nav_cmd_topic, Twist, self.on_nav_cmd, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.on_cloud, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.local_avoidance_config_topic, String, self.on_local_avoidance_config, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate)), self.on_timer)
        rospy.loginfo(
            "dog_safety_mux started. manual=%s nav=%s cloud=%s odom=%s output=%s base_frame=%s local_config=%s",
            self.manual_cmd_topic,
            self.nav_cmd_topic,
            self.cloud_topic,
            self.odom_topic,
            self.output_cmd_topic,
            self.base_frame,
            self.local_avoidance_config_topic,
        )

    def on_manual_cmd(self, msg):
        with self.lock:
            self.manual_cmd = msg
            self.last_manual_stamp = rospy.Time.now()

    def on_nav_cmd(self, msg):
        with self.lock:
            self.nav_cmd = msg
            self.last_nav_stamp = rospy.Time.now()

    def on_local_avoidance_config(self, msg):
        try:
            data = json.loads(msg.data or "{}")
            if not isinstance(data, dict):
                raise ValueError("local avoidance config must be a JSON object")
            self.apply_local_avoidance_config(data)
            self.status_pub.publish(String(data="local avoidance config updated"))
        except Exception as exc:
            rospy.logwarn("dog_safety_mux local avoidance config update failed: %s", exc)

    def apply_local_avoidance_config(self, data):
        scalar_specs = {
            "preview_time": ("preview_time", 0.2, 4.0),
            "stop_distance": ("stop_distance", 0.05, 2.0),
            "warning_distance": ("warning_distance", 0.10, 4.0),
            "rotate_stop_radius": ("rotate_stop_radius", 0.05, 2.0),
            "rotate_warning_radius": ("rotate_warning_radius", 0.10, 4.0),
            "trajectory_horizon": ("trajectory_horizon", 0.4, 4.0),
            "trajectory_dt": ("trajectory_dt", 0.05, 0.5),
            "trajectory_collision_margin": ("trajectory_collision_margin", 0.0, 0.5),
            "clearance_weight": ("clearance_weight", 0.0, 20.0),
            "heading_weight": ("heading_weight", 0.0, 20.0),
            "speed_weight": ("speed_weight", 0.0, 20.0),
            "smoothness_weight": ("smoothness_weight", 0.0, 20.0),
            "rotation_escape_weight": ("rotation_escape_weight", 0.0, 20.0),
            "lateral_margin": ("lateral_margin", 0.0, 0.5),
            "front_min_x": ("front_min_x", -0.5, 1.0),
        }
        bool_specs = {
            "local_planner_enabled": "local_planner_enabled",
            "front_only_obstacles": "front_only_obstacles",
            "stop_without_cloud": "stop_without_cloud",
        }
        int_specs = {
            "point_sample_step": ("point_sample_step", 1, 50),
            "max_points": ("max_points", 100, 20000),
        }
        with self.lock:
            for key, attr in bool_specs.items():
                if key in data:
                    setattr(self, attr, bool(data[key]))
            for key, (attr, low, high) in scalar_specs.items():
                if key in data:
                    value = self.clamp(float(data[key]), low, high)
                    if math.isfinite(value):
                        setattr(self, attr, value)
            for key, (attr, low, high) in int_specs.items():
                if key in data:
                    setattr(self, attr, int(self.clamp(int(data[key]), low, high)))
            if "candidate_linear_scales" in data:
                self.candidate_linear_scales = self.sanitize_float_list(data["candidate_linear_scales"], 0.0, 1.0, [1.0, 0.75, 0.5, 0.25, 0.0])
            if "candidate_angular_offsets" in data:
                self.candidate_angular_offsets = self.sanitize_float_list(data["candidate_angular_offsets"], -2.0, 2.0, [-0.7, -0.45, -0.25, 0.0, 0.25, 0.45, 0.7])

    @classmethod
    def sanitize_float_list(cls, values, low, high, fallback):
        if not isinstance(values, list):
            return list(fallback)
        sanitized = []
        for value in values[:21]:
            number = float(value)
            if math.isfinite(number):
                sanitized.append(cls.clamp(number, low, high))
        return sanitized or list(fallback)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        odom_to_base = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        odom_to_base[0:3, 3] = np.array([p.x, p.y, p.z])
        with self.lock:
            self.odom_matrix = tft.inverse_matrix(odom_to_base)
            self.odom_pose = (float(p.x), float(p.y), float(p.z), self.yaw_from_quaternion(q))
            self.odom_frame = msg.header.frame_id.strip("/")
            self.last_odom_stamp = rospy.Time.now()

    def on_cloud(self, msg):
        try:
            transform = self.lookup_cloud_transform(msg)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            transform = self.lookup_cloud_transform_from_odom(msg)
            if transform is None:
                rospy.logwarn_throttle(2.0, "dog_safety_mux cloud TF unavailable: %s", exc)
                return
            rospy.logwarn_throttle(5.0, "dog_safety_mux using /pointlio/odom fallback for cloud frame %s", msg.header.frame_id)
        points = self.transform_cloud_points(msg, transform)
        with self.lock:
            self.points_base = points
            self.last_cloud_stamp = rospy.Time.now()

    def lookup_cloud_transform(self, msg):
        if self.assume_cloud_in_base_frame:
            return np.identity(4)

        source_frame = msg.header.frame_id.strip("/")
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time(0)
        try:
            self.tf_listener.waitForTransform(self.base_frame, source_frame, stamp, rospy.Duration(0.02))
            trans, rot = self.tf_listener.lookupTransform(self.base_frame, source_frame, stamp)
        except tf.ExtrapolationException:
            trans, rot = self.tf_listener.lookupTransform(self.base_frame, source_frame, rospy.Time(0))
        matrix = tft.quaternion_matrix(rot)
        matrix[0:3, 3] = np.array(trans)
        return matrix

    def lookup_cloud_transform_from_odom(self, msg):
        if not self.use_odom_fallback:
            return None
        source_frame = msg.header.frame_id.strip("/")
        with self.lock:
            matrix = np.array(self.odom_matrix) if self.odom_matrix is not None else None
            odom_frame = self.odom_frame
            odom_age = rospy.Time.now() - self.last_odom_stamp
        if matrix is None:
            return None
        if odom_age > self.cloud_timeout:
            return None
        if source_frame and odom_frame and source_frame != odom_frame:
            rospy.logwarn_throttle(
                2.0,
                "dog_safety_mux odom fallback frame mismatch: cloud=%s odom=%s",
                source_frame,
                odom_frame,
            )
            return None
        return matrix

    def transform_cloud_points(self, msg, matrix):
        points = []
        for index, point in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % self.point_sample_step != 0:
                continue
            x, y, z = point
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            bx, by, bz, _ = matrix.dot(np.array([x, y, z, 1.0]))
            if self.front_only_obstacles and bx < self.front_min_x:
                continue
            if self.z_min <= bz <= self.z_max:
                points.append((float(bx), float(by), float(bz)))
            if len(points) >= self.max_points:
                break
        return points

    def on_timer(self, _event):
        if rospy.is_shutdown():
            return

        now = rospy.Time.now()
        with self.lock:
            manual_cmd = self.copy_twist(self.manual_cmd)
            nav_cmd = self.copy_twist(self.nav_cmd)
            manual_age = now - self.last_manual_stamp
            nav_age = now - self.last_nav_stamp
            cloud_age = now - self.last_cloud_stamp
            points = list(self.points_base)
            odom_pose = self.odom_pose
            odom_frame = self.odom_frame

        cmd, mode = self.select_command(manual_cmd, manual_age, nav_cmd, nav_age)
        cmd = self.clamp_twist(cmd)

        if self.stop_without_cloud and cloud_age > self.cloud_timeout:
            safe_cmd = Twist()
            status = "stop: no fresh obstacle cloud"
        else:
            safe_cmd, status = self.apply_avoidance(cmd, points, mode)

        try:
            self.cmd_pub.publish(safe_cmd)
            self.publish_manual_prediction(safe_cmd, mode, odom_pose, odom_frame)
            if status != self.last_status:
                rospy.loginfo("dog_safety_mux: %s", status)
                self.last_status = status
            self.status_pub.publish(String(data=status))
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def publish_manual_prediction(self, cmd, mode, odom_pose, odom_frame):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = odom_frame or "camera_init"
        if mode not in ("manual", "nav") or odom_pose is None or not self.is_active(cmd):
            self.manual_prediction_pub.publish(path)
            return

        x, y, z, yaw = odom_pose
        dt = self.prediction_horizon / float(self.prediction_steps)
        for _index in range(self.prediction_steps + 1):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z + 0.08
            pose.pose.orientation = self.quaternion_from_yaw(yaw)
            path.poses.append(pose)

            world_vx = math.cos(yaw) * cmd.linear.x - math.sin(yaw) * cmd.linear.y
            world_vy = math.sin(yaw) * cmd.linear.x + math.cos(yaw) * cmd.linear.y
            x += world_vx * dt
            y += world_vy * dt
            yaw += cmd.angular.z * dt
        self.manual_prediction_pub.publish(path)

    def select_command(self, manual_cmd, manual_age, nav_cmd, nav_age):
        if manual_age <= self.command_timeout and self.is_active(manual_cmd):
            return manual_cmd, "manual"
        if nav_age <= self.command_timeout and self.is_active(nav_cmd):
            return nav_cmd, "nav"
        return Twist(), "idle"

    def apply_avoidance(self, cmd, points, mode):
        vx = cmd.linear.x
        vy = cmd.linear.y
        wz = cmd.angular.z
        linear_speed = math.hypot(vx, vy)

        linear_scale = 1.0
        angular_scale = 1.0
        nearest_linear = float("inf")
        nearest_rotate = float("inf")

        if linear_speed > 1e-4:
            ux = vx / linear_speed
            uy = vy / linear_speed
            max_forward = self.footprint_length * 0.5 + self.warning_distance + linear_speed * self.preview_time
            corridor_half_width = self.footprint_width * 0.5 + self.lateral_margin

            for px, py, _pz in points:
                forward = px * ux + py * uy
                lateral = -px * uy + py * ux
                if -0.05 <= forward <= max_forward and abs(lateral) <= corridor_half_width:
                    nearest_linear = min(nearest_linear, max(0.0, forward))

            linear_scale = self.distance_scale(nearest_linear, self.stop_distance, self.warning_distance)

        if abs(wz) > 1e-4:
            for px, py, _pz in points:
                radius = math.hypot(px, py)
                if radius <= self.rotate_warning_radius:
                    nearest_rotate = min(nearest_rotate, radius)
            angular_scale = self.distance_scale(nearest_rotate, self.rotate_stop_radius, self.rotate_warning_radius)

        safe = Twist()
        safe.linear.x = vx * linear_scale
        safe.linear.y = vy * linear_scale
        safe.angular.z = wz * angular_scale
        safe = self.clamp_twist(safe)

        if mode == "idle":
            return safe, "idle"
        if self.local_planner_enabled and self.is_active(cmd) and points:
            planned, plan_status = self.choose_local_trajectory(
                cmd,
                points,
                mode,
                linear_scale,
                angular_scale,
                nearest_linear,
                nearest_rotate,
            )
            if planned is not None:
                return planned, plan_status
        if linear_scale <= 1e-3 or angular_scale <= 1e-3:
            return safe, self.format_status("stop", mode, linear_scale, angular_scale, nearest_linear, nearest_rotate)
        if linear_scale < 0.999 or angular_scale < 0.999:
            return safe, self.format_status("slow", mode, linear_scale, angular_scale, nearest_linear, nearest_rotate)
        return safe, self.format_status("clear", mode, linear_scale, angular_scale, nearest_linear, nearest_rotate)

    def choose_local_trajectory(self, cmd, points, mode, linear_scale, angular_scale, nearest_linear, nearest_rotate):
        desired_vx = cmd.linear.x
        desired_vy = 0.0
        desired_wz = cmd.angular.z
        desired_speed = abs(desired_vx)
        best = None
        best_score = -float("inf")
        best_clearance = 0.0
        collision_free_count = 0

        for linear_scale_candidate in self.candidate_linear_scales:
            candidate_vx = desired_vx * linear_scale_candidate
            for angular_offset in self.candidate_angular_offsets:
                candidate_wz = self.clamp(desired_wz + angular_offset, -self.max_angular_z, self.max_angular_z)
                if abs(desired_vx) < 1e-4 and abs(desired_wz) > 1e-4:
                    candidate_vx = 0.0
                collision, clearance = self.evaluate_trajectory_clearance(candidate_vx, desired_vy, candidate_wz, points)
                if collision:
                    continue
                collision_free_count += 1
                speed_ratio = abs(candidate_vx) / max(desired_speed, self.max_linear_x * 0.25, 1e-3)
                speed_ratio = self.clamp(speed_ratio, 0.0, 1.0)
                clearance_ratio = self.clamp(clearance / max(self.warning_distance, 1e-3), 0.0, 1.0)
                heading_error = abs(candidate_wz - desired_wz) / max(self.max_angular_z, 1e-3)
                smoothness_error = abs(1.0 - linear_scale_candidate) + 0.5 * heading_error
                rotation_escape = 0.0
                if linear_scale < 0.45 and abs(candidate_wz) > abs(desired_wz) + 0.05:
                    rotation_escape = self.rotation_escape_weight * min(abs(candidate_wz) / max(self.max_angular_z, 1e-3), 1.0)
                score = (
                    self.clearance_weight * clearance_ratio
                    + self.speed_weight * speed_ratio
                    - self.heading_weight * heading_error
                    - self.smoothness_weight * smoothness_error
                    + rotation_escape
                )
                if score > best_score:
                    best_score = score
                    best_clearance = clearance
                    best = (candidate_vx, desired_vy, candidate_wz, linear_scale_candidate)

        if best is None:
            return None, "local_plan_blocked: fallback=" + self.format_status("stop", mode, linear_scale, angular_scale, nearest_linear, nearest_rotate)

        out = Twist()
        out.linear.x = best[0]
        out.linear.y = best[1]
        out.angular.z = best[2]
        out = self.clamp_twist(out)
        state = "local_plan_clear" if best[3] > 0.99 and abs(out.angular.z - desired_wz) < 0.05 else "local_plan_adjust"
        status = (
            "%s: mode=%s vx=%.2f wz=%.2f clearance=%.2f candidates=%d base_linear=%.2f base_angular=%.2f nearest_linear=%s nearest_rotate=%s"
            % (
                state,
                mode,
                out.linear.x,
                out.angular.z,
                best_clearance,
                collision_free_count,
                linear_scale,
                angular_scale,
                self.fmt_distance(nearest_linear),
                self.fmt_distance(nearest_rotate),
            )
        )
        return out, status

    def evaluate_trajectory_clearance(self, vx, vy, wz, points):
        half_length = self.footprint_length * 0.5 + self.trajectory_collision_margin
        half_width = self.footprint_width * 0.5 + self.lateral_margin + self.trajectory_collision_margin
        horizon = max(self.trajectory_dt, self.trajectory_horizon)
        steps = max(1, int(math.ceil(horizon / self.trajectory_dt)))
        x = 0.0
        y = 0.0
        yaw = 0.0
        min_clearance = float("inf")

        for _index in range(steps + 1):
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            for px, py, _pz in points:
                dx = px - x
                dy = py - y
                local_x = cos_yaw * dx + sin_yaw * dy
                local_y = -sin_yaw * dx + cos_yaw * dy
                outside_x = max(abs(local_x) - half_length, 0.0)
                outside_y = max(abs(local_y) - half_width, 0.0)
                clearance = math.hypot(outside_x, outside_y)
                if outside_x <= 1e-6 and outside_y <= 1e-6:
                    return True, 0.0
                min_clearance = min(min_clearance, clearance)
            x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * self.trajectory_dt
            y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * self.trajectory_dt
            yaw += wz * self.trajectory_dt
        return False, min_clearance if math.isfinite(min_clearance) else self.warning_distance

    @staticmethod
    def distance_scale(distance, stop_distance, warning_distance):
        if math.isinf(distance):
            return 1.0
        if distance <= stop_distance:
            return 0.0
        if distance >= warning_distance:
            return 1.0
        return max(0.0, min(1.0, (distance - stop_distance) / max(1e-3, warning_distance - stop_distance)))

    @staticmethod
    def is_active(cmd):
        return abs(cmd.linear.x) > 1e-4 or abs(cmd.linear.y) > 1e-4 or abs(cmd.angular.z) > 1e-4

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def quaternion_from_yaw(yaw):
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def clamp_twist(self, cmd):
        out = Twist()
        out.linear.x = self.clamp(cmd.linear.x, -self.max_linear_x, self.max_linear_x)
        out.linear.y = self.clamp(cmd.linear.y, -self.max_linear_y, self.max_linear_y)
        out.angular.z = self.clamp(cmd.angular.z, -self.max_angular_z, self.max_angular_z)
        return out

    @staticmethod
    def copy_twist(cmd):
        out = Twist()
        out.linear.x = cmd.linear.x
        out.linear.y = cmd.linear.y
        out.linear.z = cmd.linear.z
        out.angular.x = cmd.angular.x
        out.angular.y = cmd.angular.y
        out.angular.z = cmd.angular.z
        return out

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def fmt_distance(value):
        return "inf" if math.isinf(value) else "%.2f" % value

    def format_status(self, state, mode, linear_scale, angular_scale, nearest_linear, nearest_rotate):
        return (
            "%s: mode=%s linear_scale=%.2f angular_scale=%.2f nearest_linear=%s nearest_rotate=%s"
            % (
                state,
                mode,
                linear_scale,
                angular_scale,
                self.fmt_distance(nearest_linear),
                self.fmt_distance(nearest_rotate),
            )
        )


def main():
    rospy.init_node("dog_safety_mux")
    SafetyMuxNode()
    rospy.spin()


if __name__ == "__main__":
    main()