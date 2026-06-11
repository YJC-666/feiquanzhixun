#!/usr/bin/env python3
import heapq
import math
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker


def clamp(value, limit):
    limit = abs(limit)
    return max(-limit, min(limit, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class LocalPlannerNode:
    def __init__(self):
        self.lock = threading.Lock()

        self.global_path_topic = rospy.get_param("~topics/global_path", "/global_plan")
        self.local_map_cloud_topic = rospy.get_param(
            "~topics/local_map_cloud", "/local_navigation_map_cloud"
        )
        self.odom_topic = rospy.get_param("~topics/odom", "/pointlio/odom")
        self.start_topic = rospy.get_param("~topics/start_navigation", "/start_navigation")
        self.stop_topic = rospy.get_param("~topics/stop_navigation", "/stop_navigation")
        self.goal_tolerance_topic = rospy.get_param(
            "~topics/goal_tolerance", "/navigation/goal_tolerance"
        )
        self.control_mode_topic = rospy.get_param("~topics/control_mode", "/navigation/control_mode")
        self.manual_topic = rospy.get_param("~topics/manual_cmd", "/web_cmd_vel")
        self.cmd_topic = rospy.get_param("~topics/nav_cmd", "/cmd_vel")
        self.local_plan_topic = rospy.get_param("~topics/local_plan", "/local_plan")
        self.status_topic = rospy.get_param("~topics/status", "/web_selection_status")
        self.tracking_marker_topic = rospy.get_param(
            "~topics/tracking_marker", "/tracking_point_marker"
        )

        self.require_start_command = bool(rospy.get_param("~control/require_start_command", True))
        self.rate = float(rospy.get_param("~control/rate", 20.0))
        self.lookahead_distance = float(rospy.get_param("~control/lookahead_distance", 0.55))
        self.tracking_tolerance = float(rospy.get_param("~control/tracking_point_tolerance", 0.22))
        self.goal_tolerance = float(rospy.get_param("~control/goal_tolerance", 0.08))
        self.goal_yaw_tolerance = float(rospy.get_param("~control/goal_yaw_tolerance", 0.24))
        self.linear_gain = float(rospy.get_param("~control/linear_gain", 0.75))
        self.lateral_gain = float(rospy.get_param("~control/lateral_gain", 0.65))
        self.heading_gain = float(rospy.get_param("~control/heading_gain", 1.15))
        self.final_yaw_gain = float(rospy.get_param("~control/final_yaw_gain", 0.65))
        self.enable_lateral_motion = bool(rospy.get_param("~control/enable_lateral_motion", True))
        self.max_linear_x = float(rospy.get_param("~control/max_linear_x", 0.30))
        self.max_linear_y = float(rospy.get_param("~control/max_linear_y", 0.22))
        self.max_angular_z = float(rospy.get_param("~control/max_angular_z", 0.65))
        self.manual_abort_threshold = float(rospy.get_param("~control/manual_abort_threshold", 0.03))
        self.squeeze_distance = float(rospy.get_param("~control/squeeze_distance", 0.20))
        self.squeeze_gain = float(rospy.get_param("~control/squeeze_gain", 0.35))

        # Local planning parameters
        self.resolution = float(rospy.get_param("~local_planner/resolution", 0.20))
        self.local_box_half = float(rospy.get_param("~local_planner/box_half_size", 1.5))
        self.robot_radius = float(rospy.get_param("~local_planner/robot_radius", 0.30))
        self.obstacle_inflation_radius = float(
            rospy.get_param("~local_planner/obstacle_inflation_radius", self.robot_radius)
        )
        self.obstacle_vertical_min_height = float(
            rospy.get_param("~local_planner/obstacle_vertical_min_height", 0.18)
        )
        self.obstacle_min_height_above_base = float(
            rospy.get_param("~local_planner/obstacle_min_height_above_base", 0.10)
        )
        self.obstacle_max_height_above_base = float(
            rospy.get_param("~local_planner/obstacle_max_height_above_base", 2.00)
        )
        self.pointcloud_min_distance = float(
            rospy.get_param("~local_planner/pointcloud_min_distance", 0.15)
        )
        self.max_iterations = int(rospy.get_param("~local_planner/max_iterations", 8000))
        self.replan_rate = float(rospy.get_param("~local_planner/replan_rate", 5.0))
        self.snap_radius_cells = int(rospy.get_param("~local_planner/snap_radius_cells", 6))
        self.local_plan_lookahead = float(
            rospy.get_param("~local_planner/local_plan_lookahead", 3.0)
        )

        self.pending_path = []
        self.active_path = []
        self.target_index = 0
        self.odom_pose = None
        self.active = False
        self.navigation_mode = "auto"
        self.frame_id = "camera_init"
        self.global_plan_poses = []
        self.local_map_cells = set()
        self.local_obstacles = set()
        self.last_replan_time = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.local_plan_pub = rospy.Publisher(self.local_plan_topic, Path, queue_size=1, latch=True)
        self.tracking_pub = rospy.Publisher(self.tracking_marker_topic, Marker, queue_size=1, latch=True)

        rospy.Subscriber(self.global_path_topic, Path, self.on_global_path, queue_size=1)
        rospy.Subscriber(self.local_map_cloud_topic, PointCloud2, self.on_local_map_cloud, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.start_topic, Bool, self.on_start_navigation, queue_size=1)
        rospy.Subscriber(self.stop_topic, Bool, self.on_stop_navigation, queue_size=1)
        rospy.Subscriber(self.goal_tolerance_topic, Float64, self.on_goal_tolerance, queue_size=1)
        rospy.Subscriber(self.control_mode_topic, String, self.on_control_mode, queue_size=1)
        rospy.Subscriber(self.manual_topic, Twist, self.on_manual_cmd, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate)), self.on_timer)
        rospy.loginfo(
            "local_planner ROS1 started. global_path=%s local_map=%s odom=%s nav_cmd=%s local_plan=%s",
            self.global_path_topic,
            self.local_map_cloud_topic,
            self.odom_topic,
            self.cmd_topic,
            self.local_plan_topic,
        )

    # ---------- Subscribers ----------

    def on_global_path(self, msg):
        poses = list(msg.poses)
        with self.lock:
            self.frame_id = msg.header.frame_id or self.frame_id
            self.global_plan_poses = poses
            if self.navigation_mode != "auto":
                self.pending_path = []
                self.active_path = []
                self.active = False
                self.target_index = 0
                return
            was_active = self.active
            self.pending_path = [] if was_active else poses
            self.active_path = poses if (was_active or not self.require_start_command) else []
            self.active = bool(poses) and (was_active or not self.require_start_command)
            self.target_index = 0
        if poses:
            if was_active:
                self.publish_status("全局路径已更新，局部规划器继续跟踪 %d 点" % len(poses))
            elif self.require_start_command:
                self.publish_status("收到全局路径：%d 点，等待 Web 确认开始导航" % len(poses))
            else:
                self.publish_status("收到全局路径：%d 点，已自动开始导航" % len(poses))
        else:
            self.stop_navigation("收到空全局路径，停止导航")

    def on_local_map_cloud(self, msg):
        map_cells, obstacles = self.cloud_to_map_and_obstacle_cells(msg)
        with self.lock:
            self.frame_id = msg.header.frame_id.strip("/") or self.frame_id
            self.local_map_cells = map_cells
            self.local_obstacles = obstacles

    def on_odom(self, msg):
        with self.lock:
            self.odom_pose = msg.pose.pose
            self.frame_id = msg.header.frame_id or self.frame_id

    def on_start_navigation(self, msg):
        if not msg.data:
            return
        with self.lock:
            if self.navigation_mode != "auto":
                self.publish_status("开始导航失败：当前为手动模式")
                return
            if not self.pending_path:
                self.publish_status("开始导航失败：没有待执行路径")
                return
            self.active_path = list(self.pending_path)
            self.pending_path = []
            self.active = True
            self.target_index = 0
        self.publish_status("开始导航：局部规划器输出 /nav_cmd_vel")

    def on_stop_navigation(self, msg):
        if msg.data:
            self.stop_navigation("Web 停止导航")

    def on_goal_tolerance(self, msg):
        value = float(msg.data)
        if not math.isfinite(value) or value <= 0.0:
            self.publish_status("目标容差更新失败：数值无效")
            return
        self.goal_tolerance = max(0.01, min(value, 2.0))
        rospy.set_param("~control/goal_tolerance", self.goal_tolerance)
        self.publish_status("目标到达容差已更新为 %.3fm" % self.goal_tolerance)

    def on_control_mode(self, msg):
        mode = str(msg.data or "").strip().lower()
        if mode not in ("manual", "auto"):
            self.publish_status("导航模式更新失败：只支持 manual/auto")
            return
        with self.lock:
            previous_mode = self.navigation_mode
            self.navigation_mode = mode
        if mode == "manual":
            self.reset_navigation_state("已切换手动模式：导航状态已重置")
        elif previous_mode != "auto":
            self.reset_navigation_state("已切换自动模式：等待重新设置导航目标")
        else:
            self.publish_status("当前已是自动模式")

    def on_manual_cmd(self, msg):
        with self.lock:
            manual_mode = (self.navigation_mode == "manual")
        if manual_mode:
            # Manual mode: forward directly to cmd_vel
            self.cmd_pub.publish(msg)
            return
        if self.is_active_twist(msg):
            with self.lock:
                was_active = self.active
                self.active = False
                self.active_path = []
                self.pending_path = []
            if was_active:
                self.publish_zero()
                self.clear_tracking_marker()
                self.publish_status("手动摇杆接管：自动导航已退出")

    # ---------- Main loop ----------

    def on_timer(self, _event):
        with self.lock:
            if not self.active or not self.active_path:
                return
            path = list(self.active_path)
            target_index = self.target_index
            odom_pose = self.odom_pose
            local_obstacles = set(self.local_obstacles)

        if odom_pose is None:
            rospy.logwarn_throttle(2.0, "local_planner waiting for odom: %s", self.odom_topic)
            return

        x = odom_pose.position.x
        y = odom_pose.position.y
        yaw = yaw_from_quaternion(odom_pose.orientation)

        # Check goal reached
        goal = path[-1].pose
        goal_dist = math.hypot(goal.position.x - x, goal.position.y - y)
        goal_yaw = yaw_from_quaternion(goal.orientation)
        goal_yaw_error = normalize_angle(goal_yaw - yaw)
        if goal_dist <= self.goal_tolerance and abs(goal_yaw_error) <= self.goal_yaw_tolerance:
            self.stop_navigation("到达目标点")
            return

        # Local replan: check if current path segment collides with local obstacles
        now = rospy.Time.now()
        should_replan = (now - self.last_replan_time).to_sec() >= (1.0 / max(1.0, self.replan_rate))
        if should_replan:
            local_path = self.try_local_replan(path, x, y, yaw)
            if local_path:
                with self.lock:
                    self.active_path = local_path
                    self.target_index = 0
                self.local_plan_pub.publish(self.poses_to_path(local_path, self.frame_id, yaw))
                self.last_replan_time = now

        # Pure pursuit
        path = self.active_path  # re-read after possible replan
        target_index = self.advance_target(path, self.target_index, x, y)
        target_index = self.lookahead_target(path, target_index, x, y)
        target = path[min(target_index, len(path) - 1)].pose

        dx = target.position.x - x
        dy = target.position.y - y
        distance_to_target = math.hypot(dx, dy)
        rx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ry = -math.sin(yaw) * dx + math.cos(yaw) * dy
        target_path_yaw = yaw_from_quaternion(target.orientation)
        heading_error = normalize_angle(target_path_yaw - yaw)
        heading_speed_scale = max(0.0, math.cos(heading_error))

        cmd = Twist()
        cmd.linear.x = clamp(
            self.linear_gain * max(0.0, distance_to_target) * heading_speed_scale,
            self.max_linear_x,
        )
        squeeze_vy = self.compute_squeeze_vy(x, y, yaw, local_obstacles)
        tracking_vy = self.lateral_gain * ry if self.enable_lateral_motion else 0.0
        cmd.linear.y = clamp(tracking_vy + squeeze_vy, self.max_linear_y)
        cmd.angular.z = clamp(self.heading_gain * heading_error, self.max_angular_z)

        if target_index >= len(path) - 1 and goal_dist <= max(
            self.lookahead_distance, self.goal_tolerance * 2.0
        ):
            final_speed_scale = max(0.0, math.cos(goal_yaw_error))
            cmd.linear.x = clamp(
                self.linear_gain * max(0.0, rx) * final_speed_scale * 0.45,
                self.max_linear_x * 0.55,
            )
            cmd.linear.y = clamp(
                tracking_vy * 0.45 + squeeze_vy, self.max_linear_y * 0.55
            )
            cmd.angular.z = clamp(self.final_yaw_gain * goal_yaw_error, self.max_angular_z * 0.65)

        with self.lock:
            self.target_index = target_index
        self.cmd_pub.publish(cmd)
        self.publish_tracking_marker(target.position)

    # ---------- Local A* replanning ----------

    def try_local_replan(self, global_path, robot_x, robot_y, robot_yaw):
        """Extract global plan segment near robot, check collision, replan locally if needed."""
        with self.lock:
            local_obstacles = set(self.local_obstacles)
            local_map_cells = set(self.local_map_cells)
            frame_id = self.frame_id

        if not global_path:
            return None

        # Extract global path segment within local box
        local_segment = []
        for pose in global_path:
            px = pose.pose.position.x
            py = pose.pose.position.y
            if abs(px - robot_x) <= self.local_box_half and abs(py - robot_y) <= self.local_box_half:
                local_segment.append(pose)

        if len(local_segment) < 2:
            return global_path  # robot near edge of map, keep global path

        # Check collision: does the local segment pass through obstacles?
        blocked = self.build_blocked_cells(local_obstacles)
        collision = False
        for pose in local_segment:
            cell = self.point_to_cell(pose.pose.position.x, pose.pose.position.y, self.resolution)
            if cell in blocked:
                collision = True
                break

        if not collision:
            return None  # no replan needed

        # Build search space: global path bounds near robot + local map cells
        all_cells = set()
        for pose in global_path:
            cell = self.point_to_cell(pose.pose.position.x, pose.pose.position.y, self.resolution)
            if abs(pose.pose.position.x - robot_x) <= self.local_box_half * 2 and abs(
                pose.pose.position.y - robot_y
            ) <= self.local_box_half * 2:
                all_cells.add(cell)
        all_cells |= local_map_cells

        # Start: robot position
        start_raw = self.point_to_cell(robot_x, robot_y, self.resolution)
        # Goal: furthest reachable point on global path within local box
        goal_raw = None
        for pose in reversed(local_segment):
            cell = self.point_to_cell(pose.pose.position.x, pose.pose.position.y, self.resolution)
            if cell not in blocked:
                goal_raw = cell
                break
        if goal_raw is None:
            # All local goals blocked, use the furthest global path point
            for pose in reversed(global_path):
                cell = self.point_to_cell(pose.pose.position.x, pose.pose.position.y, self.resolution)
                if cell not in blocked:
                    goal_raw = cell
                    break
        if goal_raw is None:
            return None  # can't find any valid goal

        bounds = self.build_search_bounds(all_cells, start_raw, goal_raw)
        start_cell = self.snap_free_cell(start_raw, blocked, bounds)
        goal_cell = self.snap_free_cell(goal_raw, blocked, bounds)
        if start_cell is None or goal_cell is None:
            return None

        blocked.discard(start_cell)
        blocked.discard(goal_cell)

        cells = self.astar(start_cell, goal_cell, blocked, bounds)
        if not cells:
            return None  # A* failed

        # Convert cells back to pose list, then append remaining global path
        local_poses = self.cells_to_poses(cells, frame_id, robot_yaw)
        self.publish_status("局部重规划成功：%d 个路径点绕过障碍" % len(local_poses))
        return local_poses

    def cloud_to_map_and_obstacle_cells(self, msg):
        # Robot position for filtering near-body points
        with self.lock:
            robot_x = self.odom_pose.position.x if self.odom_pose else 0.0
            robot_y = self.odom_pose.position.y if self.odom_pose else 0.0
            reference_z = self.odom_pose.position.z if self.odom_pose else 0.0
        min_dist_sq = self.pointcloud_min_distance * self.pointcloud_min_distance

        cells = {}
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            # Filter points within pointcloud_min_distance of robot
            dx = float(x) - robot_x
            dy = float(y) - robot_y
            if dx * dx + dy * dy < min_dist_sq:
                continue
            cell = self.point_to_cell(float(x), float(y), self.resolution)
            z = float(z)
            if cell not in cells:
                cells[cell] = [z, z, 1]
            else:
                cells[cell][0] = min(cells[cell][0], z)
                cells[cell][1] = max(cells[cell][1], z)
                cells[cell][2] += 1

        map_cells = set(cells.keys())
        obstacle_cells = set()
        for cell, (min_z, max_z, _count) in cells.items():
            vertical_span = max_z - min_z
            above_base = max_z - reference_z
            if vertical_span >= self.obstacle_vertical_min_height:
                obstacle_cells.add(cell)
                continue
            if self.obstacle_min_height_above_base <= above_base <= self.obstacle_max_height_above_base:
                obstacle_cells.add(cell)
        return map_cells, obstacle_cells

    def build_blocked_cells(self, occupied):
        inflation_radius = max(self.robot_radius, self.obstacle_inflation_radius, self.resolution)
        inflation = max(1, int(math.ceil(inflation_radius / max(1e-6, self.resolution))))
        blocked = set()
        inflation_sq = inflation * inflation
        for ox, oy in occupied:
            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    if dx * dx + dy * dy <= inflation_sq:
                        blocked.add((ox + dx, oy + dy))
        return blocked

    def build_search_bounds(self, cells, start, goal):
        padding = max(self.snap_radius_cells + 3, 10)
        xs = [c[0] for c in cells] + [start[0], goal[0]]
        ys = [c[1] for c in cells] + [start[1], goal[1]]
        return min(xs) - padding, max(xs) + padding, min(ys) - padding, max(ys) + padding

    @staticmethod
    def cell_in_bounds(cell, bounds):
        min_x, max_x, min_y, max_y = bounds
        return min_x <= cell[0] <= max_x and min_y <= cell[1] <= max_y

    def snap_free_cell(self, raw, blocked, bounds):
        if raw not in blocked and self.cell_in_bounds(raw, bounds):
            return raw
        best = None
        best_d2 = None
        for radius in range(1, self.snap_radius_cells + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    candidate = (raw[0] + dx, raw[1] + dy)
                    if candidate in blocked or not self.cell_in_bounds(candidate, bounds):
                        continue
                    d2 = dx * dx + dy * dy
                    if best is None or d2 < best_d2:
                        best = candidate
                        best_d2 = d2
            if best is not None:
                return best
        return None

    def astar(self, start, goal, blocked, bounds):
        open_heap = []
        heapq.heappush(open_heap, (self.heuristic(start, goal), 0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        iterations = 0

        while open_heap and iterations < self.max_iterations:
            _, current_g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                return self.reconstruct(came_from, current)
            closed.add(current)
            iterations += 1

            for neighbor, move_cost, dx, dy in self.neighbors(current):
                if neighbor in closed or neighbor in blocked or not self.cell_in_bounds(neighbor, bounds):
                    continue
                if dx and dy and (
                    (current[0] + dx, current[1]) in blocked
                    or (current[0], current[1] + dy) in blocked
                ):
                    continue
                tentative_g = current_g + move_cost
                if tentative_g >= g_score.get(neighbor, float("inf")):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                heapq.heappush(
                    open_heap,
                    (tentative_g + self.heuristic(neighbor, goal), tentative_g, neighbor),
                )
        return []

    @staticmethod
    def neighbors(cell):
        x, y = cell
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                yield (x + dx, y + dy), math.sqrt(2.0) if dx and dy else 1.0, dx, dy

    @staticmethod
    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def reconstruct(came_from, current):
        cells = [current]
        while current in came_from:
            current = came_from[current]
            cells.append(current)
        cells.reverse()
        return cells

    def cells_to_poses(self, cells, frame_id, robot_yaw):
        """Convert grid cells to PoseStamped list for path tracking."""
        poses = []
        for cell in cells:
            from geometry_msgs.msg import PoseStamped as PS, Quaternion
            pose = PS()
            pose.header.frame_id = frame_id
            pose.pose.position.x = cell[0] * self.resolution
            pose.pose.position.y = cell[1] * self.resolution
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        # Set orientations along path
        for i, pose in enumerate(poses):
            if i + 1 < len(poses):
                dx = poses[i + 1].pose.position.x - pose.pose.position.x
                dy = poses[i + 1].pose.position.y - pose.pose.position.y
                yaw = math.atan2(dy, dx)
            elif i > 0:
                dx = pose.pose.position.x - poses[i - 1].pose.position.x
                dy = pose.pose.position.y - poses[i - 1].pose.position.y
                yaw = math.atan2(dy, dx)
            else:
                yaw = robot_yaw
            q = Quaternion()
            q.z = math.sin(yaw * 0.5)
            q.w = math.cos(yaw * 0.5)
            pose.pose.orientation = q
        return poses

    def poses_to_path(self, poses, frame_id, robot_yaw):
        """Convert PoseStamped list to nav_msgs/Path."""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        for pose in poses:
            pose.header = path.header
            path.poses.append(pose)
        return path

    # ---------- Pure pursuit helpers ----------

    def advance_target(self, path, target_index, x, y):
        while target_index + 1 < len(path):
            p = path[target_index].pose.position
            if math.hypot(p.x - x, p.y - y) > self.tracking_tolerance:
                break
            target_index += 1
        return target_index

    def lookahead_target(self, path, target_index, x, y):
        index = target_index
        while index + 1 < len(path):
            p = path[index].pose.position
            if math.hypot(p.x - x, p.y - y) >= self.lookahead_distance:
                break
            index += 1
        return index

    # ---------- Squeeze lateral avoidance ----------

    def compute_squeeze_vy(self, robot_x, robot_y, robot_yaw, obstacle_cells):
        """Compute lateral repulsive velocity from nearby obstacles within squeeze_distance."""
        if not obstacle_cells:
            return 0.0
        left_force = 0.0
        right_force = 0.0
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        for (ox, oy) in obstacle_cells:
            wx = ox * self.resolution
            wy = oy * self.resolution
            dx = wx - robot_x
            dy = wy - robot_y
            rx = cos_yaw * dx + sin_yaw * dy
            ry = -sin_yaw * dx + cos_yaw * dy
            if abs(ry) > self.squeeze_distance or rx < -0.1:
                continue
            dist = math.hypot(rx, ry)
            if dist < 1e-3:
                continue
            weight = (self.squeeze_distance - abs(ry)) / max(self.squeeze_distance, 1e-3)
            force = weight / max(dist, 0.05)
            if ry > 0:
                right_force += force
            else:
                left_force += force
        return (left_force - right_force) * self.squeeze_gain

    # ---------- State management ----------

    def stop_navigation(self, reason):
        with self.lock:
            self.active = False
            self.active_path = []
            self.pending_path = []
            self.target_index = 0
        self.publish_zero()
        self.clear_tracking_marker()
        self.publish_status(reason)

    def reset_navigation_state(self, reason):
        with self.lock:
            self.active = False
            self.active_path = []
            self.pending_path = []
            self.target_index = 0
        self.publish_zero()
        self.clear_tracking_marker()
        self.publish_status(reason)

    def publish_zero(self):
        zero = Twist()
        for _ in range(3):
            self.cmd_pub.publish(zero)

    def publish_tracking_marker(self, point):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "tracking_point"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=point.x, y=point.y, z=point.z + 0.20)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.28
        marker.scale.y = 0.28
        marker.scale.z = 0.28
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.12
        marker.color.a = 0.95
        self.tracking_pub.publish(marker)

    def clear_tracking_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "tracking_point"
        marker.id = 1
        marker.action = Marker.DELETE
        self.tracking_pub.publish(marker)

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))
        rospy.loginfo("local_planner: %s", text)

    def is_active_twist(self, msg):
        return (
            abs(msg.linear.x) > self.manual_abort_threshold
            or abs(msg.linear.y) > self.manual_abort_threshold
            or abs(msg.angular.z) > self.manual_abort_threshold
        )

    @staticmethod
    def point_to_cell(x, y, resolution):
        return int(round(x / resolution)), int(round(y / resolution))


def main():
    rospy.init_node("local_planner")
    LocalPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()