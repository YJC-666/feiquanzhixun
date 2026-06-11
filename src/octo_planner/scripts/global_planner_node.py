#!/usr/bin/env python3
import heapq
import math
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class GlobalPlannerNode:
    def __init__(self):
        self.lock = threading.Lock()

        self.start_topic = rospy.get_param("~topics/start", "/start_point")
        self.goal_topic = rospy.get_param("~topics/goal", "/goal_point")
        self.goal_pose_topic = rospy.get_param("~topics/goal_pose", "/goal_pose")
        self.odom_topic = rospy.get_param("~topics/odom", "/pointlio/odom")
        self.global_map_cloud_topic = rospy.get_param(
            "~topics/global_map_cloud", rospy.get_param("~topics/map_cloud", "/navigation_map_cloud")
        )
        self.path_topic = rospy.get_param("~topics/path", "/global_plan")
        self.selection_marker_topic = rospy.get_param("~topics/selection_marker", "/selection_markers")
        self.status_topic = rospy.get_param("~topics/status", "/web_selection_status")

        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.resolution = float(rospy.get_param("~grid/resolution", 0.20))
        self.snap_radius_cells = int(rospy.get_param("~planner/snap_radius_cells", 8))
        self.max_iterations = int(rospy.get_param("~planner/max_iterations", 220000))
        self.robot_radius = float(rospy.get_param("~planner/robot_radius", 0.30))
        self.obstacle_inflation_radius = float(
            rospy.get_param("~planner/obstacle_inflation_radius", self.robot_radius)
        )
        self.obstacle_vertical_min_height = float(
            rospy.get_param("~planner/obstacle_vertical_min_height", 0.18)
        )
        self.obstacle_min_height_above_base = float(
            rospy.get_param("~planner/obstacle_min_height_above_base", 0.10)
        )
        self.obstacle_max_height_above_base = float(
            rospy.get_param("~planner/obstacle_max_height_above_base", 2.00)
        )
        self.replan_on_map_update = bool(rospy.get_param("~planner/replan_on_map_update", True))
        self.use_odom_start_for_replan = bool(
            rospy.get_param("~planner/use_odom_start_for_replan", True)
        )
        self.search_padding_cells = int(rospy.get_param("~planner/search_padding_cells", 40))
        self.max_path_points = int(rospy.get_param("~planner/max_path_points", 5000))

        self.global_map_cells = set()
        self.global_obstacles = set()
        self.start_point = None
        self.goal_point = None
        self.goal_yaw = None
        self.odom_pose = None
        self.last_status_text = None
        self.last_status_time = rospy.Time(0)

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.selection_pub = rospy.Publisher(
            self.selection_marker_topic, MarkerArray, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)

        rospy.Subscriber(self.start_topic, PointStamped, self.on_start, queue_size=1)
        rospy.Subscriber(self.goal_topic, PointStamped, self.on_goal, queue_size=1)
        rospy.Subscriber(self.goal_pose_topic, PoseStamped, self.on_goal_pose, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.global_map_cloud_topic, PointCloud2, self.on_global_map_cloud, queue_size=1)

        rospy.loginfo(
            "global_planner ROS1 started. start=%s goal=%s odom=%s global_map=%s path=%s",
            self.start_topic,
            self.goal_topic,
            self.odom_topic,
            self.global_map_cloud_topic,
            self.path_topic,
        )

    def on_odom(self, msg):
        with self.lock:
            self.odom_pose = msg.pose.pose
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id.strip("/")

    def navigation_reference_z(self):
        with self.lock:
            if self.odom_pose is not None and math.isfinite(self.odom_pose.position.z):
                return float(self.odom_pose.position.z)
            if self.start_point is not None and math.isfinite(self.start_point.z):
                return float(self.start_point.z)
        return 0.0

    def cloud_to_map_and_obstacle_cells(self, msg):
        cells = {}
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
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
        reference_z = self.navigation_reference_z()
        for cell, (min_z, max_z, _count) in cells.items():
            vertical_span = max_z - min_z
            above_base = max_z - reference_z
            if vertical_span >= self.obstacle_vertical_min_height:
                obstacle_cells.add(cell)
                continue
            if self.obstacle_min_height_above_base <= above_base <= self.obstacle_max_height_above_base:
                obstacle_cells.add(cell)
        return map_cells, obstacle_cells

    def on_global_map_cloud(self, msg):
        map_cells, obstacles = self.cloud_to_map_and_obstacle_cells(msg)
        with self.lock:
            self.frame_id = msg.header.frame_id.strip("/") or self.frame_id
            self.global_map_cells = map_cells
            self.global_obstacles = obstacles
            should_replan = (
                self.replan_on_map_update
                and self.start_point is not None
                and self.goal_point is not None
            )
        rospy.loginfo_throttle(
            1.0,
            "global_planner: global map updated: %d map cells, %d obstacle cells",
            len(map_cells),
            len(obstacles),
        )
        if should_replan:
            self.try_plan(use_current_odom_start=True)

    def on_start(self, msg):
        with self.lock:
            self.start_point = Point(x=msg.point.x, y=msg.point.y, z=msg.point.z)
            self.frame_id = msg.header.frame_id.strip("/") or self.frame_id
        self.publish_status(
            "起点已设置，等待目标点" if self.goal_point is None else "起点已更新，重新规划"
        )
        self.publish_selection()
        self.try_plan()

    def on_goal(self, msg):
        with self.lock:
            self.goal_point = Point(x=msg.point.x, y=msg.point.y, z=msg.point.z)
            self.frame_id = msg.header.frame_id.strip("/") or self.frame_id
            self.goal_yaw = None
        self.ensure_start_from_odom()
        self.publish_status("目标点已设置，开始全局路径规划")
        self.publish_selection()
        self.try_plan()

    def on_goal_pose(self, msg):
        with self.lock:
            self.goal_point = Point(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                z=msg.pose.position.z,
            )
            self.goal_yaw = yaw_from_quaternion(msg.pose.orientation)
            self.frame_id = msg.header.frame_id.strip("/") or self.frame_id
        self.ensure_start_from_odom()
        self.publish_status("目标姿态已设置，开始全局路径规划")
        self.publish_selection()
        self.try_plan()

    def ensure_start_from_odom(self):
        with self.lock:
            if self.start_point is not None or self.odom_pose is None:
                return
            self.start_point = Point(
                x=self.odom_pose.position.x,
                y=self.odom_pose.position.y,
                z=self.odom_pose.position.z,
            )

    def try_plan(self, use_current_odom_start=False):
        with self.lock:
            stored_start = self.copy_point(self.start_point)
            goal = self.copy_point(self.goal_point)
            odom_start = self.odom_pose_to_point(self.odom_pose)
            frame_id = self.frame_id
            resolution = self.resolution
            global_map_cells = set(self.global_map_cells)
            obstacle_cells = set(self.global_obstacles)
            goal_yaw = self.goal_yaw

        start = stored_start
        if use_current_odom_start and self.use_odom_start_for_replan and odom_start is not None:
            start = odom_start

        if start is None or goal is None:
            return
        if not global_map_cells:
            self.publish_status("规划失败：还没有收到全局点云地图，无法生成全局路径")
            return

        planning_cells = global_map_cells | obstacle_cells
        blocked = self.build_blocked_cells(obstacle_cells, resolution)
        start_cell_raw = self.point_to_cell(start.x, start.y, resolution)
        goal_cell_raw = self.point_to_cell(goal.x, goal.y, resolution)
        bounds = self.build_search_bounds(planning_cells, start_cell_raw, goal_cell_raw)
        start_cell = self.snap_free_cell(start_cell_raw, blocked, bounds)
        goal_cell = self.snap_free_cell(goal_cell_raw, blocked, bounds)
        if start_cell is None or goal_cell is None:
            self.publish_status("规划失败：起点或目标点落在全局障碍方格内，附近找不到空位")
            return

        blocked.discard(start_cell)
        blocked.discard(goal_cell)

        cells = self.astar(start_cell, goal_cell, blocked, bounds)
        if not cells:
            if use_current_odom_start:
                self.publish_status("全局重规划暂时被障碍切断，保留上一条全局路径")
            else:
                self.publish_empty_path(frame_id)
                self.publish_status("规划失败：全局路径范围内被障碍切断，没有可绕行路径")
            return

        if len(cells) > self.max_path_points:
            step = int(math.ceil(float(len(cells)) / float(self.max_path_points)))
            cells = cells[::step] + ([cells[-1]] if cells[-1] != cells[::step][-1] else [])

        path_z = start.z if math.isfinite(start.z) else 0.0
        path = self.cells_to_path(cells, frame_id, resolution, goal_yaw, path_z)
        self.path_pub.publish(path)
        self.publish_status("全局规划成功：%d 个路径点" % len(path.poses))

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

    def build_search_bounds(self, occupied, start, goal):
        padding = max(
            self.search_padding_cells,
            self.snap_radius_cells
            + int(
                math.ceil(max(self.robot_radius, self.obstacle_inflation_radius) / max(1e-6, self.resolution))
            )
            + 2,
        )
        xs = [cell[0] for cell in occupied] + [start[0], goal[0]]
        ys = [cell[1] for cell in occupied] + [start[1], goal[1]]
        return min(xs) - padding, max(xs) + padding, min(ys) - padding, max(ys) + padding

    @staticmethod
    def cell_in_bounds(cell, bounds):
        min_x, max_x, min_y, max_y = bounds
        return min_x <= cell[0] <= max_x and min_y <= cell[1] <= max_y

    def build_blocked_cells(self, occupied, resolution):
        inflation_radius = max(self.robot_radius, self.obstacle_inflation_radius, resolution)
        inflation = max(1, int(math.ceil(inflation_radius / max(1e-6, resolution))))
        blocked = set()
        inflation_sq = inflation * inflation
        for ox, oy in occupied:
            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    if dx * dx + dy * dy <= inflation_sq:
                        blocked.add((ox + dx, oy + dy))
        return blocked

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
                    open_heap, (tentative_g + self.heuristic(neighbor, goal), tentative_g, neighbor)
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

    def cells_to_path(self, cells, frame_id, resolution, goal_yaw, path_z):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        for index, cell in enumerate(cells):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = cell[0] * resolution
            pose.pose.position.y = cell[1] * resolution
            pose.pose.position.z = path_z
            if index + 1 < len(cells):
                next_cell = cells[index + 1]
                yaw = math.atan2(next_cell[1] - cell[1], next_cell[0] - cell[0])
            elif goal_yaw is not None:
                yaw = goal_yaw
            elif index > 0:
                prev_cell = cells[index - 1]
                yaw = math.atan2(cell[1] - prev_cell[1], cell[0] - prev_cell[0])
            else:
                yaw = 0.0
            pose.pose.orientation = quaternion_from_yaw(yaw)
            path.poses.append(pose)
        return path

    def publish_empty_path(self, frame_id):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        self.path_pub.publish(path)

    def publish_selection(self):
        with self.lock:
            start = self.copy_point(self.start_point)
            goal = self.copy_point(self.goal_point)
            frame_id = self.frame_id
            start_yaw = self.current_yaw()
            goal_yaw = self.goal_yaw
        array = MarkerArray()
        if start is not None:
            array.markers.append(
                self.make_arrow_marker(frame_id, start, start_yaw, 0, "start_heading", (0.15, 0.85, 1.0, 0.95))
            )
            array.markers.append(
                self.make_cube_marker(frame_id, start, 2, "start_point", (0.15, 0.85, 1.0, 0.95))
            )
        if goal is not None:
            if goal_yaw is None and start is not None:
                goal_yaw = math.atan2(goal.y - start.y, goal.x - start.x)
            array.markers.append(
                self.make_arrow_marker(
                    frame_id, goal, goal_yaw or 0.0, 1, "goal_heading", (1.0, 0.45, 0.18, 0.95)
                )
            )
            array.markers.append(
                self.make_cube_marker(frame_id, goal, 3, "goal_point", (1.0, 0.45, 0.18, 0.95))
            )
        self.selection_pub.publish(array)

    def make_arrow_marker(self, frame_id, point, yaw, marker_id, namespace, color):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [
            Point(x=point.x, y=point.y, z=point.z + 0.18),
            Point(
                x=point.x + math.cos(yaw) * 0.55,
                y=point.y + math.sin(yaw) * 0.55,
                z=point.z + 0.18,
            ),
        ]
        return marker

    def make_cube_marker(self, frame_id, point, marker_id, namespace, color):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.32
        marker.scale.y = 0.32
        marker.scale.z = 0.32
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def current_yaw(self):
        if self.odom_pose is None:
            return 0.0
        return yaw_from_quaternion(self.odom_pose.orientation)

    def publish_status(self, text):
        now = rospy.Time.now()
        if (
            self.last_status_text == text
            and self.last_status_time.to_sec() > 0.0
            and (now - self.last_status_time).to_sec() < 1.0
        ):
            rospy.loginfo_throttle(1.0, "global_planner: %s", text)
            return
        self.last_status_text = text
        self.last_status_time = now
        self.status_pub.publish(String(data=text))
        rospy.loginfo_throttle(1.0, "global_planner: %s", text)

    @staticmethod
    def point_to_cell(x, y, resolution):
        return int(round(x / resolution)), int(round(y / resolution))

    @staticmethod
    def odom_pose_to_point(pose):
        if pose is None:
            return None
        return Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)

    @staticmethod
    def copy_point(point):
        if point is None:
            return None
        return Point(x=point.x, y=point.y, z=point.z)


def main():
    rospy.init_node("global_planner")
    GlobalPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()