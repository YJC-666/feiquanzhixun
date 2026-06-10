#!/usr/bin/env python3
import math
import threading

import rospy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
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


class D1ControllerNode:
    def __init__(self):
        self.lock = threading.Lock()

        self.path_topic = rospy.get_param("~topics/path", "/planned_path")
        self.odom_topic = rospy.get_param("~topics/odom", "/pointlio/odom")
        self.start_topic = rospy.get_param("~topics/start_navigation", "/start_navigation")
        self.stop_topic = rospy.get_param("~topics/stop_navigation", "/stop_navigation")
        self.goal_tolerance_topic = rospy.get_param("~topics/goal_tolerance", "/navigation/goal_tolerance")
        self.control_mode_topic = rospy.get_param("~topics/control_mode", "/navigation/control_mode")
        self.manual_topic = rospy.get_param("~topics/manual_cmd", "/web_cmd_vel")
        self.cmd_topic = rospy.get_param("~topics/nav_cmd", "/nav_cmd_vel")
        self.status_topic = rospy.get_param("~topics/status", "/web_selection_status")
        self.tracking_marker_topic = rospy.get_param("~topics/tracking_marker", "/tracking_point_marker")

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

        self.pending_path = []
        self.active_path = []
        self.target_index = 0
        self.odom_pose = None
        self.active = False
        self.navigation_mode = "auto"
        self.frame_id = "camera_init"

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.tracking_pub = rospy.Publisher(self.tracking_marker_topic, Marker, queue_size=1, latch=True)

        rospy.Subscriber(self.path_topic, Path, self.on_path, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.start_topic, Bool, self.on_start_navigation, queue_size=1)
        rospy.Subscriber(self.stop_topic, Bool, self.on_stop_navigation, queue_size=1)
        rospy.Subscriber(self.goal_tolerance_topic, Float64, self.on_goal_tolerance, queue_size=1)
        rospy.Subscriber(self.control_mode_topic, String, self.on_control_mode, queue_size=1)
        rospy.Subscriber(self.manual_topic, Twist, self.on_manual_cmd, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.rate)), self.on_timer)
        rospy.loginfo(
            "d1_controller ROS1 started. path=%s odom=%s nav_cmd=%s start=%s stop=%s",
            self.path_topic,
            self.odom_topic,
            self.cmd_topic,
            self.start_topic,
            self.stop_topic,
        )

    def on_path(self, msg):
        poses = list(msg.poses)
        with self.lock:
            self.frame_id = msg.header.frame_id or self.frame_id
            if self.navigation_mode != "auto":
                self.pending_path = []
                self.active_path = []
                self.active = False
                self.target_index = 0
                ignore_for_manual = True
                was_active = False
            else:
                ignore_for_manual = False
                was_active = self.active
                self.pending_path = [] if was_active else poses
                self.active_path = poses if (was_active or not self.require_start_command) else []
                self.active = bool(poses) and (was_active or not self.require_start_command)
                self.target_index = 0
        if ignore_for_manual:
            if poses:
                self.publish_status("当前为手动模式：已忽略路径，需切回自动并重新设置导航目标")
            return
        if poses:
            if was_active:
                self.publish_status("路径已重规划：继续导航跟踪 %d 点" % len(poses))
            else:
                self.publish_status("收到路径：%d 点，等待 Web 确认开始导航" % len(poses) if self.require_start_command else "收到路径：已自动开始导航")
        else:
            self.stop_navigation("收到空路径，停止导航")

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
        self.publish_status("开始导航：路径跟踪输出 /nav_cmd_vel")

    def on_stop_navigation(self, msg):
        if msg.data:
            self.stop_navigation("Web 停止导航/急停")

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
            self.reset_navigation_state("已切换手动模式：机器狗停止，导航状态已重置")
        elif previous_mode != "auto":
            self.reset_navigation_state("已切换自动模式：等待重新设置导航目标")
        else:
            self.publish_status("当前已是自动模式")

    def reset_navigation_state(self, reason):
        with self.lock:
            self.active = False
            self.active_path = []
            self.pending_path = []
            self.target_index = 0
        self.publish_zero()
        self.clear_tracking_marker()
        self.publish_status(reason)

    def on_manual_cmd(self, msg):
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

    def on_timer(self, _event):
        with self.lock:
            if not self.active or not self.active_path:
                return
            path = list(self.active_path)
            target_index = self.target_index
            odom_pose = self.odom_pose

        if odom_pose is None:
            rospy.logwarn_throttle(2.0, "d1_controller waiting for odom: %s", self.odom_topic)
            return

        x = odom_pose.position.x
        y = odom_pose.position.y
        yaw = yaw_from_quaternion(odom_pose.orientation)

        goal = path[-1].pose
        goal_dist = math.hypot(goal.position.x - x, goal.position.y - y)
        goal_yaw = yaw_from_quaternion(goal.orientation)
        goal_yaw_error = normalize_angle(goal_yaw - yaw)
        if goal_dist <= self.goal_tolerance and abs(goal_yaw_error) <= self.goal_yaw_tolerance:
            self.stop_navigation("到达目标点")
            return

        target_index = self.advance_target(path, target_index, x, y)
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
        cmd.linear.x = clamp(self.linear_gain * max(0.0, distance_to_target) * heading_speed_scale, self.max_linear_x)
        cmd.linear.y = clamp(self.lateral_gain * ry, self.max_linear_y) if self.enable_lateral_motion else 0.0
        cmd.angular.z = clamp(self.heading_gain * heading_error, self.max_angular_z)

        if target_index >= len(path) - 1 and goal_dist <= max(self.lookahead_distance, self.goal_tolerance * 2.0):
            final_speed_scale = max(0.0, math.cos(goal_yaw_error))
            cmd.linear.x = clamp(self.linear_gain * max(0.0, rx) * final_speed_scale * 0.45, self.max_linear_x * 0.55)
            cmd.linear.y = clamp(self.lateral_gain * ry * 0.45, self.max_linear_y * 0.55) if self.enable_lateral_motion else 0.0
            cmd.angular.z = clamp(self.final_yaw_gain * goal_yaw_error, self.max_angular_z * 0.65)

        with self.lock:
            self.target_index = target_index
        self.cmd_pub.publish(cmd)
        self.publish_tracking_marker(target.position)

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

    def stop_navigation(self, reason):
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
        rospy.loginfo("d1_controller: %s", text)

    def is_active_twist(self, msg):
        return (
            abs(msg.linear.x) > self.manual_abort_threshold
            or abs(msg.linear.y) > self.manual_abort_threshold
            or abs(msg.angular.z) > self.manual_abort_threshold
        )


def main():
    rospy.init_node("d1_controller")
    D1ControllerNode()
    rospy.spin()


if __name__ == "__main__":
    main()