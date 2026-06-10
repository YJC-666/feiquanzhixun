#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
by:杨锦成
vx:18873833517
time:2025.5.29
file_name:nav_position_pkg.py
说明:无人机参数MC_YAW_P=0.48
'''
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Float32MultiArray
# from filterpy.kalman import KalmanFilter  # 删除卡尔曼滤波
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
import threading
import math

class SmoothedControl:
    def __init__(self, 
                 alpha_x=0.2, alpha_y=0.2, alpha_yaw=0.2,
                 window_size_x=5, window_size_y=5, window_size_yaw=5,
                 mode='lpf', # 'lpf'/'ma'/'adaptive'
                 threshold_x=0.0, threshold_y=0.0, threshold_yaw=0.0):
        """
        mode: 'lpf' 低通滤波, 'ma' 滑动平均, 'adaptive' 自适应低通
        threshold_*: 二值化阈值，>0启用
        """
        self.alpha_x = alpha_x
        self.alpha_y = alpha_y
        self.alpha_yaw = alpha_yaw
        self.window_size_x = window_size_x
        self.window_size_y = window_size_y
        self.window_size_yaw = window_size_yaw
        self.mode = mode
        self.threshold_x = threshold_x
        self.threshold_y = threshold_y
        self.threshold_yaw = threshold_yaw
        # 状态
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.x_window = []
        self.y_window = []
        self.yaw_window = []
        self.initialized = False

    def reset(self):
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.x_window = []
        self.y_window = []
        self.yaw_window = []
        self.initialized = False

    def _adaptive_alpha(self, base_alpha, value, last):
        # 简单自适应：变化大时alpha大，变化小时alpha小
        diff = abs(value - last)
        adaptive = min(1.0, max(0.05, base_alpha + diff * 0.2))
        return adaptive

    def _binarize(self, value, threshold):
        if abs(value) < threshold:
            return 0.0
        return value

    def filter(self, channel, value):
        if not self.initialized:
            self.last_x = value if channel == 'x' else self.last_x
            self.last_y = value if channel == 'y' else self.last_y
            self.last_yaw = value if channel == 'yaw' else self.last_yaw
            self.x_window = [self.last_x]
            self.y_window = [self.last_y]
            self.yaw_window = [self.last_yaw]
            self.initialized = True
        if channel == 'x':
            if self.mode == 'lpf':
                self.last_x = self.alpha_x * value + (1 - self.alpha_x) * self.last_x
                out = self.last_x
            elif self.mode == 'ma':
                self.x_window.append(value)
                if len(self.x_window) > self.window_size_x:
                    self.x_window.pop(0)
                out = np.mean(self.x_window)
            elif self.mode == 'adaptive':
                alpha = self._adaptive_alpha(self.alpha_x, value, self.last_x)
                self.last_x = alpha * value + (1 - alpha) * self.last_x
                out = self.last_x
            else:
                out = value
            return self._binarize(out, self.threshold_x)
        elif channel == 'y':
            if self.mode == 'lpf':
                self.last_y = self.alpha_y * value + (1 - self.alpha_y) * self.last_y
                out = self.last_y
            elif self.mode == 'ma':
                self.y_window.append(value)
                if len(self.y_window) > self.window_size_y:
                    self.y_window.pop(0)
                out = np.mean(self.y_window)
            elif self.mode == 'adaptive':
                alpha = self._adaptive_alpha(self.alpha_y, value, self.last_y)
                self.last_y = alpha * value + (1 - alpha) * self.last_y
                out = self.last_y
            else:
                out = value
            return self._binarize(out, self.threshold_y)
        elif channel == 'yaw':
            if self.mode == 'lpf':
                self.last_yaw = self.alpha_yaw * value + (1 - self.alpha_yaw) * self.last_yaw
                out = self.last_yaw
            elif self.mode == 'ma':
                self.yaw_window.append(value)
                if len(self.yaw_window) > self.window_size_yaw:
                    self.yaw_window.pop(0)
                out = np.mean(self.yaw_window)
            elif self.mode == 'adaptive':
                alpha = self._adaptive_alpha(self.alpha_yaw, value, self.last_yaw)
                self.last_yaw = alpha * value + (1 - alpha) * self.last_yaw
                out = self.last_yaw
            else:
                out = value
            return self._binarize(out, self.threshold_yaw)
        else:
            return value

class HoverController:
    def __init__(self, action_mode="参数", log_function=rospy.loginfo):
        self.goal_reached = False
        self.odom_recorded = False
        self.hover_position = None
        self.goal_status = None
        self.new_plan_detected = False
        self.odom_msg = None
        self.cmd_vel_active = True
        self.action_mode = action_mode
        self.log_function = log_function

        # 启动目标状态监听
        self.goal_thread = threading.Thread(target=self._goal_message_listener)
        self.goal_thread.daemon = True
        self.goal_thread.start()

        # 根据模式选择里程计订阅
        if self.action_mode in ["sim", "sim_nolog"]:
            self.odom_thread = threading.Thread(target=self._sim_odom_message_listener)
        else:
            self.odom_thread = threading.Thread(target=self._odom_message_listener)

        self.odom_thread.daemon = True
        self.odom_thread.start()

    def _goal_message_listener(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.goal_status = rospy.wait_for_message('/move_base/status', GoalStatusArray, timeout=0.5)
                self._check_for_new_plan(self.goal_status)
            except rospy.ROSException:
                continue
            rate.sleep()

    def _check_for_new_plan(self, status_msg):
        for status in status_msg.status_list:
            if status.status == 1:
                self.new_plan_detected = True
                self.log_function("检测到新路径规划，退出悬停模式，继续速度控制")
                return
        self.new_plan_detected = False

    def _odom_message_listener(self):
        self.log_function("订阅里程计话题：/mavros/local_position/odom")
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._odom_callback, queue_size=10)

    def _sim_odom_message_listener(self):
        self.log_function("订阅仿真里程计话题：/iris_0/mavros/local_position/odom")
        rospy.Subscriber('/iris_0/mavros/local_position/odom', Odometry, self._odom_callback, queue_size=10)

    def _odom_callback(self, msg):
        self.odom_msg = msg

    def get_current_yaw(self):
        """获取当前偏航角（弧度）"""
        if self.odom_msg:
            orientation = self.odom_msg.pose.pose.orientation
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            return self.quaternion_to_yaw(q)
        return 0.0

    @staticmethod
    def quaternion_to_yaw(q):
        """四元数转偏航角计算"""
        return math.atan2(2.0*(q[3]*q[2] + q[0]*q[1]),
                          1.0 - 2.0*(q[1]**2 + q[2]**2))

    def process_goal_status(self):
        """判断目标是否到达"""
        if self.goal_status:
            for status in self.goal_status.status_list:
                if status.status == 3:
                    self.log_function("目标已到达，准备进入悬停模式")
                    self.goal_reached = True
                    break

    def process_new_goal(self):
        """判断是否检测到新的目标规划"""
        if self.new_plan_detected:
            self.goal_reached = False
            self.odom_recorded = False
            self.hover_position = None
            self.log_function("检测到新目标，重置悬停状态，继续速度控制")

    def record_odom(self):
        """在目标到达后记录当前位置"""
        if self.odom_msg:
            if not self.odom_recorded and self.goal_reached:
                self.hover_position = (
                    self.odom_msg.pose.pose.position.x,
                    self.odom_msg.pose.pose.position.y
                )
                self.odom_recorded = True
                self.log_function(f"已记录悬停位置：x={self.hover_position[0]}, y={self.hover_position[1]}")

    def get_current_altitude(self):
        """获取当前无人机高度"""
        if self.odom_msg:
            return self.odom_msg.pose.pose.position.z
        return None

def degrees_to_radians(degrees):
    return -degrees * (math.pi / 180.0)

def publish_nav_goal_x_y_z_yaw_t_frame(
    x, y, z, yaw, t, frame,
    setpoint_pub, nav_goal_pub,
    log_function=rospy.loginfo,
    action_mode="参数",
    stop_thread_flag=None
):
    try:
        log_function(f"开始发布导航目标... [模式={action_mode}]")

        rate = rospy.Rate(15)
        target = PositionTarget()

        smooth_control = SmoothedControl(alpha_x=0.5, alpha_y=0.5, alpha_yaw=0.5, window_size_x=2, window_size_y=2, window_size_yaw=2, mode='lpf', threshold_x=0.0, threshold_y=0.0, threshold_yaw=0.0)
        hover_controller = HoverController(action_mode=action_mode, log_function=log_function)

        # 新增：用于存储从 /move_base_simple/goal 获取的目标坐标和偏航角
        goal_position = None
        goal_yaw = None

        def goal_callback(msg):
            nonlocal goal_position, goal_yaw
            # 提取目标位置
            goal_position = (msg.pose.position.x, msg.pose.position.y)
            # 提取目标偏航角（从四元数转换为 yaw）
            orientation = msg.pose.orientation
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            goal_yaw = hover_controller.quaternion_to_yaw(q)
            log_function(f"接收到新目标坐标：x={goal_position[0]}, y={goal_position[1]}, yaw={math.degrees(goal_yaw):.2f}°")

        # 订阅 /move_base_simple/goal 话题
        goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

        # 主动向/move_base_simple/goal发布PoseStamped
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map" if frame == "local" else "base_link"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        # 计算四元数 - 根据坐标系类型选择计算方式
        if frame == "local":  # map坐标系
            cy = math.cos(math.radians(-yaw) * 0.5)  # 添加负号修正yaw方向
            sy = math.sin(math.radians(-yaw) * 0.5)  # 添加负号修正yaw方向
        else:  # base_link坐标系
            cy = math.cos(math.radians(-yaw) * 0.5)  # base_link也加负号，方向与map一致
            sy = math.sin(math.radians(-yaw) * 0.5)
        
        cp = 1.0
        sp = 0.0
        cr = 1.0
        sr = 0.0
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # 连续多次发布，确保move_base/rviz能收到
        for _ in range(10):
            goal_pub.publish(pose_msg)
            rospy.sleep(0.05)
        
        # 根据坐标系类型输出不同的日志
        if frame == "local":
            log_function(f"已主动向/move_base_simple/goal发布PoseStamped: x={x}, y={y}, z={z}, yaw={yaw}(实际发送{-yaw})")
        else:
            log_function(f"已主动向/move_base_simple/goal发布PoseStamped: x={x}, y={y}, z={z}, yaw={yaw}")

        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(t)

        # 新增：统计进入"悬停模式条件"的次数
        hover_mode_entry_count = 0
        
        # 获取当前yaw - 只在循环外获取一次
        current_yaw = hover_controller.get_current_yaw()
        
        # 预先计算目标yaw
        if frame == "local":  # map坐标系
            target_yaw = math.radians(-yaw)  # 使用负号，修正yaw方向
        else:  # base_link坐标系
            target_yaw = current_yaw + math.radians(-yaw)  # base_link也加负号
            
        log_function(f"初始yaw: {math.degrees(current_yaw):.2f}°, 目标yaw: {math.degrees(target_yaw):.2f}°")

        while rospy.Time.now() < end_time:
            if rospy.is_shutdown():
                break
            if stop_thread_flag is not None and stop_thread_flag.is_set():
                log_function("nav 线程被中断")
                break

            # 处理目标状态和新目标检测
            hover_controller.process_goal_status()
            hover_controller.process_new_goal()

            # 获取 /cmd_vel，如果没有收到则认为指令停止
            try:
                cmd_vel_msg = rospy.wait_for_message('/cmd_vel', Twist, timeout=0.1)
                hover_controller.cmd_vel_active = True
                target.type_mask = 0b100111000011

                # 使用低通滤波平滑速度和偏航率
                target.velocity.x = smooth_control.filter('x', cmd_vel_msg.linear.x)
                target.velocity.y = smooth_control.filter('y', cmd_vel_msg.linear.y)
                target.yaw_rate   = smooth_control.filter('yaw', cmd_vel_msg.angular.z)
                target.position.z = z

                # 使用最新计算的目标yaw
                target.yaw = current_yaw
            except rospy.ROSException:
                hover_controller.cmd_vel_active = False

            # 获取当前高度
            current_altitude = hover_controller.get_current_altitude()

            # 获取当前位置并输出调试信息
            if hover_controller.odom_msg:
                current_x = hover_controller.odom_msg.pose.pose.position.x
                current_y = hover_controller.odom_msg.pose.pose.position.y
                
                # 获取实时yaw用于显示和日志（这不影响目标yaw的计算）
                display_current_yaw = hover_controller.get_current_yaw()
                current_yaw_deg = math.degrees(display_current_yaw)
                target_yaw_deg = math.degrees(target_yaw)
                
                # 计算与目标偏航角的差值（角度制）
                yaw_diff = abs((target_yaw_deg - current_yaw_deg + 180) % 360 - 180)
                
                # 计算与目标位置的距离
                distance = math.sqrt((current_x - x)**2 + (current_y - y)**2)
                
                log_function(f"当前位置: x={current_x:.3f}, y={current_y:.3f}, 距离目标: {distance:.3f}m, "
                         f"当前yaw: {current_yaw_deg:.2f}°, 目标yaw: {target_yaw_deg:.2f}°, yaw差值: {yaw_diff:.2f}°")

            # 如果 current_altitude 不为 None，则继续保持当前高度
            if current_altitude is not None:
                # 控制高度：通过计算当前高度与目标高度的差异来确定垂直速度
                altitude_error = z - current_altitude
                # 使用线性控制直接调整垂直速度
                vertical_velocity = np.clip(altitude_error * 2.0, -0.2, 0.2)
                target.velocity.z = vertical_velocity

            # 记录当前姿态用于悬停
            hover_controller.record_odom()

            # 处理悬停模式
            if (not hover_controller.cmd_vel_active
                and hover_controller.goal_reached
                and hover_controller.odom_recorded
                and hover_controller.hover_position
            ):
                hover_mode_entry_count += 1
                if hover_mode_entry_count >= 3:
                    target.type_mask = 0b100111111000
                    if goal_position:
                        target.position.x = goal_position[0]
                        target.position.y = goal_position[1]
                    else:
                        target.position.x = x
                        target.position.y = y

                    # 明确使用坐标系区分
                    if frame == "local":  # map坐标系
                        target.yaw = math.radians(-yaw)  # 使用负号，修正yaw方向
                    if frame == "body":  # base_link坐标系
                        # 注意：这里使用的是计算好的target_yaw，不是当前的yaw角度
                        target.yaw = target_yaw  # 已经在循环外计算过，包含了current_yaw + 目标偏航量

                    target.position.z = z
                    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    target.velocity.x = 0.0
                    target.velocity.y = 0.0
                else:
                    target.type_mask = 0b110111000011
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
            else:
                target.type_mask = 0b110111000011
                target.coordinate_frame = PositionTarget.FRAME_BODY_NED

            # 时间戳与发布
            target.header.stamp = rospy.Time.now()
            setpoint_pub.publish(target)

            # 打印详细的控制指令日志
            log_function(
                "控制指令：vx=%.2f, vy=%.2f, vz=%.2f, yaw_rate=%.2f | "
                "位置(x=%.2f, y=%.2f, z=%.2f), yaw=%.2f | "
                "type_mask=%s, frame=%s"
                % (
                    target.velocity.x, target.velocity.y, target.velocity.z, target.yaw_rate,
                    target.position.x, target.position.y, target.position.z, target.yaw,
                    bin(target.type_mask), 
                    "LOCAL_NED" if target.coordinate_frame == PositionTarget.FRAME_LOCAL_NED else "BODY_NED"
                )
            )

            rate.sleep()
    except Exception as e:
        log_function(f"导航过程中发生错误: {str(e)}")
        raise
    finally:
        # 清理订阅者
        if 'goal_subscriber' in locals():
            goal_subscriber.unregister()
        
        # 发送一个停止命令
        try:
            stop_target = PositionTarget()
            stop_target.header.stamp = rospy.Time.now()
            stop_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            stop_target.type_mask = 0b100111111000  # 使用位置控制
            stop_target.position.x = current_x if 'current_x' in locals() else x
            stop_target.position.y = current_y if 'current_y' in locals() else y
            stop_target.position.z = current_z if 'current_z' in locals() else z
            stop_target.yaw = target_yaw if 'target_yaw' in locals() else 0.0
            
            # 发布停止命令多次以确保接收
            for _ in range(3):
                setpoint_pub.publish(stop_target)
                rospy.sleep(0.05)
        except Exception as e:
            log_function(f"发送停止命令时发生错误: {str(e)}")

        log_function("导航任务结束，已清理相关资源")

def publish_nav_goal_x_y_z_yaw_tol_frame(
    x, y, z, yaw, tolerance,
    frame,
    setpoint_pub, nav_goal_pub,
    log_function=rospy.loginfo,
    action_mode="参数",
    stop_thread_flag=None,
    yaw_tolerance=0.5,  # 添加yaw容差参数，默认1.5度
):
    try:
        log_function(f"开始发布导航目标... [模式={action_mode}]")

        rate = rospy.Rate(15)
        target = PositionTarget()

        smooth_control = SmoothedControl(alpha_x=0.5, alpha_y=0.5, alpha_yaw=0.5, window_size_x=2, window_size_y=2, window_size_yaw=2, mode='lpf', threshold_x=0.0, threshold_y=0.0, threshold_yaw=0.0)
        hover_controller = HoverController(action_mode=action_mode, log_function=log_function)

        # 新增：用于存储从 /move_base_simple/goal 获取的目标坐标和偏航角
        goal_position = None
        goal_yaw = None

        def goal_callback(msg):
            nonlocal goal_position, goal_yaw
            # 提取目标位置
            goal_position = (msg.pose.position.x, msg.pose.position.y)
            # 提取目标偏航角（从四元数转换为 yaw）
            orientation = msg.pose.orientation
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            goal_yaw = hover_controller.quaternion_to_yaw(q)
            log_function(f"接收到新目标坐标：x={goal_position[0]}, y={goal_position[1]}, yaw={math.degrees(goal_yaw):.2f}°")

        # 订阅 /move_base_simple/goal 话题
        goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

        # 主动向/move_base_simple/goal发布PoseStamped
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map" if frame == "local" else "base_link"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        # 计算四元数 - 根据坐标系类型选择计算方式
        if frame == "local":  # map坐标系
            cy = math.cos(math.radians(-yaw) * 0.5)  # 添加负号修正yaw方向
            sy = math.sin(math.radians(-yaw) * 0.5)  # 添加负号修正yaw方向
        else:  # base_link坐标系
            cy = math.cos(math.radians(-yaw) * 0.5)  # base_link也加负号，方向与map一致
            sy = math.sin(math.radians(-yaw) * 0.5)
        
        cp = 1.0
        sp = 0.0
        cr = 1.0
        sr = 0.0
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # 连续多次发布，确保move_base/rviz能收到
        for _ in range(10):
            goal_pub.publish(pose_msg)
            rospy.sleep(0.05)
        
        # 根据坐标系类型输出不同的日志
        if frame == "local":
            log_function(f"已主动向/move_base_simple/goal发布PoseStamped: x={x}, y={y}, z={z}, yaw={yaw}(实际发送{-yaw})")
        else:
            log_function(f"已主动向/move_base_simple/goal发布PoseStamped: x={x}, y={y}, z={z}, yaw={yaw}")

        # 新增：统计进入"悬停模式条件"的次数
        hover_mode_entry_count = 0
        
        # 获取当前yaw - 只在循环外获取一次
        current_yaw = hover_controller.get_current_yaw()
        
        # 预先计算目标yaw
        if frame == "local":  # map坐标系
            target_yaw = math.radians(-yaw)  # 使用负号，修正yaw方向
        else:  # base_link坐标系
            target_yaw = current_yaw + math.radians(-yaw)  # base_link也加负号
            
        log_function(f"初始yaw: {math.degrees(current_yaw):.2f}°, 目标yaw: {math.degrees(target_yaw):.2f}°")

        while not rospy.is_shutdown():
            if stop_thread_flag is not None and stop_thread_flag.is_set():
                log_function("导航任务被中断")
                break

            # 获取当前位置
            if hover_controller.odom_msg:
                current_x = hover_controller.odom_msg.pose.pose.position.x
                current_y = hover_controller.odom_msg.pose.pose.position.y
                current_z = hover_controller.odom_msg.pose.pose.position.z
                
                # 获取实时yaw用于显示和检查容差（这不影响目标yaw的计算）
                display_current_yaw = hover_controller.get_current_yaw()
                current_yaw_deg = math.degrees(display_current_yaw)
                target_yaw_deg = math.degrees(target_yaw)
                
                # 计算与目标偏航角的差值（角度制）
                yaw_diff = abs((target_yaw_deg - current_yaw_deg + 180) % 360 - 180)
                
                # 计算与目标位置的距离
                distance = math.sqrt((current_x - x)**2 + (current_y - y)**2)
                
                log_function(f"当前位置: x={current_x:.3f}, y={current_y:.3f}, z={current_z:.3f}, "
                             f"距离目标: {distance:.3f}m, 当前yaw: {current_yaw_deg:.2f}°, "
                             f"目标yaw: {target_yaw_deg:.2f}°, yaw差值: {yaw_diff:.2f}°, "
                             f"容差: {yaw_tolerance:.2f}°, 位置容差: {tolerance:.3f}m")

                # 检查是否同时满足位置和偏航角容差要求
                if distance <= tolerance and yaw_diff <= yaw_tolerance:
                    log_function(f"到达目标位置和姿态！(位置误差: {distance:.3f}m, yaw误差: {yaw_diff:.2f}°)")
                    break
                elif distance <= tolerance:
                    log_function(f"已到达位置，等待姿态对齐 (yaw误差: {yaw_diff:.2f}°, 容差: {yaw_tolerance:.2f}°)")

            # 处理目标状态和新目标检测
            hover_controller.process_goal_status()
            hover_controller.process_new_goal()

            # 获取 /cmd_vel，如果没有收到则认为指令停止
            try:
                cmd_vel_msg = rospy.wait_for_message('/cmd_vel', Twist, timeout=0.1)
                hover_controller.cmd_vel_active = True
                target.type_mask = 0b100111000011

                # 使用低通滤波平滑速度和偏航率
                target.velocity.x = smooth_control.filter('x', cmd_vel_msg.linear.x)
                target.velocity.y = smooth_control.filter('y', cmd_vel_msg.linear.y)
                target.yaw_rate   = smooth_control.filter('yaw', cmd_vel_msg.angular.z)
                target.position.z = z

                # 使用最新计算的目标yaw
                target.yaw = current_yaw
            except rospy.ROSException:
                hover_controller.cmd_vel_active = False

            # 获取当前高度
            current_altitude = hover_controller.get_current_altitude()

            # 如果 current_altitude 不为 None，则继续保持当前高度
            if current_altitude is not None:
                # 控制高度：通过计算当前高度与目标高度的差异来确定垂直速度
                altitude_error = z - current_altitude
                # 使用线性控制直接调整垂直速度
                vertical_velocity = np.clip(altitude_error * 2.0, -0.2, 0.2)
                target.velocity.z = vertical_velocity

            # 记录当前姿态用于悬停
            hover_controller.record_odom()

            # 处理悬停模式
            if (not hover_controller.cmd_vel_active
                and hover_controller.goal_reached
                and hover_controller.odom_recorded
                and hover_controller.hover_position
            ):
                hover_mode_entry_count += 1
                if hover_mode_entry_count >= 3:
                    target.type_mask = 0b100111111000
                    if goal_position:
                        target.position.x = goal_position[0]
                        target.position.y = goal_position[1]
                    else:
                        target.position.x = x
                        target.position.y = y

                    # 明确使用坐标系区分
                    if frame == "local":  # map坐标系
                        target.yaw = math.radians(-yaw)  # 使用负号，修正yaw方向
                    if frame == "body":  # base_link坐标系
                        # 注意：这里使用的是计算好的target_yaw，不是当前的yaw角度
                        target.yaw = target_yaw  # 已经在循环外计算过，包含了current_yaw + 目标偏航量

                    target.position.z = z
                    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    target.velocity.x = 0.0
                    target.velocity.y = 0.0
                else:
                    target.type_mask = 0b110111000011
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
            else:
                target.type_mask = 0b110111000011
                target.coordinate_frame = PositionTarget.FRAME_BODY_NED

            # 时间戳与发布
            target.header.stamp = rospy.Time.now()
            setpoint_pub.publish(target)

            # 打印详细的控制指令日志
            log_function(
                "控制指令：vx=%.2f, vy=%.2f, vz=%.2f, yaw_rate=%.2f | "
                "位置(x=%.2f, y=%.2f, z=%.2f), yaw=%.2f | "
                "type_mask=%s, frame=%s"
                % (
                    target.velocity.x, target.velocity.y, target.velocity.z, target.yaw_rate,
                    target.position.x, target.position.y, target.position.z, target.yaw,
                    bin(target.type_mask), 
                    "LOCAL_NED" if target.coordinate_frame == PositionTarget.FRAME_LOCAL_NED else "BODY_NED"
                )
            )

            rate.sleep()

    except Exception as e:
        log_function(f"导航过程中发生错误: {str(e)}")
        raise
    finally:
        # 清理订阅者
        if 'goal_subscriber' in locals():
            goal_subscriber.unregister()
        
        # 发送一个停止命令
        try:
            stop_target = PositionTarget()
            stop_target.header.stamp = rospy.Time.now()
            stop_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            stop_target.type_mask = 0b100111111000  # 使用位置控制
            stop_target.position.x = current_x if 'current_x' in locals() else x
            stop_target.position.y = current_y if 'current_y' in locals() else y
            stop_target.position.z = current_z if 'current_z' in locals() else z
            stop_target.yaw = target_yaw if 'target_yaw' in locals() else 0.0
            
            # 发布停止命令多次以确保接收
            for _ in range(3):
                setpoint_pub.publish(stop_target)
                rospy.sleep(0.05)
        except Exception as e:
            log_function(f"发送停止命令时发生错误: {str(e)}")

        log_function("导航任务结束，已清理相关资源")


