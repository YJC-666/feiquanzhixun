#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
by:杨锦成
vx:18873833517
time:2025.5.29
file_name:action_t.py
说明:mavros_mask 0b010111000011 //选择需要的功能掩码，需要的给0,不需要的给1,从右往左对应PX PY PZ VX VY VZ AFX AFY AFZ FORCE YAW YAW_RATE
'''

from ros_initialization import init_ros_publishers, init_ros_services, init_ros_subscribers, init_ros_node, yolo_callback, odom_callback, sim_init_ros_publishers, sim_init_ros_services, sim_init_ros_subscribers
from nav_position_pkg import publish_nav_goal_x_y_z_yaw_t_frame, publish_nav_goal_x_y_z_yaw_tol_frame
from ros_initialization import current_position_x, current_position_y, current_position_z
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Path
from mavros_msgs.srv import CommandLongRequest
from pykalman import KalmanFilter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import rospy
import time
import math
from nav_msgs.msg import Odometry  # 导入 Odometry 消息类型
import tf
from std_msgs.msg import String
import math  # 添加此行
import threading
from std_msgs.msg import Int32, UInt16  # 添加UInt16用于舵机状态回调
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class Action_t:
    """
    类名称: Action_t
    类功能: 用于控制无人机的各种操作，包括解锁、上锁、陀螺仪热初始化和OFFBOARD模式切换
    """

    def __init__(self, ExecuteType):
        """
        函数名称: __init__
        函数功能: 初始化Action_t对象，根据执行类型设置ROS服务和订阅
        参数:
        - ExecuteType: 执行类型（"cheak"或"real"）
        使用案例:
        - 创建实际模式对象: action = Action_t("real")
        - 创建检查当前页面语法模式对象: action = Action_t("cheak")
        """
        versioninfo = "px4_control_realease_3.1"  # 版本信息
        print("版本信息:" + str(versioninfo))

        self.ActionExecuteType = ExecuteType  # 设置执行类型（仿真或真实）
        self.state = State()  # 初始化状态
        self.last_command = None  # 存储最后一次的控制命令
        self.offboard_mask = False  # OFFBOARD模式标志
        self.current_position_z = 0.0  # 当前高度
        self.current_position_x = 0.0  # 初始化当前位置X
        self.current_position_y = 0.0  # 初始化当前位置Y
        self.current_orientation = None  # 初始化为 None 以确保类型正确
        self.current_yaw = 0.0
        self.overtime = 0
        self.count = 0  # 添加count属性，用于周期性日志输出

        # --------------- 多线程初始化 -----------------#
        self.CONTROL_COMPLETE = False
        self.send_position_thread_running = False
        self.hover_thread_running = False
        self.unlock_thread_running = False
        self.lock_thread_running = False
        self.takeoff_thread_running = False
        self.land_auto_thread_running = False
        self.nav_goal_thread_running = False
        self.control_yaw_thread_running = False
        self.velocity_thread_running = False
        self.land_lock_thread_running = False
        self.control_points_thread_running = False
        self.position_yaw_thread_running = False
        self.circle_thread_running = False
        self.track_velocity_thread_running = False
        self.time_sleep_thread_running = None
        self.tracking_thread_running = False  # 添加tracking_thread_running属性
        self.send_position_yaw_thread_running = False
        self.track_velocity_z_centertol_tout_t_thread_running = False  # 添加track_velocity_z_centertol_tout_t_thread_running属性
        self.track_velocity_direction_thread_running = False  # 添加track_velocity_direction_thread_running属性
        


        self.send_position_thread = None
        self.hover_thread = None
        self.unlock_thread = None
        self.lock_thread = None
        self.takeoff_thread = None
        self.land_auto_thread = None
        self.nav_goal_thread = None
        self.control_yaw_thread = None
        self.velocity_thread = None
        self.land_lock_thread = None
        self.control_points_thread = None
        self.position_yaw_thread = None
        self.circle_thread = None
        self.track_velocity_thread = None
        self.time_sleep_thread = None
        self.tracking_thread = None  # 添加tracking_thread属性
        self.send_position_yaw_thread = None
        self.track_velocity_z_centertol_tout_t_thread = None  # 添加track_velocity_z_centertol_tout_t_thread属性
        self.track_velocity_direction_thread = None  # 添加track_velocity_direction_thread属性

        
        self.stop_thread_flag = threading.Event()  # 终止标志
        # ---------------------------------------------#

        if self.ActionExecuteType == "real" or self.ActionExecuteType == "real_nolog":
            # 设置不使用仿真时间
            rospy.set_param('/use_sim_time', False)

            # 初始化ROS节点（若节点尚未初始化）
            init_ros_node('drone_controller')
            print("ROS node initialized.")

            # 初始化发布器与服务
            self.setpoint_pub, self.nav_goal_pub = init_ros_publishers()
            self.arming_service, self.set_mode_service, self.command_service = init_ros_services()

            # 初始化订阅
            init_ros_subscribers(self)
            rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback, callback_args=self, queue_size=10)
            rospy.Subscriber('/ai_detect_info', String, yolo_callback, callback_args=self, queue_size=30)
            
        elif self.ActionExecuteType == "sim" or self.ActionExecuteType == "sim_nolog":
             # 设置使用仿真时间
            rospy.set_param('/use_sim_time', True)

            # 初始化ROS节点（若节点尚未初始化）
            init_ros_node('drone_controller')
            print("ROS node initialized.")

            # 初始化发布器与服务
            self.setpoint_pub, self.nav_goal_pub = sim_init_ros_publishers()
            self.arming_service, self.set_mode_service, self.command_service = sim_init_ros_services()

            # 初始化订阅
            sim_init_ros_subscribers(self)
            rospy.Subscriber('/iris_0/mavros/local_position/odom', Odometry, odom_callback, callback_args=self, queue_size=10)
            rospy.Subscriber('/ai_detect_info', String, yolo_callback, callback_args=self, queue_size=30)


    
  

    # ----------------- 日志输出封装，用于在 "nolog" 时屏蔽 ----------------- #
    def log_info(self, msg):
        if self.ActionExecuteType != "real_nolog" and self.ActionExecuteType != "sim_nolog":
            print(self.ActionExecuteType)
            rospy.loginfo(msg)

    def log_warn(self, msg):
        if self.ActionExecuteType != "real_nolog" and self.ActionExecuteType != "sim_nolog":
            rospy.logwarn(msg)

    def log_err(self, msg):
        if self.ActionExecuteType != "real_nolog" and self.ActionExecuteType != "sim_nolog":
            rospy.logerr(msg)

    def log_debug(self, msg):
        if self.ActionExecuteType != "real_nolog" and self.ActionExecuteType != "sim_nolog":
            rospy.logdebug(msg)
    # --------------------------------------------------------------------- #

    # ----------------- 无人机数据回调 ----------------- #
        
    def control_complete(self, value=None):
            if value is not None:
                self.CONTROL_COMPLETE = value
            return self.CONTROL_COMPLETE
    
    def get_current_xyz(self):
        """
        函数名称: get_current_xyz
        函数功能: 获取无人机当前的X和Y坐标。
        返回值:
        - (x, y, z): 当前的X和Y坐标。
        """
        # 等待回调函数获取位置数据
        rospy.sleep(0.1)  # 等待数据更新
        # 确保数据正确性
        if hasattr(self, 'current_position_x') and hasattr(self, 'current_position_y') and hasattr(self, 'current_position_z'):
            self.log_debug(f"当前坐标: x={self.current_position_x}, y={self.current_position_y}, z={self.current_position_z}")
            return self.current_position_x, self.current_position_y, self.current_position_z
        
    def get_current_yaw(self):
        """
        函数名称: get_current_yaw
        函数功能: 获取无人机当前的 yaw 角度（欧拉角形式）。
        返回值:
        - yaw: 当前 yaw 角度，单位为弧度。
        使用案例:
        - yaw = action.get_current_yaw()  # 获取当前 yaw 角度
        """
        # 等待回调函数获取姿态数据
        rospy.sleep(0.1)  # 等待数据更新

        # 确保 yaw 角度已经被更新
        if hasattr(self, 'current_yaw'):
            self.log_debug(f"当前 yaw 角度: {self.current_yaw} (弧度)")
            return self.current_yaw
        else:
            self.log_warn("当前未获取到 yaw 角度，默认返回 yaw=0.0")
            return 0.0
    
    def get_current_yolo_xy(self):
        """
        函数名称: get_current_yolo_xy
        函数功能: 获取当前目标的X和Y坐标
        返回值:
        - (yolo_x, yolo_y): 目标的X和Y坐标，未检测到目标时返回(-1.0, -1.0)
        """
        # 检查属性是否存在，不存在则初始化为-1
        if not hasattr(self, 'yolo_x'):
            self.yolo_x = -1.0
        if not hasattr(self, 'yolo_y'):
            self.yolo_y = -1.0
        
        return self.yolo_x, self.yolo_y
    
    def get_current_yolo_class(self):
        """
        获取当前检测到的目标类别。
        返回值:
        - 类别字符串，如 'circle'，未检测到则返回 'none'
        """
        if not hasattr(self, 'yolo_class'):
            self.yolo_class = 'none'
        return self.yolo_class

    def get_yolo_result_width(self):
        """
        函数名称: get_yolo_result_width
        函数功能: 获取当前目标的宽度
        返回值:
        - (width): 目标的宽度，未检测到目标时返回-1.0
        """
        # 检查属性是否存在，不存在则初始化为-1
        if not hasattr(self, 'yolo_result_width'):
            self.yolo_result_width = -1.0
        
        return self.yolo_result_width
    
    def get_yolo_result_height(self):
        """
        函数名称: get_yolo_result_height
        函数功能: 获取当前目标的高度
        返回值:
        - (height): 目标的高度，未检测到目标时返回-1.0
        """
        # 检查属性是否存在，不存在则初始化为-1
        if not hasattr(self, 'yolo_result_height'):
            self.yolo_result_height = -1.0
        
        return self.yolo_result_height

    # ------------------------------------------------- #

    # ----------------- 无人机控制的返回标志集合函数 ----------------- #
    def found_target(self):
        """
        函数名称: found_target
        函数功能: 飞行过程中这个函数会被循环调用，用于检查是否检测到目标。
                  若当前获取的目标坐标 (yolo_x, yolo_y) 都为 -1.0，则表示未检测到目标，返回 False；
                  否则，表示检测到目标，返回 True。
        返回值:
        - bool: 若当前目标坐标为 (-1.0, -1.0) 则返回 False，否则返回 True。
        """
        yolo_x, yolo_y = self.get_current_yolo_xy()
        if yolo_x != -1.0 and yolo_y != -1.0:
            self.log_info(f"检测到目标: x={yolo_x}, y={yolo_y}")
            return True
        return False

    # ---------------------------------------------------------------#
    def _time_sleep(self, duration):
        """
        使用 rospy.Rate 实现类似 time.sleep 的功能。

        参数:
        - duration: 延迟时间，单位秒。
        """
        if duration <= 0:
            return
        
        rate = rospy.Rate(1.0 / duration)  # 设置频率为 1/duration Hz
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.stop_thread_flag.is_set():
                rospy.logwarn("_time_sleep 线程被中断")
                break
            rate.sleep()  # 等待一个周期





    # ------------------------位置控制---------------------------------------#
    def _send_position_x_y_z_t_frame(self, x, y, z, duration, frame="local"):
        """
        函数名称: _send_position_x_y_z_t_frame
        函数功能: 在指定时间内循环发送位置目标，支持不同坐标系
        参数:
        - x: 目标位置X坐标
        - y: 目标位置Y坐标
        - z: 目标位置Z坐标
        - duration: 发送位置目标的时间持续时长（秒）
        - frame: 坐标系类型，"local"表示本地坐标系，"body"表示机体坐标系
                 也可以使用"body_v=速度值"格式指定机体坐标系下的速度，如"body_v=0.3"
                 或使用"local_v=速度值"格式指定本地坐标系下的速度，如"local_v=0.3"
                 或使用"body_vx_vy_vz=0.3_0.3_0.3"格式指定机体坐标系下的各轴速度
                 或使用"local_vx_vy_vz=0.3_0.3_0.3"格式指定本地坐标系下的各轴速度
        """
        if self.ActionExecuteType == "cheak":
            # 检查参数是否合法
            try:
                # 检查坐标值是否为数值类型
                if not all(isinstance(val, (int, float)) for val in [x, y, z, duration]):
                    print("\033[91m错误: 坐标和持续时间必须是数值类型, 当前值: x={}, y={}, z={}, duration={}\033[0m".format(x, y, z, duration))
                    print("建议: 请确保所有输入参数都是数字类型(整数或浮点数)")
                    return False
                
                # 检查持续时间是否为正数
                if duration <= 0:
                    print("\033[91m错误: 持续时间必须大于0, 当前值: {}\033[0m".format(duration))
                    print("建议: 持续时间建议设置在1-30秒之间，过短可能导致控制不稳定，过长可能造成任务延迟")
                    return False
                
                # 检查坐标范围是否合理
                if abs(x) > 100 or abs(y) > 100 or abs(z) > 100:
                    print(f"警告: 目标位置可能超出安全范围: x={x}, y={y}, z={z}")
                    print("建议: 建议将目标位置控制在±100米范围内，确保飞行安全")
                
                # 解析frame参数
                frame_type = frame.lower()
                if frame_type.startswith("body_v=") or frame_type.startswith("local_v="):
                    try:
                        speed_str = frame_type.split("=")[1]
                        speed = float(speed_str)
                        if speed <= 0:
                            print("\033[91m错误: 速度必须大于0, 当前值: {}\033[0m".format(speed))
                            print("建议: 速度建议设置在0.1-3.0 m/s之间，过慢影响效率，过快可能不稳定")
                            return False
                        elif speed > 3.0:
                            print(f"警告: 速度值较大: {speed} m/s")
                            print("建议: 建议将速度控制在3.0 m/s以下，以确保飞行安全")
                    except (IndexError, ValueError):
                        print("\033[91m错误: 速度格式不正确, 当前值: {}\033[0m".format(frame))
                        print("建议: 正确格式示例: 'body_v=0.5' 或 'local_v=0.5'")
                        return False
                elif frame_type.startswith("body_vx_vy_vz=") or frame_type.startswith("local_vx_vy_vz="):
                    try:
                        speeds_str = frame_type.split("=")[1]
                        vx, vy, vz = map(float, speeds_str.split("_"))
                        if any(speed <= 0 for speed in [vx, vy, vz]):
                            print("\033[91m错误: 各轴速度必须大于0, 当前值: vx={}, vy={}, vz={}\033[0m".format(vx, vy, vz))
                            print("建议: 各轴速度建议设置在0.1-3.0 m/s之间")
                            return False
                        elif any(speed > 3.0 for speed in [vx, vy, vz]):
                            print(f"警告: 存在较大的轴向速度: vx={vx}, vy={vy}, vz={vz}")
                            print("建议: 建议将各轴速度都控制在3.0 m/s以下")
                    except (IndexError, ValueError):
                        print("\033[91m错误: 速度格式不正确, 当前值: {}\033[0m".format(frame))
                        print("建议: 正确格式示例: 'body_vx_vy_vz=0.5_0.5_0.5' 或 'local_vx_vy_vz=0.5_0.5_0.5'")
                        return False
                elif frame_type not in ["local", "body"]:
                    print(f"警告: 未识别的坐标系类型: {frame_type}, 将使用默认的local方式")
                    print("建议: 建议使用 'local' 或 'body' 作为坐标系类型")
                
                # 模拟打印运行信息
                print(f"检查模式: 发送位置 ({x}, {y}, {z}), 持续时间: {duration}秒, 坐标系: {frame}")
                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True
                
            except Exception as e:
                print("\033[91m参数检查过程中发生错误: {}\033[0m".format(str(e)))
                print("建议: 请检查所有输入参数的格式是否正确")
                self.control_complete(True)
                return False
                
        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    self.control_complete(True)
                    return
                    
            hold_yaw = self.get_current_yaw()  # 保存当前yaw角度
            start_time = rospy.Time.now()
            rate = rospy.Rate(20)
            
            # 解析frame参数，检查是否包含速度设置
            frame_type = frame.lower()
            default_speed = 0.3  # 默认速度 m/s
            vx_speed = vy_speed = vz_speed = default_speed  # 默认各轴速度相同
            
            if frame_type.startswith("body_v="):
                try:
                    speed_str = frame_type.split("=")[1]
                    default_speed = float(speed_str)
                    vx_speed = vy_speed = vz_speed = default_speed
                    frame_type = "body"
                    self.log_info(f"使用机体坐标系，自定义速度: {default_speed} m/s")
                except (IndexError, ValueError):
                    self.log_warn(f"速度格式错误，使用默认速度: {default_speed} m/s")
                    frame_type = "body"
            elif frame_type.startswith("local_v="):
                try:
                    speed_str = frame_type.split("=")[1]
                    default_speed = float(speed_str)
                    vx_speed = vy_speed = vz_speed = default_speed
                    frame_type = "local"
                    self.log_info(f"使用本地坐标系，自定义速度: {default_speed} m/s")
                except (IndexError, ValueError):
                    self.log_warn(f"速度格式错误，使用默认速度: {default_speed} m/s")
                    frame_type = "local"
            elif frame_type.startswith("body_vx_vy_vz="):
                try:
                    speeds_str = frame_type.split("=")[1]
                    vx_speed, vy_speed, vz_speed = map(float, speeds_str.split("_"))
                    frame_type = "body"
                    self.log_info(f"使用机体坐标系，自定义各轴速度: vx={vx_speed} m/s, vy={vy_speed} m/s, vz={vz_speed} m/s")
                except (IndexError, ValueError):
                    self.log_warn(f"速度格式错误，使用默认速度: {default_speed} m/s")
                    frame_type = "body"
            elif frame_type.startswith("local_vx_vy_vz="):
                try:
                    speeds_str = frame_type.split("=")[1]
                    vx_speed, vy_speed, vz_speed = map(float, speeds_str.split("_"))
                    frame_type = "local"
                    self.log_info(f"使用本地坐标系，自定义各轴速度: vx={vx_speed} m/s, vy={vy_speed} m/s, vz={vz_speed} m/s")
                except (IndexError, ValueError):
                    self.log_warn(f"速度格式错误，使用默认速度: {default_speed} m/s")
                    frame_type = "local"
            
            # 根据frame参数设置不同的坐标系
            if frame_type == "body":
                self.log_info(f"使用机体坐标系(FRAME_BODY_NED)控制")
                
                # 计算需要飞行的距离
                distance = np.sqrt(x**2 + y**2 + z**2)
                
                # 如果距离为0，不需要飞行
                if distance > 0:
                    # 计算归一化的速度分量
                    vx_norm = (x / distance) * vx_speed if x != 0 else 0
                    vy_norm = (y / distance) * vy_speed if y != 0 else 0
                    vz_norm = (z / distance) * vz_speed if z != 0 else 0
                    
                    # 计算预计飞行时间
                    flight_time = max(abs(x) / vx_speed if x != 0 else 0, 
                                     abs(y) / vy_speed if y != 0 else 0,
                                     abs(z) / vz_speed if z != 0 else 0)
                    
                    # 使用实际的duration或计算的flight_time中较大的值
                    actual_duration = max(duration, flight_time)
                else:
                    vx_norm = 0
                    vy_norm = 0
                    vz_norm = 0
                    actual_duration = duration
                
                # 记录初始位置
                initial_x, initial_y, initial_z = self.get_current_xyz()
                traveled_distance = 0
                
                while (rospy.Time.now() - start_time).to_sec() < actual_duration:
                    if self.stop_thread_flag.is_set():
                       rospy.logwarn("send_position_x_y_z_t_frame 线程被中断")
                       break
                    
                    # 获取当前yaw和高度
                    current_yaw = self.get_current_yaw()
                    current_x, current_y, current_z = self.get_current_xyz()
                    
                    # 计算已经飞行的距离（在机体坐标系下的投影距离）
                    # 这里简化计算，使用欧氏距离作为近似
                    dx = current_x - initial_x
                    dy = current_y - initial_y
                    
                    # 将dx和dy旋转到初始机体坐标系下
                    initial_yaw = hold_yaw
                    cos_yaw = np.cos(initial_yaw)
                    sin_yaw = np.sin(initial_yaw)
                    dx_body = dx * cos_yaw + dy * sin_yaw
                    dy_body = -dx * sin_yaw + dy * cos_yaw
                    
                    # 计算在x和y方向上的飞行距离
                    traveled_x = dx_body
                    traveled_y = dy_body
                    
                    # 检查是否已经接近目标距离（在0.1m误差范围内）
                    x_close = abs(traveled_x - x) < 0.1 if x != 0 else True
                    y_close = abs(traveled_y - y) < 0.1 if y != 0 else True
                    
                    # 计算yaw误差和yaw_rate
                    yaw_error = hold_yaw - current_yaw
                    # 标准化yaw误差到[-pi, pi]
                    if yaw_error > np.pi:
                        yaw_error -= 2 * np.pi
                    elif yaw_error < -np.pi:
                        yaw_error += 2 * np.pi
                    yaw_rate = np.clip(yaw_error * 1.0, -0.5, 0.5)  # 使用比例控制，限制最大旋转速率
                    
                    # 对于z轴高度，当高度接近时使用更精细的控制
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 2.0, -0.2, 0.2)  # 使用与nav_position_pkg.py相同的控制逻辑
                    
                    target = PositionTarget()
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                    target.type_mask = 0b011111000111  # 使用速度和yaw_rate控制
                    
                    # 如果在x或y方向上已经接近目标，则停止对应方向的移动
                    target.velocity.x = 0 if x_close else vx_norm
                    target.velocity.y = 0 if y_close else vy_norm
                    target.velocity.z = vertical_velocity if abs(altitude_error) < 0.5 else vz_norm  # 当高度接近时使用更精确的控制
                    target.yaw_rate = yaw_rate
                    
                    if self.setpoint_pub:
                        self.setpoint_pub.publish(target)
                    
                    self.log_info(f"Real模式: 发布速度指令 vx={target.velocity.x:.2f}, vy={target.velocity.y:.2f}, vz={target.velocity.z:.2f}, 已飞行距离x={traveled_x:.2f}, y={traveled_y:.2f}, 目标Yaw={hold_yaw}, 当前Yaw={current_yaw:.2f}, Yaw误差={yaw_error:.2f}, Yaw_rate={yaw_rate:.2f}, 坐标系: {frame}")
                    rate.sleep()
            elif frame_type == "local":  # 使用本地坐标系
                self.log_info(f"使用本地坐标系(FRAME_LOCAL_NED)控制")
                
                # 获取当前位置
                current_x, current_y, current_z = self.get_current_xyz()
                
                # 计算预计飞行时间
                dx = x - current_x
                dy = y - current_y
                dz = z - current_z
                
                # 计算总距离
                distance = np.sqrt(dx**2 + dy**2 + dz**2)
                
                # 计算预计飞行时间
                flight_time = max(abs(dx) / vx_speed if dx != 0 else 0,
                                 abs(dy) / vy_speed if dy != 0 else 0,
                                 abs(dz) / vz_speed if dz != 0 else 0)
                
                # 使用实际的duration或计算的flight_time中较大的值
                actual_duration = max(duration, flight_time)
                
                while (rospy.Time.now() - start_time).to_sec() < actual_duration:
                    if self.stop_thread_flag.is_set():
                       rospy.logwarn("send_position_x_y_z_t_frame 线程被中断")
                       break
                    
                    # 获取当前位置和yaw
                    current_x, current_y, current_z = self.get_current_xyz()
                    current_yaw = self.get_current_yaw()
                    
                    # 计算位置误差
                    dx = x - current_x
                    dy = y - current_y
                    dz = z - current_z
                    
                    # 计算当前距离
                    current_distance = np.sqrt(dx**2 + dy**2 + dz**2)
                    
                    # 如果已经非常接近目标，则直接发送位置指令
                    if current_distance < 0.1:
                        target = PositionTarget()
                        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                        target.type_mask = 0b101111111000  # 使用位置和yaw控制
                        target.position.x = x
                        target.position.y = y
                        target.position.z = z
                        target.yaw = hold_yaw
                    else:
                        # 计算x和y轴速度，考虑方向和最大速度限制
                        vx = np.clip(dx, -vx_speed, vx_speed) if abs(dx) > 0.1 else 0
                        vy = np.clip(dy, -vy_speed, vy_speed) if abs(dy) > 0.1 else 0
                        
                        target = PositionTarget()
                        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                        target.type_mask = 0b100111000011  # 使用速度、位置和yaw控制
                        target.velocity.x = vx
                        target.velocity.y = vy
                        target.position.z = z  # z轴使用位置控制
                        target.yaw = hold_yaw  # 使用位置yaw控制
                    
                    if self.setpoint_pub:
                        self.setpoint_pub.publish(target)
                    
                    self.log_info(f"Real模式: 发布指令, 目标位置=({x}, {y}, {z}), 当前位置=({current_x:.2f}, {current_y:.2f}, {current_z:.2f}), 距离={current_distance:.2f}, 目标Yaw={hold_yaw}, 当前Yaw={current_yaw:.2f}, 坐标系: {frame}")
                    rate.sleep()
            else:  # 处理未识别的坐标系类型，使用默认的local方式
                self.log_warn(f"未识别的坐标系类型: {frame_type}，使用默认的local方式")
                
                # 获取当前位置
                current_x, current_y, current_z = self.get_current_xyz()
                
                # 计算需要飞行的距离
                dx = x - current_x
                dy = y - current_y
                dz = z - current_z
                
                # 计算预计飞行时间
                flight_time = max(abs(dx) / vx_speed if dx != 0 else 0,
                                 abs(dy) / vy_speed if dy != 0 else 0,
                                 abs(dz) / vz_speed if dz != 0 else 0)
                
                # 使用实际的duration或计算的flight_time中较大的值
                actual_duration = max(duration, flight_time)
                
                while (rospy.Time.now() - start_time).to_sec() < actual_duration:
                    if self.stop_thread_flag.is_set():
                       rospy.logwarn("send_position_x_y_z_t_frame 线程被中断")
                       break
                    
                    # 获取当前位置和yaw
                    current_x, current_y, current_z = self.get_current_xyz()
                    current_yaw = self.get_current_yaw()
                    
                    # 计算位置误差
                    dx = x - current_x
                    dy = y - current_y
                    dz = z - current_z
                    
                    # 计算当前距离
                    current_distance = np.sqrt(dx**2 + dy**2 + dz**2)
                    
                    # 如果已经非常接近目标，则直接发送位置指令
                    if current_distance < 0.1:
                        target = PositionTarget()
                        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                        target.type_mask = 0b101111111000  # 使用位置和yaw控制
                        target.position.x = x
                        target.position.y = y
                        target.position.z = z
                        target.yaw = hold_yaw
                    else:
                        # 计算x和y轴速度，考虑方向和最大速度限制
                        vx = np.clip(dx, -vx_speed, vx_speed) if abs(dx) > 0.1 else 0
                        vy = np.clip(dy, -vy_speed, vy_speed) if abs(dy) > 0.1 else 0
                        
                        target = PositionTarget()
                        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                        target.type_mask = 0b101111000011  # 使用速度、位置和yaw控制
                        target.velocity.x = vx
                        target.velocity.y = vy
                        target.position.z = z  # z轴使用位置控制
                        target.yaw = hold_yaw  # 使用位置yaw控制
                    
                    if self.setpoint_pub:
                        self.setpoint_pub.publish(target)
                    
                    self.log_info(f"Real模式: 发布指令, 目标位置=({x}, {y}, {z}), 当前位置=({current_x:.2f}, {current_y:.2f}, {current_z:.2f}), 距离={current_distance:.2f}, 目标Yaw={hold_yaw}, 当前Yaw={current_yaw:.2f}, 坐标系: {frame}")
                    rate.sleep()
            
            self.control_complete(True)  # 设置控制完成标志

    def send_position_x_y_z_t_frame(self, x, y, z, duration, frame="local", use_thread=False):
        """
        函数名称: send_position_x_y_z_t_frame
        函数功能: 在指定时间内将无人机移动到指定位置，支持不同坐标系
        参数:
        - x: 目标位置X坐标
        - y: 目标位置Y坐标
        - z: 目标位置Z坐标
        - duration: 移动持续时间（秒）
        - frame: 坐标系类型，"local"表示本地坐标系，"body"表示机体坐标系
                 也可以使用"body_v=速度值"格式指定机体坐标系下的速度，如"body_v=0.3"
                 或使用"local_v=速度值"格式指定本地坐标系下的速度，如"local_v=0.3"
                 或使用"body_vx_vy_vz=0.3_0.3_0.3"格式指定机体坐标系下的各轴速度
                 或使用"local_vx_vy_vz=0.3_0.3_0.3"格式指定本地坐标系下的各轴速度
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 在本地坐标系移动到(0, 0, 5)位置，持续10秒: action.send_position_x_y_z_t_frame(0, 0, 5, 10, "local")
        - 在本地坐标系移动到(0, 0, 5)位置，速度设为0.5m/s: action.send_position_x_y_z_t_frame(0, 0, 5, 10, "local_v=0.5")
        - 在本地坐标系移动到(0, 0, 5)位置，各轴速度分别设为0.3,0.4,0.5m/s: action.send_position_x_y_z_t_frame(0, 0, 5, 10, "local_vx_vy_vz=0.3_0.4_0.5")
        - 在机体坐标系前进5米: action.send_position_x_y_z_t_frame(5, 0, 0, 10, "body")
        - 在机体坐标系前进5米，速度设为0.5m/s: action.send_position_x_y_z_t_frame(5, 0, 0, 10, "body_v=0.5")
        - 在机体坐标系前进5米，各轴速度分别设为0.3,0.4,0.5m/s: action.send_position_x_y_z_t_frame(5, 0, 0, 10, "body_vx_vy_vz=0.3_0.4_0.5")
        - 使用线程方式移动: action.send_position_x_y_z_t_frame(0, 0, 5, 10, "local", use_thread=True)
        """
        try:
            if self.ActionExecuteType == "cheak":
                # 检查模式下，直接调用内部函数进行参数检查，不需要实际执行
                return self._send_position_x_y_z_t_frame(x, y, z, duration, frame)
            elif use_thread:
                self.thread_send_position_x_y_z_t_frame(x, y, z, duration, frame, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.send_position_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 send_position_x_y_z_t_frame 线程")
                    self.thread_send_position_x_y_z_t_frame(x, y, z, duration, frame, use_thread=False)
                
                # 直接执行位置控制函数
                return self._send_position_x_y_z_t_frame(x, y, z, duration, frame)
        except Exception as e:
            self.log_err(f"位置控制过程中发生错误: {str(e)}")
            self.send_position_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_send_position_x_y_z_t_frame(self, x, y, z, duration, frame="local", use_thread=False):
        """
        线程管理函数，用于启动或停止位置控制线程，支持不同坐标系
        """
        if self.ActionExecuteType == "cheak":
            # 检查模式下，直接打印线程操作信息，不需要实际执行
            if use_thread:
                print(f"检查模式: 启动位置控制线程 x={x}, y={y}, z={z}, 持续={duration}秒, 坐标系={frame}")
            else:
                print(f"检查模式: 停止位置控制线程")
            return True
            
        try:
            if use_thread:
                # 启动线程
                if self.send_position_thread_running:
                    rospy.logwarn("位置控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.send_position_thread and self.send_position_thread.is_alive():
                        self.send_position_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.send_position_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.send_position_thread = threading.Thread(
                        target=self._send_position_x_y_z_t_frame,
                        args=(x, y, z, duration, frame)
                    )
                    self.send_position_thread.daemon = True  # 设为守护线程
                    self.send_position_thread.start()
                    rospy.loginfo(f"位置控制线程已启动: x={x}, y={y}, z={z}, 持续={duration}s, 坐标系={frame}")
                except Exception as e:
                    rospy.logerr(f"启动位置控制线程失败: {str(e)}")
                    self.send_position_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.send_position_thread_running:
                    self.send_position_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止位置控制线程...")
                    if self.send_position_thread and self.send_position_thread.is_alive():
                        self.send_position_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("位置控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的位置控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.send_position_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------位置控制---------------------------------------#




    # ------------------------延时控制---------------------------------------#
    def _hover_delay_t(self, delay_s):
        """
        函数名称: hover_delay_t
        函数功能: 在指定延迟时间内保持当前位置和旋转角度，持续发布控制指令
        参数:
        - delay_s: 延迟时间，单位秒
        使用案例:
        - 延时10秒，并在此期间保持当前位置和航向角: action.hover_delay_t(10)
        """
        if self.ActionExecuteType == "cheak":
            # 检查参数是否合法
            try:
                # 检查延迟时间是否为数值类型
                if not isinstance(delay_s, (int, float)):
                    print(f"\033[91m错误: 延迟时间必须是数值类型, 当前值: {delay_s}\033[0m")
                    print("建议: 请输入整数或浮点数类型的延迟时间")
                    return False
                
                # 检查延迟时间是否为正数
                if delay_s <= 0:
                    print(f"\033[91m错误: 延迟时间必须大于0, 当前值: {delay_s}\033[0m")
                    print("建议: 延迟时间建议设置在1-30秒之间，过短可能导致控制不稳定，过长可能造成任务延迟")
                    return False
                
                # 检查延迟时间是否过长
                if delay_s > 30:
                    print(f"警告: 延迟时间较长: {delay_s}秒")
                    print("建议: 建议将延迟时间控制在30秒以内，过长的延迟可能影响任务执行效率")
                
                # 模拟打印运行信息
                print(f"检查模式: 悬停延时 {delay_s} 秒")
                print("提示: 在实际运行时，无人机将在当前位置保持悬停，并维持当前航向角")
                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True
                
            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                self.control_complete(True)
                return False
                
        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    self.control_complete(True)
                    return
                    
            rate = rospy.Rate(20)  # 设置频率为20Hz
            start_time = rospy.Time.now() + rospy.Duration(delay_s)  # 计算结束时间
            # 在函数开始时读取一次当前位置和yaw角度
            current_x, current_y, current_z = self.get_current_xyz()  # 获取当前X和Y坐标
            hold_yaw = self.get_current_yaw()  # 获取当前yaw角度

            # 在延时期间持续发布当前位置和当前yaw角度
            while rospy.Time.now() < start_time:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("hover_delay_t 线程被中断")
                    break
                    
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.type_mask = 0b101111000111  
                target.velocity.x = 0
                target.velocity.y = 0
                target.velocity.z = 0
                target.yaw = hold_yaw  # 使用当前yaw角度保持旋转角

                # 发布位置和yaw角度控制指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)

                rate.sleep()  # 保持20Hz的频率
            
            self.control_complete(True)  # 设置控制完成标志

    def hover_delay_t(self, delay_s, use_thread=False):
        """
        函数名称: hover_delay_t
        函数功能: 在当前位置悬停指定时间
        参数:
        - delay_s: 悬停时间，单位秒
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 悬停5秒: action.hover_delay_t(5)
        - 使用线程方式悬停: action.hover_delay_t(5, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_hover_delay_t(delay_s, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.hover_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 hover_delay_t 线程")
                    self.thread_hover_delay_t(delay_s, use_thread=False)
                
                # 直接执行悬停函数
                return self._hover_delay_t(delay_s)
        except Exception as e:
            self.log_err(f"悬停过程中发生错误: {str(e)}")
            self.hover_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_hover_delay_t(self, delay_s, use_thread=False):
        """
        线程管理函数，用于启动或停止悬停控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.hover_thread_running:
                    rospy.logwarn("悬停控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.hover_thread and self.hover_thread.is_alive():
                        self.hover_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.hover_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.hover_thread = threading.Thread(
                        target=self._hover_delay_t,
                        args=(delay_s,)
                    )
                    self.hover_thread.daemon = True  # 设为守护线程
                    self.hover_thread.start()
                    rospy.loginfo(f"悬停控制线程已启动: 延迟={delay_s}s")
                except Exception as e:
                    rospy.logerr(f"启动悬停控制线程失败: {str(e)}")
                    self.hover_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.hover_thread_running:
                    self.hover_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止悬停控制线程...")
                    if self.hover_thread and self.hover_thread.is_alive():
                        self.hover_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("悬停控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的悬停控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.hover_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------延时控制---------------------------------------#






    # ------------------------解锁控制---------------------------------------#
    def _unlock(self):
        """
        函数名称: unlock
        函数功能: 解锁无人机，并在解锁时执行陀螺仪热初始化和OFFBOARD模式切换
        使用案例:
        - 解锁无人机并准备进入OFFBOARD模式: action.unlock()
        """
        if self.ActionExecuteType == "cheak":
            try:
                # 检查当前状态
                if self.state.armed:
                    print("\033[91m错误: 无人机当前已处于解锁状态\033[0m")
                    print("建议: 请先执行锁定操作后再尝试解锁")
                    self.control_complete(True)
                    return False

                # 检查电池电量
                if hasattr(self, 'battery_percentage') and self.battery_percentage < 20:
                    print("\033[91m错误: 当前电池电量较低 ({:.1f}%)\033[0m".format(self.battery_percentage))
                    print("建议: 建议在电池电量高于20%时执行解锁操作，以确保飞行安全")

                # 检查GPS信号
                if hasattr(self, 'gps_status') and self.gps_status < 3:
                    print("\033[91m错误: GPS信号质量不佳\033[0m")
                    print("建议: 请确保GPS信号良好(卫星数≥8)后再执行解锁操作")

                # 检查IMU状态
                if hasattr(self, 'imu_status') and not self.imu_status:
                    print("\033[91m错误: IMU状态异常\033[0m")
                    print("建议: 请检查IMU传感器状态，确保陀螺仪和加速度计工作正常")

                # 模拟打印运行信息
                print("\n执行解锁操作流程:")
                print("1. 开始陀螺仪热初始化...")
                print("2. 陀螺仪热初始化完成")
                print("3. 尝试切换至OFFBOARD模式...")
                print("4. 成功进入OFFBOARD模式")
                print("5. 执行解锁指令...")
                print("6. 解锁操作完成\n")

                print("提示: 在实际运行时，系统将:")
                print("- 执行陀螺仪热初始化(约需2秒)")
                print("- 切换至OFFBOARD模式")
                print("- 发送解锁指令(最多尝试3秒)")
                print("- 持续监控无人机状态直至解锁完成")

                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.offboard_mask = True
                self.control_complete(True)
                return True

            except Exception as e:
                print(f"\033[91m错误: 参数检查过程中发生错误: {str(e)}\033[0m")
                self.control_complete(True)
                return False

        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            try:
                # 进行陀螺仪热校准
                command = CommandLongRequest()
                command.broadcast = False
                command.command = 241  # MAV_CMD_PREFLIGHT_CALIBRATION
                command.confirmation = 0
                command.param1 = 1  # Gyro calibration
                command.param2 = 0
                command.param3 = 0
                command.param4 = 0
                command.param5 = 0
                command.param6 = 0
                command.param7 = 0

                # 调用服务
                result = self.command_service(command)
                if result.success:
                    print("陀螺仪热初始化成功")  # 打印成功消息
                else:
                    print("陀螺仪热初始化失败")  # 打印失败消息

                # 等待2秒以确保热初始化过程完成
                rospy.sleep(2)

                # 尝试进入OFFBOARD模式
                target = PositionTarget()  # 创建位置目标消息
                target.type_mask = 0b101111111000  # 设置功能掩码
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.position.x = 0  # 设置X坐标
                target.position.y = 0  # 设置Y坐标
                target.position.z = 0  # 设置Z坐标
                target.yaw = 0  # 设置目标yaw角度
                self.last_command = target  # 存储当前命令

                # 发布目标点以激活OFFBOARD模式
                rate = rospy.Rate(5)  # 频率设置为20Hz
                while not rospy.is_shutdown():
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("unlock 线程被中断")
                        break
                    if self.setpoint_pub:
                        self.setpoint_pub.publish(target)  # 发布最后一次的命令
                    if self.state.mode != "OFFBOARD":
                        if self.set_mode_service:
                            self.set_mode_service(custom_mode="OFFBOARD")  # 设置模式为OFFBOARD
                        rospy.sleep(0.1)  # 等待0.2秒
                    if self.state.mode == "OFFBOARD":
                        print("成功进入OFFBOARD模式")  # 打印成功消息
                        self.offboard_mask = True
                        break
                    rate.sleep()  # 等待以保持频率

                # 解锁无人机
                ros_time_limit = 3  # 解锁操作的ROS时间限制，设置为3秒
                start_time = rospy.Time.now()  # 获取当前时间
                while (rospy.Time.now() - start_time).to_sec() < ros_time_limit:
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("unlock 线程被中断")
                        break
                    self.arming_service(True)  # 发送解锁指令
                    rospy.sleep(0.5)  # 等待0.5秒以减少时间浪费
                print("解锁完成")  # 打印完成消息
                
                self.control_complete(True)  # 设置控制完成标志

            except rospy.ServiceException as e:
                print("服务调用失败: %s" % e)  # 打印服务调用失败的错误消息
                self.control_complete(True)  # 设置控制完成标志

    def unlock(self, use_thread=False):
        """
        函数名称: unlock
        函数功能: 解锁无人机，并在解锁时执行陀螺仪热初始化和OFFBOARD模式切换
        参数:
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 解锁无人机并准备进入OFFBOARD模式: action.unlock()
        - 使用线程方式解锁: action.unlock(use_thread=True)
        """
        try:
            if use_thread:
                self.thread_unlock(use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.unlock_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 unlock 线程")
                    self.thread_unlock(use_thread=False)
                
                # 直接执行解锁函数
                return self._unlock()
        except Exception as e:
            self.log_err(f"解锁过程中发生错误: {str(e)}")
            self.unlock_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_unlock(self, use_thread=False):
        """
        线程管理函数，用于启动或停止解锁控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.unlock_thread_running:
                    rospy.logwarn("解锁控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.unlock_thread and self.unlock_thread.is_alive():
                        self.unlock_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.unlock_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.unlock_thread = threading.Thread(
                        target=self._unlock
                    )
                    self.unlock_thread.daemon = True  # 设为守护线程
                    self.unlock_thread.start()
                    rospy.loginfo("解锁控制线程已启动")
                except Exception as e:
                    rospy.logerr(f"启动解锁控制线程失败: {str(e)}")
                    self.unlock_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.unlock_thread_running:
                    self.unlock_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止解锁控制线程...")
                    if self.unlock_thread and self.unlock_thread.is_alive():
                        self.unlock_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("解锁控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的解锁控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.unlock_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------解锁控制---------------------------------------#


    # ------------------------强制上锁控制---------------------------------------#
    def _lock(self):
        """
        函数名称: lock
        函数功能: 上锁无人机，添加高度检查
        使用案例:
        - 上锁无人机: action.lock()
        - 说明:无人机高度高于0.2米上锁功能无效
        """
        if self.ActionExecuteType == "cheak":
            # 检查参数是否合法
            try:
                # 检查当前状态
                if not self.state.armed:
                    self.control_complete(True)
                    return False

                # 检查高度
                if hasattr(self, 'current_position_z') and self.current_position_z > 0.5:
                    print(f"\033[91m错误: 当前高度 ({self.current_position_z:.2f}米) 超过安全上锁高度\033[0m")
                    print("建议: 请先降低无人机高度至0.5米以下再执行上锁操作")
                    self.control_complete(True)
                    return False

                # 检查飞行模式
                if not self.offboard_mask:
                    print("\033[91m错误: 无人机不在OFFBOARD模式下\033[0m")
                    print("建议: 请先切换至OFFBOARD模式再执行上锁操作")

                # 模拟打印运行信息
                print("\n执行上锁操作流程:")
                print("1. 检查无人机高度...")
                print("2. 确认高度低于0.5米")
                print("3. 发送上锁指令...")
                print("4. 等待上锁状态确认...")
                print("5. 上锁操作完成\n")

                print("提示: 在实际运行时，系统将:")
                print("- 检查无人机当前高度是否安全")
                print("- 发送上锁指令(最多尝试3秒)")
                print("- 持续监控无人机状态直至上锁完成")
                print("- 如高度超过0.5米将拒绝执行上锁")

                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True

            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("建议: 请检查所有输入参数的格式是否正确")
                self.control_complete(True)
                return False

        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    return

            if self.current_position_z > 0.5:
                print("警告: 无人机高度高于0.5米，上锁失败")
                return

            # 在实际模式下，执行上锁操作
            print("正在上锁无人机...")
            
            # 自动强制上锁，有高空坠落风险
            disarm_cmd_long = CommandLongRequest()
            disarm_cmd_long.broadcast = False
            disarm_cmd_long.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
            disarm_cmd_long.param1 = 0  # Disarm
            disarm_cmd_long.param2 = 21196  # Kill no check landed

            rate = rospy.Rate(1.0)
            while not rospy.is_shutdown():
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("lock 线程被中断")
                    break
                result = self.command_service(disarm_cmd_long)
                if result.success:
                    print("无人机已解除武装 (disarmed)")
                    break
                else:
                    print("解除武装失败，重试中...")
                rate.sleep()
            
            self.control_complete(True)  # 设置控制完成标志
            print("紧急解除武装完成，继续发布设定点。")

    def lock(self, use_thread=False):
        """
        函数名称: lock
        函数功能: 锁定无人机（上锁）
        参数:
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 锁定无人机: action.lock()
        - 使用线程方式锁定: action.lock(use_thread=True)
        """
        try:
            if use_thread:
                self.thread_lock(use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.lock_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 lock 线程")
                    self.thread_lock(use_thread=False)
                
                # 直接执行锁定函数
                return self._lock()
        except Exception as e:
            self.log_err(f"锁定过程中发生错误: {str(e)}")
            self.lock_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_lock(self, use_thread=False):
        """
        线程管理函数，用于启动或停止锁定控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.lock_thread_running:
                    rospy.logwarn("锁定控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.lock_thread and self.lock_thread.is_alive():
                        self.lock_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.lock_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.lock_thread = threading.Thread(
                        target=self._lock
                    )
                    self.lock_thread.daemon = True  # 设为守护线程
                    self.lock_thread.start()
                    rospy.loginfo("锁定控制线程已启动")
                except Exception as e:
                    rospy.logerr(f"启动锁定控制线程失败: {str(e)}")
                    self.lock_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.lock_thread_running:
                    self.lock_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止锁定控制线程...")
                    if self.lock_thread and self.lock_thread.is_alive():
                        self.lock_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("锁定控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的锁定控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.lock_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------强制上锁控制---------------------------------------#




    # ------------------------自动降落控制（目前还不是特别稳定）---------------------------------------#
    def _land_auto(self):
        """
        函数名称: land_auto
        函数功能: 自动控制无人机降落，根据当前高度决定何时终止降落并上锁。
        使用案例:
        - 自动降落：action.land_auto()
        """
        if self.ActionExecuteType == "cheak":
            # 检查参数是否合法
            try:
                # 检查当前状态
                if not self.state.armed:
                    print("\033[91m错误: 无人机当前处于上锁状态\033[0m")
                    print("建议: 请先解锁无人机后再执行自动降落")
                    self.control_complete(True)
                    return False

                # 检查当前高度
                if hasattr(self, 'current_position_z') and self.current_position_z < 0.3:
                    print(f"警告: 当前高度 ({self.current_position_z:.2f}米) 过低")
                    print("建议: 建议在高度大于0.3米时执行自动降落")
                    print("说明: 过低的起始高度可能导致降落不稳定")

                # 检查飞行模式
                if not self.offboard_mask:
                    print("警告: 无人机不在OFFBOARD模式下")
                    print("建议: 请先切换至OFFBOARD模式再执行自动降落")

                # 模拟打印运行信息
                print("\n执行自动降落操作流程:")
                print("1. 第一阶段: 降落至0.15米高度")
                print("   - 使用位置控制")
                print("   - 保持当前X、Y位置不变")
                print("   - 维持当前航向角")
                print("   - 位置容差设为0.05米")
                print("2. 第二阶段: 最终降落")
                print("   - 切换为速度控制")
                print("   - Z轴速度设为-0.1米/秒")
                print("   - 持续1.5秒")
                print("3. 最终阶段: 执行上锁\n")

                print("参数建议:")
                print("- 建议在电机状态良好时执行自动降落")
                print("- 建议在周围无障碍物的开阔环境下降落")
                print("- 建议保持环境光照充足以确保高度估计准确")
                print("- 建议在无强风环境下执行降落操作")
                print("- 如遇到不稳定情况，可随时中断降落并切换为手动控制")

                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True

            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                self.control_complete(True)
                return False

        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    return

            rate = rospy.Rate(20)  # 设置频率为20Hz
            hold_yaw = self.get_current_yaw()  # 保存当前yaw角度

            # 第一阶段：降落到0.15米高度
            current_x, current_y, current_z = self.get_current_xyz()
            setpoint_position = PositionTarget()
            setpoint_position.header.stamp = rospy.Time.now()
            setpoint_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint_position.type_mask = 0b010111111000  # 使用位置控制，忽略速度和加速度
            setpoint_position.position.x = current_x  # 保持当前x位置
            setpoint_position.position.y = current_y  # 保持当前y位置
            setpoint_position.position.z = 0.1       # 设置目标z位置为0.1米
            setpoint_position.yaw = hold_yaw          # 保持当前yaw角度
            
            # 持续发送位置指令，直到接近目标位置（容差为0.1米）
            while not rospy.is_shutdown():
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("land_auto 线程被中断")
                    break
                    
                current_x, current_y, current_z = self.get_current_xyz()
                
                # 检查当前高度是否接近目标高度（容差为0.1米）
                if abs(current_z - 0.15) <= 0.1:
                    self.log_info("已达到目标高度0.15米，准备进入最终降落阶段")
                    break

                # 发布位置控制指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(setpoint_position)

                self.log_debug(f"发布位置指令: x={current_x}, y={current_y}, z=0.15, yaw={hold_yaw}")
                rate.sleep()

            # 第二阶段：使用速度控制进行最终降落
            print("开始最终降落阶段，Z轴速度设为-0.28米/秒")
            setpoint_velocity = PositionTarget()
            setpoint_velocity.header.stamp = rospy.Time.now()
            setpoint_velocity.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint_velocity.type_mask = 0b101111000011  # 使用速度控制，忽略位置和加速度，使用yaw而不是yaw_rate
            setpoint_velocity.velocity.x = 0.0   # X轴速度为0
            setpoint_velocity.velocity.y = 0.0   # Y轴速度为0
            setpoint_velocity.velocity.z = -0.28  # Z轴速度为-0.28米/秒（向下）
            setpoint_position.position.z = 0
            setpoint_velocity.yaw = hold_yaw     # 直接使用hold_yaw来保持航向角

            start_time = rospy.Time.now()
            duration = 1.5  # 持续1.5秒
            self.log_info("开始最终降落阶段，Z轴速度设为-0.28米/秒")
            
            while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("land_auto 线程被中断")
                    break
                
                # 发布速度控制指令，直接使用hold_yaw保持航向
                if self.setpoint_pub:
                    self.setpoint_pub.publish(setpoint_velocity)

                self.log_debug(f"发布速度指令: vx=0, vy=0, vz=-0.28, yaw={hold_yaw}")
                rate.sleep()

            # 最后阶段：强制上锁
            self._lock()
            self.log_info("自动降落完成，执行上锁操作")

    def land_auto(self, use_thread=False):
        """
        函数名称: land_auto
        函数功能: 控制无人机自动降落
        参数:
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 自动降落: action.land_auto()
        - 使用线程方式自动降落: action.land_auto(use_thread=True)
        """
        if self.ActionExecuteType == "cheak":
            # 检查参数是否合法
            try:
                if not isinstance(use_thread, bool):
                    print(f"错误: use_thread参数必须是布尔类型, 当前值: {use_thread}")
                    return False
                
                # 模拟打印运行信息
                print(f"检查模式: 执行自动降落操作, 使用线程: {use_thread}")
                print("参数建议: 在实际飞行中，建议使用线程方式执行自动降落以避免阻塞主程序")
                return True
            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                return False
                
        try:
            if use_thread:
                self.thread_land_auto(use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.land_auto_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 land_auto 线程")
                    self.thread_land_auto(use_thread=False)
                
                # 直接执行自动降落函数
                return self._land_auto()
        except Exception as e:
            self.log_err(f"自动降落过程中发生错误: {str(e)}")
            self.land_auto_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志


    def thread_land_auto(self, use_thread=False):
        """
        线程管理函数，用于启动或停止自动降落控制线程
        """
        if self.ActionExecuteType == "cheak":
            # 检查参数是否合法
            try:
                if not isinstance(use_thread, bool):
                    print(f"错误: use_thread参数必须是布尔类型, 当前值: {use_thread}")
                    return False
                
                # 模拟打印运行信息
                if use_thread:
                    print("检查模式: 启动自动降落控制线程")
                else:
                    print("检查模式: 停止自动降落控制线程")
                return True
            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                return False
                
        try:
            if use_thread:
                # 启动线程
                if self.land_auto_thread_running:
                    rospy.logwarn("自动降落控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.land_auto_thread and self.land_auto_thread.is_alive():
                        self.land_auto_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.land_auto_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.land_auto_thread = threading.Thread(
                        target=self._land_auto
                    )
                    self.land_auto_thread.daemon = True  # 设为守护线程
                    self.land_auto_thread.start()
                    rospy.loginfo("自动降落控制线程已启动")
                except Exception as e:
                    rospy.logerr(f"启动自动降落控制线程失败: {str(e)}")
                    self.land_auto_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.land_auto_thread_running:
                    self.land_auto_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止自动降落控制线程...")
                    if self.land_auto_thread and self.land_auto_thread.is_alive():
                        self.land_auto_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("自动降落控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的自动降落控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.land_auto_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------自动降落控制（目前还不是特别稳定）---------------------------------------#




    # ------------------------导航避障控制---------------------------------------#
    def _publish_nav_goal_x_y_z_yaw_t_frame(self, x, y, z, yaw, t, frame):
        """
        函数名称: _publish_nav_goal_x_y_z_yaw_t_frame
        函数功能: 发布导航目标到 'nav_goal' 话题，并在指定时间内以 20Hz 发布 mavros/setpoint_raw/local 消息
        参数:
        - x: 目标位置的X坐标
        - y: 目标位置的Y坐标
        - z: 目标位置的Z坐标
        - yaw: 目标航向角（弧度）
        - 偏航角说明(无人机到达容差范围内，也就是到达目标点后开始旋转):
        - 正值: 顺时针旋转，角度范围为 0 到 360 度。
        - 负值: 逆时针旋转，角度范围为 0 到 -360 度。
        - t: 持续时间（秒）
        - frame: 坐标系（'local'表示map坐标系，'body'表示base_link坐标系）
        """
        if self.ActionExecuteType == "cheak":
            try:
                # 参数类型检查
                if not isinstance(x, (int, float)) or not isinstance(y, (int, float)) or not isinstance(z, (int, float)):
                    print("\033[91m错误: 坐标值(x, y, z)必须是数字类型\033[0m")
                    print("建议: 请输入有效的数字坐标")
                    self.control_complete(True)
                    return False

                if not isinstance(yaw, (int, float)):
                    print("\033[91m错误: 偏航角(yaw)必须是数字类型\033[0m")
                    print("建议: 请输入有效的偏航角度值(弧度)")
                    self.control_complete(True)
                    return False

                if not isinstance(t, (int, float)) or t <= 0:
                    print("\033[91m错误: 持续时间(t)必须是正数\033[0m")
                    print("建议: 请输入大于0的时间值")
                    self.control_complete(True)
                    return False

                if not isinstance(frame, str) or frame.lower() not in ['local', 'body']:
                    print("\033[91m错误: 坐标系参数(frame)必须是'local'(map)或'body'(base_link)\033[0m")
                    print("建议: 请选择正确的坐标系参数")
                    self.control_complete(True)
                    return False

                # 参数范围建议
                print("\n参数检查与建议:")
                
                # 高度建议
                if z < 0:
                    print("警告: 目标高度为负值，这可能导致无人机试图飞到地面以下")
                    print("建议: 请设置一个大于0的目标高度")
                elif z < 0.5:
                    print("警告: 目标高度较低（<0.5m）")
                    print("建议: 建议将目标高度设置在0.5米以上以确保飞行安全")
                elif z > 3:
                    print("警告: 目标高度较高（>3m）")
                    print("建议: 建议将目标高度控制在3米以内，以确保控制精度")

                # 水平距离建议
                distance = (x**2 + y**2)**0.5
                if distance > 5:
                    print(f"警告: 目标点距离较远（{distance:.2f}m）")
                    print("建议: 建议将目标点设置在5米范围内，以确保导航精度")

                # 时间建议
                if t < 3:
                    print("警告: 设定时间较短（<3s）")
                    print("建议: 建议设置3秒以上的执行时间，以确保动作平稳")
                elif t > 30:
                    print("警告: 设定时间较长（>30s）")
                    print("建议: 建议将执行时间控制在30秒内，过长可能影响任务效率")

                # 偏航角建议
                if abs(yaw) > 2 * math.pi:
                    print("警告: 偏航角度超过360度")
                    print("建议: 建议将偏航角限制在-2π到2π之间")

                # 模拟执行流程
                print("\n执行流程模拟:")
                print(f"1. 设置导航目标点: X={x}m, Y={y}m, Z={z}m")
                print(f"2. 选择坐标系: {'map' if frame.lower() == 'local' else 'base_link'}")
                print("3. 开始向目标点导航")
                print("4. 到达目标点后调整偏航角")
                print(f"5. 完成导航任务，预计用时{t}秒\n")

                print("执行过程中的安全提示:")
                print("- 请确保导航路径上无障碍物")
                print("- 建议在光照良好的环境下进行导航")
                print("- 确保电池电量充足")
                print("- 保持环境中有足够的定位特征点")

                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True

            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("建议: 请检查所有输入参数的格式是否正确")
                self.control_complete(True)
                return False

        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    return

            if "nolog" not in self.ActionExecuteType:
                self.log_info(f"Publishing nav goal: x={x}, y={y}, z={z}, yaw={yaw}, frame={frame}, duration={t}s")

            publish_nav_goal_x_y_z_yaw_t_frame(
                x, y, z, yaw, t, frame, self.setpoint_pub, self.nav_goal_pub, self.log_info, self.ActionExecuteType, stop_thread_flag=self.stop_thread_flag
            )
            
            self.control_complete(True)  # 设置控制完成标志

        try:
            if self.ActionExecuteType == "cheak":
                print(f"检查当前页面语法模式: 发布导航目标到 (x: {x}, y: {y}, z: {z}, yaw: {yaw}, frame: {frame})，持续时间 {t} 秒")
        except Exception as e:
            self.log_err(f"导航过程中发生错误: {str(e)}")
            raise
        finally:
            self.control_complete(True)
            if self.nav_goal_thread_running:
                self.nav_goal_thread_running = False

    def publish_nav_goal_x_y_z_yaw_t_frame(self, x, y, z, yaw, t, frame, use_thread=False):
        """
        函数名称: _publish_nav_goal_x_y_z_yaw_t_frame
        函数功能: 发布导航目标到 'nav_goal' 话题，并在指定时间内以 20Hz 发布 mavros/setpoint_raw/local 消息
        参数:
        - x: 目标位置的X坐标
        - y: 目标位置的Y坐标
        - z: 目标位置的Z坐标
        - yaw: 目标航向角（弧度）
        - 偏航角说明(无人机到达容差范围内，也就是到达目标点后开始旋转):
        - 正值: 顺时针旋转，角度范围为 0 到 360 度。
        - 负值: 逆时针旋转，角度范围为 0 到 -360 度。
        - t: 持续时间（秒）
        - frame: 坐标系（'local'表示map坐标系，'body'表示base_link坐标系）
        """
        try:
            if use_thread:
                self.thread_publish_nav_goal_x_y_z_yaw_t_frame(x, y, z, yaw, t, frame, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.nav_goal_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断导航目标点线程")
                    self.thread_publish_nav_goal_x_y_z_yaw_t_frame(x, y, z, yaw, t, frame, use_thread=False)
            
            # 直接执行导航目标点发布函数
            return self._publish_nav_goal_x_y_z_yaw_t_frame(x, y, z, yaw, t, frame)
        except Exception as e:
            self.log_err(f"发布导航目标点过程中发生错误: {str(e)}")
            self.nav_goal_thread_running = False
            self.control_complete(True)
            return False
        
    def thread_publish_nav_goal_x_y_z_yaw_t_frame(self, x, y, z, yaw, t, frame, use_thread=False):
        """发布导航目标点线程函数
        
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            yaw: 目标偏航角
            t: 时间
            frame: 坐标系
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                # 启动线程
                if self.nav_goal_thread_running:
                    rospy.logwarn("导航目标点线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.nav_goal_thread and self.nav_goal_thread.is_alive():
                        self.nav_goal_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.nav_goal_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.nav_goal_thread = threading.Thread(
                        target=self._publish_nav_goal_x_y_z_yaw_t_frame,
                        args=(x, y, z, yaw, t, frame)
                    )
                    self.nav_goal_thread.daemon = True  # 设为守护线程
                    self.nav_goal_thread.start()
                    rospy.loginfo(f"导航目标点线程已启动: x={x}, y={y}, z={z}, yaw={yaw}")
                except Exception as e:
                    rospy.logerr(f"启动导航目标点线程失败: {str(e)}")
                    self.nav_goal_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.nav_goal_thread_running:
                    self.nav_goal_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止导航目标点线程...")
                    if self.nav_goal_thread and self.nav_goal_thread.is_alive():
                        self.nav_goal_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("导航目标点线程已停止")
                else:
                    rospy.logwarn("没有运行中的导航目标点线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.nav_goal_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------导航避障控制---------------------------------------#




    # ------------------------偏航控制---------------------------------------#
    def _control_yaw_z_t(self, yaw, z, t):
        """
        函数名称: control_yaw_z_t
        函数功能: 控制无人机在指定高度并在指定时间内进行偏航控制，保持当前位置的X和Y不变。
        
        参数:
        - z: 目标高度
        - yaw: 相对偏航角，范围为 0 到 360 度 或 0 到 -360 度。
        - t: 持续时间（秒）

        偏航角说明:
        - 正值: 顺时针旋转，角度范围为 0 到 360 度。
        - 负值: 逆时针旋转，角度范围为 0 到 -360 度。
        """
        if self.ActionExecuteType == "cheak":
            # 检查参数是否合法
            try:
                # 检查偏航角参数
                if not isinstance(yaw, (int, float)):
                    print(f"\033[91m错误: 偏航角必须是数值类型\033[0m")
                    print("建议: 请输入有效的数字角度值")
                    self.control_complete(True)
                    return False

                if abs(yaw) > 360:
                    print(f"警告: 偏航角 {yaw} 度超出了推荐范围 [-360, 360]")
                    print("建议: 建议使用[-360, 360]范围内的值，过大的旋转角度可能导致不稳定")
                elif abs(yaw) < 1:
                    print(f"警告: 偏航角 {yaw} 度过小")
                    print("建议: 过小的旋转角度可能看不出明显效果，建议使用大于1度的角度")

                # 检查高度参数
                if not isinstance(z, (int, float)):
                    print(f"\033[91m错误: 高度必须是数值类型\033[0m")
                    print("建议: 请输入有效的数字高度值")
                    self.control_complete(True)
                    return False

                if z < 0:
                    print(f"\033[91m错误: 高度值 {z} 不能为负数\033[0m")
                    print("建议: 请设置一个大于0的目标高度")
                    self.control_complete(True)
                    return False
                elif z < 0.5:
                    print(f"警告: 高度值 {z} 米过低")
                    print("建议: 建议设置在0.5米以上以确保安全裕度")
                    print("说明: 过低的飞行高度可能导致下洗气流影响和意外碰撞")
                elif z > 5:
                    print(f"警告: 高度值 {z} 米较高")
                    print("建议: 建议在5米以下飞行以保证控制精度")
                    print("说明: 过高飞行可能受风力影响较大")

                # 检查时间参数
                if not isinstance(t, (int, float)):
                    print(f"\033[91m错误: 持续时间必须是数值类型\033[0m")
                    print("建议: 请输入有效的数字时间值")
                    self.control_complete(True)
                    return False

                if t <= 0:
                    print(f"\033[91m错误: 持续时间 {t} 秒必须为正数\033[0m")
                    print("建议: 请设置一个大于0的持续时间")
                    self.control_complete(True)
                    return False
                elif t < 1:
                    print(f"警告: 持续时间 {t} 秒过短")
                    print("建议: 建议设置1秒以上的执行时间")
                    print("说明: 过短的执行时间可能导致动作不完整")
                elif t > 30:
                    print(f"警告: 持续时间 {t} 秒过长")
                    print("建议: 建议将动作分解为多个较短的控制指令")
                    print("说明: 过长的单次控制可能影响整体任务节奏")

                # 模拟打印运行信息
                print("\n执行偏航控制操作流程:")
                print(f"1. 保持当前XY位置不变")
                print(f"2. 调整飞行高度至 {z} 米")
                print(f"3. 执行偏航旋转 {yaw} 度")
                print(f"4. 持续控制 {t} 秒")
                print(f"5. 完成偏航控制\n")

                print("控制过程中的注意事项:")
                print("- 确保执行空间内无障碍物")
                print("- 注意观察飞机姿态是否稳定")
                print("- 保持充足的安全裕度")
                print("- 如发现异常及时中断操作")

                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True

            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("建议: 请检查所有输入参数的格式是否正确")
                self.control_complete(True)
                return False

        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    self.control_complete(True)
                    return

            rate = rospy.Rate(20)  # 设置频率为20Hz
            # 获取当前X、Y坐标和当前yaw角度，只读取一次
            current_x, current_y, current_z = self.get_current_xyz()
            hold_yaw = self.get_current_yaw()  # 获取当前yaw角度

            # 将目标yaw角度转换为弧度并反转正负符号
            relative_yaw = -yaw * (math.pi / 180.0)
            # 获取当前时间
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < t:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("control_yaw_z_t 线程被中断")
                    break
                # 设置目标的yaw和保持当前位置的x, y
                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  # 使用本地NED坐标系
                target.type_mask = 0b101111111000  # 使用位置控制并控制yaw
                target.position.x = current_x  # 保持当前位置的X坐标
                target.position.y = current_y  # 保持当前位置的Y坐标
                target.position.z = z  # 目标高度
                target.yaw = hold_yaw + relative_yaw  # 使用相对yaw值
                # 发布目标点指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                    self.log_info(f"发布偏航控制: yaw={yaw} 度, 高度={z}, 当前x={current_x}, y={current_y}")
                rate.sleep()  # 保持20Hz的循环频率
            self.log_info("偏航控制完成")
            
            self.control_complete(True)  # 设置控制完成标志

    def control_yaw_z_t(self, yaw, z, t, use_thread=False):
        """
        函数名称: control_yaw_z_t
        函数功能: 控制无人机的偏航角和高度
        参数:
        - yaw: 目标偏航角（度）
        - z: 目标高度（米）
        - t: 持续时间（秒）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 控制偏航角和高度: action.control_yaw_z_t(90, 1.5, 10)  # 旋转90度并保持1.5米高度10秒
        - 使用线程方式: action.control_yaw_z_t(90, 1.5, 10, use_thread=True)
        """
        try:
            if use_thread:
                # 启动线程
                if hasattr(self, 'control_yaw_thread_running') and self.control_yaw_thread_running:
                    rospy.logwarn("偏航角控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if hasattr(self, 'control_yaw_thread') and self.control_yaw_thread and self.control_yaw_thread.is_alive():
                        self.control_yaw_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.control_yaw_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.control_yaw_thread = threading.Thread(
                        target=self._control_yaw_z_t,
                        args=(yaw, z, t)
                    )
                    self.control_yaw_thread.daemon = True  # 设为守护线程
                    self.control_yaw_thread.start()
                    rospy.loginfo(f"偏航角控制线程已启动: yaw={yaw}, z={z}, t={t}")
                    return True
                except Exception as e:
                    rospy.logerr(f"启动偏航角控制线程失败: {str(e)}")
                    self.control_yaw_thread_running = False
                    self.control_complete(True)
                    return False
            else:
                # 如果线程正在运行，先停止它
                if hasattr(self, 'control_yaw_thread_running') and self.control_yaw_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 control_yaw_z_t 线程")
                    if hasattr(self, 'control_yaw_thread') and self.control_yaw_thread and self.control_yaw_thread.is_alive():
                        self.control_yaw_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    self.control_yaw_thread_running = False
                
                # 直接执行偏航角和高度控制函数
                return self._control_yaw_z_t(yaw, z, t)
        except Exception as e:
            self.log_err(f"偏航角和高度控制过程中发生错误: {str(e)}")
            if hasattr(self, 'control_yaw_thread_running'):
                self.control_yaw_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志
    # ------------------------偏航控制---------------------------------------#



    # ------------------------速度控制---------------------------------------#
    def _send_velocity_vx_vy_z_t_frame(self, vx, vy, z, duration, frame="local"):
        """
        函数名称: send_velocity_vx_vy_z_t_frame
        函数功能: 在指定坐标系下以指定的速度控制无人机移动一段时间
        参数:
        - vx: X方向速度
        - vy: Y方向速度
        - z: 目标高度
        - duration: 持续时间（秒）
        - frame: 坐标系类型，"local"表示本地坐标系，"body"表示机体坐标系
        使用案例:
        - 控制无人机以速度(vx, vy)在高度z飞行duration秒: action.send_velocity_vx_vy_z_t_frame(1, 1, 5, 10, "local")
        """
        if self.ActionExecuteType == "cheak":
            try:
                # 参数类型检查
                if not isinstance(vx, (int, float)) or not isinstance(vy, (int, float)):
                    print("\033[91m错误: 速度值(vx, vy)必须是数字类型\033[0m")
                    print("建议: 请输入有效的数字速度值")
                    self.control_complete(True)
                    return False

                if not isinstance(z, (int, float)):
                    print("\033[91m错误: 高度值必须是数字类型\033[0m")
                    print("建议: 请输入有效的数字高度值")
                    self.control_complete(True)
                    return False

                if not isinstance(duration, (int, float)) or duration <= 0:
                    print("\033[91m错误: 持续时间必须是正数\033[0m")
                    print("建议: 请输入大于0的时间值")
                    self.control_complete(True)
                    return False

                if not isinstance(frame, str) or frame.lower() not in ["local", "body"]:
                    print("\033[91m错误: 坐标系参数必须是'local'或'body'\033[0m")
                    print("建议: 请选择正确的坐标系参数")
                    self.control_complete(True)
                    return False

                # 参数范围建议
                print("\n参数检查与建议:")

                # 速度建议
                if abs(vx) > 2 or abs(vy) > 2:
                    print(f"警告: 速度值({vx}, {vy})较大")
                    print("建议: 建议将速度控制在±2m/s以内，过大的速度可能导致控制不稳定")
                elif abs(vx) < 0.1 and abs(vy) < 0.1 and (vx != 0 or vy != 0):
                    print(f"警告: 速度值({vx}, {vy})较小")
                    print("建议: 速度过小可能导致运动不明显，建议使用大于0.1m/s的速度")

                # 高度建议
                if z < 0:
                    print(f"\033[91m错误: 高度值 {z} 不能为负数\033[0m")
                    print("建议: 请设置一个大于0的目标高度")
                    self.control_complete(True)
                    return False
                elif z < 0.5:
                    print(f"警告: 高度值 {z} 米过低")
                    print("建议: 建议设置在0.5米以上以确保安全裕度")
                    print("说明: 过低的飞行高度可能导致下洗气流影响和意外碰撞")
                elif z > 5:
                    print(f"警告: 高度值 {z} 米较高")
                    print("建议: 建议将高度控制在5米以内，过高可能影响控制精度")

                # 持续时间建议
                if duration > 30:
                    print(f"警告: 持续时间 {duration} 秒较长")
                    print("建议: 建议将单次控制时间控制在30秒以内，过长的控制时间可能导致累积误差")
                elif duration < 2:
                    print(f"警告: 持续时间 {duration} 秒较短")
                    print("建议: 过短的控制时间可能无法达到预期效果，建议设置2秒以上")

                # 坐标系说明
                print("\n坐标系说明:")
                if frame.lower() == "body":
                    print("- 当前使用机体坐标系(body)")
                    print("- vx正值表示机头方向，vy正值表示机体右侧方向")
                else:
                    print("- 当前使用本地坐标系(local)")
                    print("- vx正值表示正北方向，vy正值表示正东方向")

                # 模拟执行步骤
                print("\n模拟执行步骤:")
                print("1. 检查无人机是否处于OFFBOARD模式")
                print(f"2. 获取当前位置和偏航角")
                print(f"3. 设置目标速度: vx={vx}m/s, vy={vy}m/s")
                print(f"4. 保持目标高度z={z}米")
                print(f"5. 保持当前偏航角不变")
                print(f"6. 以20Hz频率发送控制指令，持续{duration}秒")
                print("7. 完成速度控制任务")

                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True

            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("建议: 请检查所有输入参数的格式是否正确")
                self.control_complete(True)
                return False

        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    return

            rate = rospy.Rate(20)  # 设置频率为20Hz
            start_time = rospy.Time.now()
            
            # 保存当前位置和yaw角度，仅在初次进入循环时保存一次
            current_x, current_y, current_z = self.get_current_xyz()  # 获取当前X和Y坐标
            hold_yaw = self.get_current_yaw()  # 保存初始yaw角度

            # 开始控制逻辑
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("send_velocity_vx_vy_z_t_frame 线程被中断")
                    break
                    
                # 获取当前高度，用于计算垂直速度
                _, _, current_z = self.get_current_xyz()
                # 获取当前yaw角度，用于body坐标系下的yaw控制
                current_yaw = self.get_current_yaw()
                
                target = PositionTarget()
                
                # 根据frame参数设置不同的坐标系
                if frame.lower() == "body":
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                    self.log_info(f"使用机体坐标系(FRAME_BODY_NED)速度控制")
                    
                    # 在body坐标系下使用特殊的高度控制方法
                    # 计算高度误差和垂直速度
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 2.0, -0.2, 0.2)  # 使用与nav_position_pkg.py相同的控制逻辑
                    
                    # 计算yaw误差和yaw角速度
                    yaw_error = hold_yaw - current_yaw
                    # 标准化yaw误差到[-pi, pi]
                    if yaw_error > np.pi:
                        yaw_error -= 2 * np.pi
                    elif yaw_error < -np.pi:
                        yaw_error += 2 * np.pi
                    yaw_rate = -np.clip(yaw_error * 1.0, -0.5, 0.5)  # 使用比例控制，限制角速度
                    
                    # 设置目标速度和位置控制
                    if vx == 0 and vy != 0:
                        target.type_mask = 0b100111000111  # 使用速度控制和yaw_rate
                        target.velocity.x = 0
                        target.velocity.y = vy
                        target.velocity.z = vertical_velocity
                        target.yaw_rate = yaw_rate
                    elif vy == 0 and vx != 0:
                        target.type_mask = 0b100111000111  # 使用速度控制和yaw_rate
                        target.velocity.y = 0
                        target.velocity.x = vx
                        target.velocity.z = vertical_velocity
                        target.yaw_rate = yaw_rate
                    elif vx == 0 and vy == 0:
                        target.type_mask = 0b100111000111  # 使用速度控制和yaw_rate
                        target.velocity.x = 0
                        target.velocity.y = 0
                        target.velocity.z = vertical_velocity
                        target.yaw_rate = yaw_rate
                    else:
                        target.type_mask = 0b100111000111  # 使用速度控制和yaw_rate
                        target.velocity.x = vx
                        target.velocity.y = vy
                        target.velocity.z = vertical_velocity
                        target.yaw_rate = yaw_rate
                    
                    self.log_info(f"高度控制: 当前高度={current_z:.2f}, 目标高度={z:.2f}, 误差={altitude_error:.2f}, 垂直速度={vertical_velocity:.2f}")
                    self.log_info(f"偏航控制: 当前偏航={current_yaw:.2f}, 目标偏航={hold_yaw:.2f}, 误差={yaw_error:.2f}, 偏航速率={yaw_rate:.2f}")
                else:  # 默认使用local坐标系
                    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    self.log_info(f"使用本地坐标系(FRAME_LOCAL_NED)速度控制")
                    
                    # 设置目标速度和位置控制
                    if vx == 0 and vy != 0:
                        target.type_mask = 0b101111000000  # 保持X位置
                        target.position.x = current_x
                        target.velocity.x = 0
                        target.velocity.y = vy
                        target.velocity.z = 0
                    elif vy == 0 and vx != 0:
                        target.type_mask = 0b101111000000  # 保持Y位置
                        target.position.y = current_y
                        target.velocity.y = 0
                        target.velocity.x = vx
                        target.velocity.z = 0
                    elif vx == 0 and vy == 0:
                        target.type_mask = 0b101111000000  # 保持X和Y位置
                        target.position.x = current_x
                        target.velocity.x = 0
                        target.position.y = current_y
                        target.velocity.y = 0
                        target.velocity.z = 0
                    else:
                        target.type_mask = 0b101111000011  # 使用速度控制
                        target.velocity.x = vx
                        target.velocity.y = vy
                        target.velocity.z = 0
                    
                    # 设置高度
                    target.position.z = z  # 保持目标高度
                    
                    # 设置yaw角度
                    target.yaw = hold_yaw  # 设置初始保存的yaw角度

                # 发布目标指令
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)

                # ROS 日志打印输出当前状态
                if frame.lower() == "body":
                    self.log_info(f"Real模式 - 发布速度控制: 速度(vx={vx}, vy={vy}, vz={vertical_velocity:.2f}), 目标高度={z}, 当前高度={current_z:.2f}, 目标Yaw={hold_yaw}, 当前Yaw={current_yaw:.2f}, Yaw速率={yaw_rate:.2f}, 坐标系: {frame}")
                else:
                    self.log_info(f"Real模式 - 发布速度控制: 速度(vx={vx}, vy={vy}), 位置保持(X={current_x if vx == 0 else 'N/A'}, Y={current_y if vy == 0 else 'N/A'}), 高度={z}, Yaw={hold_yaw}, 坐标系: {frame}")

                rate.sleep()  # 保持20Hz的循环频率
            
            self.control_complete(True)  # 设置控制完成标志
            
    def send_velocity_vx_vy_z_t_frame(self, vx, vy, z, duration, frame="local", use_thread=False):
        """
        函数名称: send_velocity_vx_vy_z_t_frame
        函数功能: 控制无人机以指定速度飞行指定时间，支持不同坐标系
        参数:
        - vx: x方向速度（米/秒）
        - vy: y方向速度（米/秒）
        - z: 目标高度（米）
        - duration: 持续时间（秒）
        - frame: 坐标系类型，"local"表示本地坐标系，"body"表示机体坐标系
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 在本地坐标系以0.5m/s的速度向前飞行10秒: action.send_velocity_vx_vy_z_t_frame(0.5, 0, 1.5, 10, "local")
        - 在机体坐标系以0.5m/s的速度向前飞行10秒: action.send_velocity_vx_vy_z_t_frame(0.5, 0, 1.5, 10, "body")
        - 使用线程方式: action.send_velocity_vx_vy_z_t_frame(0.5, 0, 1.5, 10, "local", use_thread=True)
        """
        try:
            if use_thread:
                self.thread_send_velocity_vx_vy_z_t_frame(vx, vy, z, duration, frame, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.send_velocity_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 send_velocity_vx_vy_z_t_frame 线程")
                    self.thread_send_velocity_vx_vy_z_t_frame(vx, vy, z, duration, frame, use_thread=False)
                
                # 直接执行速度控制函数
                return self._send_velocity_vx_vy_z_t_frame(vx, vy, z, duration, frame)
        except Exception as e:
            self.log_err(f"速度控制过程中发生错误: {str(e)}")
            self.send_velocity_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_send_velocity_vx_vy_z_t_frame(self, vx, vy, z, duration, frame="local", use_thread=False):
        """
        线程管理函数，用于启动或停止速度控制线程，支持不同坐标系
        """
        try:
            if use_thread:
                # 启动线程
                if self.velocity_thread_running:
                    rospy.logwarn("速度控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.velocity_thread and self.velocity_thread.is_alive():
                        self.velocity_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.velocity_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.velocity_thread = threading.Thread(
                        target=self._send_velocity_vx_vy_z_t_frame,
                        args=(vx, vy, z, duration, frame)
                    )
                    self.velocity_thread.daemon = True  # 设为守护线程
                    self.velocity_thread.start()
                    rospy.loginfo(f"速度控制线程已启动: vx={vx}, vy={vy}, z={z}, 持续={duration}s, 坐标系={frame}")
                except Exception as e:
                    rospy.logerr(f"启动速度控制线程失败: {str(e)}")
                    self.velocity_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.velocity_thread_running:
                    self.velocity_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止速度控制线程...")
                    if self.velocity_thread and self.velocity_thread.is_alive():
                        self.velocity_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("速度控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的速度控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.velocity_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------速度控制---------------------------------------#




    # ------------------------降落（使用速度控制）---------------------------------------#
    def _land_lock_vz_t(self, vz, duration):
        """
        函数名称: land_lock_vz_t
        函数功能: 控制无人机降落
        参数:
        - vz: 下降速度（米/秒）
        - duration: 降落时间（秒）
        使用案例:
        - 以-0.4 m/s的速度降落10秒: action.land_lock_vz_t(-0.4, 10)
        """
        if self.ActionExecuteType == "cheak":
            try:
                # 参数类型检查
                if not isinstance(vz, (int, float)):
                    print("\033[91m错误: 下降速度必须是数字类型\033[0m")
                    print("建议: 请输入有效的数字速度值")
                    self.control_complete(True)
                    return False

                if not isinstance(duration, (int, float)) or duration <= 0:
                    print("\033[91m错误: 持续时间必须是正数\033[0m")
                    print("建议: 请输入大于0的时间值")
                    self.control_complete(True)
                    return False

                # 参数范围建议
                print("\n参数检查与建议:")

                # 速度建议
                if vz >= 0:
                    print("\033[91m错误: 下降速度必须为负值\033[0m")
                    print("建议: 请输入负的速度值，表示向下运动")
                    self.control_complete(True)
                    return False
                elif vz > -0.1:
                    print(f"警告: 下降速度({vz}m/s)过慢")
                    print("建议: 建议设置在-0.1到-0.5m/s之间，过慢的速度会导致降落时间过长")
                elif vz < -1.0:
                    print(f"警告: 下降速度({vz}m/s)过快")
                    print("建议: 建议不要超过-1.0m/s，过快的速度可能导致不稳定或硬着陆")

                # 时间建议
                if duration < 3:
                    print(f"警告: 降落时间({duration}秒)过短")
                    print("建议: 建议至少设置3秒以上，以确保安全降落")
                elif duration > 30:
                    print(f"警告: 降落时间({duration}秒)过长")
                    print("建议: 建议不要超过30秒，过长的降落时间可能增加风险")

                # 打印执行流程
                print("\n执行降落操作流程:")
                print("1. 检查飞行模式是否为OFFBOARD")
                print("2. 以20Hz频率发送速度控制指令")
                print("3. 保持X、Y方向速度为0")
                print(f"4. Z方向以{vz}m/s的速度下降")
                print("5. 降落3秒后尝试自动上锁")
                print("6. 持续发送控制指令直至完成\n")

                print("安全建议:")
                print("- 建议在电机状态良好时执行降落")
                print("- 建议在周围无障碍物的开阔环境下降落")
                print("- 建议在无强风环境下执行降落操作")
                print("- 如遇到不稳定情况，可随时中断降落并切换为手动控制")

                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True

            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("建议: 请检查所有输入参数的格式是否正确")
                self.control_complete(True)
                return False

        try:
            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    # 等待进入OFFBOARD模式
                    rate = rospy.Rate(1.0)  # 设置频率为1Hz
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(20)  # 设置频率为20Hz
                state_start_time = rospy.Time.now()
                while (rospy.Time.now() - state_start_time).to_sec() < duration:
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("land_lock_vz_t 线程被中断")
                        break
                    setpoint_raw = PositionTarget()
                    setpoint_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    setpoint_raw.type_mask = 0b101111000111  # 设置功能掩码
                    setpoint_raw.velocity.x = 0
                    setpoint_raw.velocity.y = 0
                    setpoint_raw.velocity.z = vz
                    self.setpoint_pub.publish(setpoint_raw)
                    if (rospy.Time.now() - state_start_time).to_sec() > 3:
                        setpoint_raw.type_mask = 0b101111000111  # 设置功能掩码
                        self.log_info("Landing complete")
                        # 自动强制上锁，有高空坠落风险
                        disarm_cmd_long = CommandLongRequest()
                        disarm_cmd_long.broadcast = False
                        disarm_cmd_long.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
                        disarm_cmd_long.param1 = 0  # Disarm
                        disarm_cmd_long.param2 = 21196  # Kill no check landed
                        rate1 = rospy.Rate(1.0)
                        while not rospy.is_shutdown():
                            if self.stop_thread_flag.is_set():
                                rospy.logwarn("land_lock_vz_t 线程被中断")
                                break
                            result = self.command_service(disarm_cmd_long)
                            if result.success:
                                print("无人机已解除武装 (disarmed)")
                                break
                            else:
                                print("解除武装失败，重试中...")
                            rate1.sleep()
                        print("紧急解除武装完成，继续发布设定点。")
                        break
                    rospy.sleep(0.05)
                    rate.sleep()
        except Exception as e:
            self.log_err(f"降落过程中发生错误: {str(e)}")
            raise  # 向上传递异常以在调用函数中处理
        finally:
            self.control_complete(True)  # 确保在任何情况下都设置控制完成标志
            if hasattr(self, 'land_lock_thread_running') and self.land_lock_thread_running:
                self.land_lock_thread_running = False  # 重置线程状态

    def land_lock_vz_t(self, vz, duration, use_thread=False):
        """
        函数名称: land_lock_vz_t
        函数功能: 以指定速度降落指定时间
        参数:
        - vz: 降落速度，单位米/秒（负值表示下降）
        - duration: 降落持续时间（秒）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 以-0.4m/s的速度降落10秒: action.land_lock_vz_t(-0.4, 10)
        - 使用线程方式降落: action.land_lock_vz_t(-0.4, 10, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_land_lock_vz_t(vz, duration, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.land_lock_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 land_lock_vz_t 线程")
                    self.thread_land_lock_vz_t(vz, duration, use_thread=False)
                
                # 直接执行降落函数
                return self._land_lock_vz_t(vz, duration)
        except Exception as e:
            self.log_err(f"降落过程中发生错误: {str(e)}")
            self.land_lock_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_land_lock_vz_t(self, vz, duration, use_thread=False):
        """
        线程管理函数，用于启动或停止降落控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.land_lock_thread_running:
                    rospy.logwarn("降落线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if hasattr(self, 'land_lock_thread') and self.land_lock_thread and self.land_lock_thread.is_alive():
                        self.land_lock_thread.join(timeout=2.0)  # 等待线程结束，最多2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.land_lock_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                self.land_lock_thread = threading.Thread(
                    target=self._land_lock_vz_t,
                    args=(vz, duration)
                )
                self.land_lock_thread.daemon = True  # 设为守护线程
                self.land_lock_thread.start()
                self.log_info(f"降落控制线程已启动: vz={vz}, 持续={duration}s")
            else:
                # 停止线程
                if self.land_lock_thread_running:
                    self.land_lock_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    self.log_info("正在停止降落控制线程...")
                    if hasattr(self, 'land_lock_thread') and self.land_lock_thread and self.land_lock_thread.is_alive():
                        self.land_lock_thread.join(timeout=2.0)  # 等待线程结束，最多2秒
                    self.log_info("降落控制线程已停止")
                else:
                    self.log_warn("没有运行中的降落控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"降落线程管理过程中发生错误: {str(e)}")
            self.land_lock_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------降落（使用速度控制）---------------------------------------#





    # ------------------------使用多个目标位置采样点控制---------------------------------------#
    def _control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(
        self, x_target, y_target, z, stepsize=0.35, frame="local", tolerance=0.03, axisTolerance=0.1
    ):
        """
        函数功能: 控制无人机沿直线轨迹从当前位置飞往目标位置，生成每隔指定步长的采样点逐步移动到目标位置。
                当某个轴（X、Y或Z）的剩余距离小于axisTolerance时，直接将该轴的采样点设置为目标位置。
                支持不同坐标系下的控制。

        参数:
        - x_target: 目标位置的X坐标
        - y_target: 目标位置的Y坐标
        - z: 目标高度
        - stepsize: 每个采样点之间的步长（默认0.35米），通过调整步长控制飞行激进度。
        - frame: 坐标系类型，"local"表示本地坐标系，"local_offset"表示本地偏移坐标系，"body"表示机体坐标系
        - tolerance: 目标位置的容差（默认0.03米）
        - axisTolerance: 轴的容差（默认0.1米），适用于X、Y和Z轴
        """
        # 检查当前执行模式是否为cheak，如果是则检查参数合法性
        if self.ActionExecuteType == "cheak":
            try:
                # 检查参数合法性
                param_errors = []
                param_warnings = []
                
                # 检查坐标值是否为数值类型
                if not isinstance(x_target, (int, float)):
                    param_errors.append(f"\033[91m错误: X坐标必须为数值类型，当前类型为 {type(x_target).__name__}\033[0m")
                if not isinstance(y_target, (int, float)):
                    param_errors.append(f"\033[91m错误: Y坐标必须为数值类型，当前类型为 {type(y_target).__name__}\033[0m")
                if not isinstance(z, (int, float)):
                    param_errors.append(f"\033[91m错误: Z坐标必须为数值类型，当前类型为 {type(z).__name__}\033[0m")
                
                # 检查坐标值是否在合理范围内，根据不同坐标系进行不同检查
                frame_lower = frame.lower() if isinstance(frame, str) else ""
                
                # 检查X和Y坐标
                if isinstance(x_target, (int, float)):
                    if frame_lower == "local":
                        if abs(x_target) > 10.0:
                            param_warnings.append(f"警告: 本地坐标系下X坐标值 {x_target} 较大，建议在±10.0米范围内以确保安全飞行")
                    elif frame_lower in ["local_offset", "body"]:
                        if abs(x_target) > 5.0:
                            param_warnings.append(f"警告: {frame}坐标系下X偏移值 {x_target} 较大，建议在±5.0米范围内以确保安全飞行")
                
                if isinstance(y_target, (int, float)):
                    if frame_lower == "local":
                        if abs(y_target) > 10.0:
                            param_warnings.append(f"警告: 本地坐标系下Y坐标值 {y_target} 较大，建议在±10.0米范围内以确保安全飞行")
                    elif frame_lower in ["local_offset", "body"]:
                        if abs(y_target) > 5.0:
                            param_warnings.append(f"警告: {frame}坐标系下Y偏移值 {y_target} 较大，建议在±5.0米范围内以确保安全飞行")
                
                # 检查高度是否在合理范围内，根据不同坐标系进行不同检查
                if isinstance(z, (int, float)):
                    if frame_lower == "local":
                        # 本地坐标系下，Z是绝对高度
                        if z < 0.1:
                            param_errors.append(f"\033[91m错误: 本地坐标系下高度值 {z} 米过低，可能导致无人机着陆或碰撞，建议至少设置为0.5米\033[0m")
                        elif z < 0.5:
                            param_warnings.append(f"警告: 本地坐标系下高度值 {z} 米较低，建议在0.5-3.0米范围内以确保安全飞行")
                        elif z > 3.0:
                            param_warnings.append(f"警告: 本地坐标系下高度值 {z} 米较高，建议在0.5-3.0米范围内以确保安全飞行")
                    elif frame_lower in ["local_offset", "body"]:
                        # 偏移坐标系下，Z是相对当前高度的偏移
                        if abs(z) > 1.5:
                            param_warnings.append(f"警告: {frame}坐标系下高度偏移值 {z} 米较大，建议在±1.5米范围内以确保安全飞行")
                        # 在偏移坐标系中，负值是允许的（表示下降），所以不需要检查z<0的情况
                
                # 检查步长是否合理
                if isinstance(stepsize, (int, float)):
                    if stepsize <= 0:
                        param_errors.append(f"\033[91m错误: 步长必须为正数，当前值为 {stepsize}\033[0m")
                    elif stepsize < 0.1:
                        param_warnings.append(f"警告: 步长 {stepsize} 米过小，可能导致飞行缓慢，建议在0.1-1.0米范围内")
                    elif stepsize > 1.0:
                        param_warnings.append(f"警告: 步长 {stepsize} 米过大，可能导致飞行不稳定，建议不超过1.0米")
                else:
                    param_errors.append(f"\033[91m错误: 步长必须为数值类型，当前类型为 {type(stepsize).__name__}\033[0m")
                
                # 检查坐标系参数
                if not isinstance(frame, str):
                    param_errors.append(f"\033[91m错误: 坐标系参数必须为字符串类型，当前类型为 {type(frame).__name__}\033[0m")
                elif frame_lower not in ["local", "local_offset", "body"]:
                    param_errors.append(f"\033[91m错误: 坐标系参数必须为'local'、'local_offset'或'body'，当前值为 {frame}\033[0m")
                
                # 检查容差参数
                if not isinstance(tolerance, (int, float)):
                    param_errors.append(f"\033[91m错误: 容差必须为数值类型，当前类型为 {type(tolerance).__name__}\033[0m")
                elif tolerance <= 0:
                    param_errors.append(f"\033[91m错误: 容差必须为正数，当前值为 {tolerance}\033[0m")
                elif tolerance > 0.1:
                    param_warnings.append(f"警告: 容差值 {tolerance} 米较大，可能导致定位不够精确，建议在0.01-0.1米范围内")
                
                # 检查轴容差参数
                if not isinstance(axisTolerance, (int, float)):
                    param_errors.append(f"\033[91m错误: 轴容差必须为数值类型，当前类型为 {type(axisTolerance).__name__}\033[0m")
                elif axisTolerance <= 0:
                    param_errors.append(f"\033[91m错误: 轴容差必须为正数，当前值为 {axisTolerance}\033[0m")
                elif axisTolerance > 0.2:
                    param_warnings.append(f"警告: 轴容差值 {axisTolerance} 米较大，可能影响定位精度，建议在0.05-0.2米范围内")
                
                # 输出检查结果
                print("\n【参数检查结果】")
                if param_errors:
                    print("\n发现以下错误:")
                    for error in param_errors:
                        print(f"{error}")
                    print("\n\033[91m请修正以上错误后重试\033[0m")
                    self.control_complete(True)
                    return False
                
                print("\n基本参数检查通过")
                if param_warnings:
                    print("\n优化建议:")
                    for warning in param_warnings:
                        print(f"- {warning}")
                
                # 模拟飞行过程
                print("\n【模拟飞行过程】")
                print(f"目标位置: X={x_target}米, Y={y_target}米, Z={z}米")
                print(f"坐标系: {frame}")
                
                # 根据不同坐标系输出不同的描述
                if frame_lower == "local":
                    print("使用本地坐标系 - 目标为绝对位置")
                elif frame_lower == "local_offset":
                    print("使用本地偏移坐标系 - 目标为相对当前位置的偏移量")
                elif frame_lower == "body":
                    print("使用机体坐标系 - 目标为相对机体前进方向的偏移量")
                    
                print(f"飞行参数: 步长={stepsize}米, 容差={tolerance}米, 轴容差={axisTolerance}米")
                
                # 计算预估的采样点数量
                current_x, current_y = 0, 0  # 假设当前位置
                if frame_lower == "local":
                    # 本地坐标系下，计算从当前位置到目标绝对位置的距离
                    try:
                        current_x, current_y, _ = self.get_current_xyz()
                    except:
                        print("无法获取当前位置，使用(0,0)作为起点进行模拟")
                
                distance = ((x_target - current_x) ** 2 + (y_target - current_y) ** 2) ** 0.5
                points_count = int(distance / stepsize) + 1
                print(f"\n预计飞行距离: {distance:.2f}米")
                print(f"预计采样点数量: {points_count}个")
                
                print("\n模拟飞行完成")
                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True
                
            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("\033[91m建议: 请检查所有输入参数的格式是否正确\033[0m")
                self.control_complete(True)
                return False

        # 实际控制代码
        if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                rate = rospy.Rate(1.0)
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    return False

            rate = rospy.Rate(20)
            hold_yaw = self.get_current_yaw()
            current_x, current_y, current_z = self.get_current_xyz()
            
            # 根据坐标系计算实际目标位置
            if frame.lower() == "local_offset":
                # 使用当前位置计算偏移后的目标位置
                actual_x_target = current_x + x_target
                actual_y_target = current_y + y_target
                actual_z_target = current_z + z  # z为相对当前高度的偏移
            elif frame.lower() == "body":
                # 基于机体坐标系，需要根据当前偏航角计算实际目标位置
                sin_yaw = math.sin(hold_yaw)
                cos_yaw = math.cos(hold_yaw)
                # 将机体坐标系下的偏移转换为世界坐标系下的偏移
                dx = x_target * cos_yaw - y_target * sin_yaw
                dy = x_target * sin_yaw + y_target * cos_yaw
                actual_x_target = current_x + dx
                actual_y_target = current_y + dy
                actual_z_target = current_z + z  # z为相对当前高度的偏移
            else:  # "local"
                actual_x_target = x_target
                actual_y_target = y_target
                actual_z_target = z  # z为绝对高度

            while True:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("control_points_x_y_z_stepsize_frame_tolerance_axisTolerance 线程被中断")
                    break

                current_x, current_y, current_z = self.get_current_xyz()
                
                # 计算各轴距离
                x_distance = actual_x_target - current_x
                y_distance = actual_y_target - current_y
                z_distance = actual_z_target - current_z
                
                # 计算各轴方向
                x_abs_distance = abs(x_distance)
                y_abs_distance = abs(y_distance)
                z_abs_distance = abs(z_distance)
                
                # 计算下一个采样点位置
                if x_abs_distance <= axisTolerance:
                    next_x = actual_x_target
                else:
                    direction_x = 1 if x_distance > 0 else -1
                    next_x = current_x + direction_x * min(stepsize, x_abs_distance)

                if y_abs_distance <= axisTolerance:
                    next_y = actual_y_target
                else:
                    direction_y = 1 if y_distance > 0 else -1
                    next_y = current_y + direction_y * min(stepsize, y_abs_distance)

                if z_abs_distance <= axisTolerance:
                    next_z = actual_z_target
                else:
                    direction_z = 1 if z_distance > 0 else -1
                    next_z = current_z + direction_z * min(stepsize, z_abs_distance)

                # 计算到目标的总距离
                distance_to_target = math.sqrt(x_abs_distance**2 + y_abs_distance**2 + z_abs_distance**2)
                
                # 检查是否所有轴都达到目标位置的容差范围内
                if x_abs_distance < tolerance and y_abs_distance < tolerance and z_abs_distance < tolerance:
                    self.log_info("到达目标位置")
                    break

                target = PositionTarget()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.type_mask = 0b101111111000  # 使用位置控制
                target.position.x = next_x
                target.position.y = next_y
                target.position.z = next_z
                target.yaw = hold_yaw

                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)

                rate.sleep()

            self.control_complete(True)
            return True

    def control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(
        self, x_target, y_target, z, stepsize=0.35, frame="local", tolerance=0.03, axisTolerance=0.1, use_thread=False
    ):
        """
        函数名称: control_points_x_y_z_stepsize_frame_tolerance_axisTolerance
        函数功能: 控制无人机沿直线轨迹飞行，支持不同坐标系
        参数:
        - x_target: 目标位置的X坐标
        - y_target: 目标位置的Y坐标
        - z: 目标高度
        - stepsize: 每个采样点之间的步长（默认0.35米）
        - frame: 坐标系类型，可选值：
          * "local": 本地坐标系，坐标值为绝对位置
          * "local_offset": 本地偏移坐标系，坐标值为相对当前位置的偏移量
          * "body": 机体坐标系，坐标值为相对机体前进方向的偏移量
        - tolerance: 目标位置的容差（默认0.03米）
        - axisTolerance: 轴的容差（默认0.1米）
        - use_thread: 是否使用线程执行，默认为False
        使用案例:
        - 在本地坐标系飞行到绝对位置(2, 3, 1.5): action.control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(2, 3, 1.5)
        - 使用线程方式执行: action.control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(2, 3, 1.5, use_thread=True)
        - 在本地偏移坐标系前进2米，右移3米，上升0.5米: action.control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(2, 3, 0.5, frame="local_offset")
        - 在机体坐标系前进5米，不改变高度: action.control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(5, 0, 0, frame="body")
        - 在机体坐标系右移2米，下降0.5米: action.control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(0, 2, -0.5, frame="body")
        """
        try:
            if use_thread:
                self.thread_control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(
                    x_target, y_target, z, stepsize, frame, tolerance, axisTolerance, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.control_points_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 control_points_x_y_z_stepsize_frame_tolerance_axisTolerance 线程")
                    self.thread_control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(
                        x_target, y_target, z, stepsize, frame, tolerance, axisTolerance, use_thread=False)

                # 直接执行控制函数
                return self._control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(
                    x_target, y_target, z, stepsize, frame, tolerance, axisTolerance)
        except Exception as e:
            self.log_err(f"沿直线轨迹飞行过程中发生错误: {str(e)}")
            self.control_points_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(
        self, x_target, y_target, z, stepsize=0.35, frame="local", tolerance=0.03, axisTolerance=0.1, use_thread=False
    ):
        """
        线程管理函数，用于启动或停止沿直线轨迹飞行控制线程，支持不同坐标系
        """
        try:
            if use_thread:
                # 启动线程
                if self.control_points_thread_running:
                    rospy.logwarn("沿直线轨迹飞行线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.control_points_thread and self.control_points_thread.is_alive():
                        self.control_points_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.control_points_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.control_points_thread = threading.Thread(
                        target=self._control_points_x_y_z_stepsize_frame_tolerance_axisTolerance,
                        args=(x_target, y_target, z, stepsize, frame, tolerance, axisTolerance)
                    )
                    self.control_points_thread.daemon = True  # 设为守护线程
                    self.control_points_thread.start()
                    rospy.loginfo(f"沿直线轨迹飞行线程已启动: 目标=({x_target}, {y_target}, {z}), 坐标系={frame}")
                except Exception as e:
                    rospy.logerr(f"启动沿直线轨迹飞行线程失败: {str(e)}")
                    self.control_points_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.control_points_thread_running:
                    self.control_points_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止沿直线轨迹飞行线程...")
                    if self.control_points_thread and self.control_points_thread.is_alive():
                        self.control_points_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("沿直线轨迹飞行线程已停止")
                else:
                    rospy.logwarn("没有运行中的沿直线轨迹飞行线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.control_points_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------使用多个目标位置采样点控制---------------------------------------#




    # ------------------------位置控制默认local坐标系（自带飞到目标位置后yaw控制）---------------------------------------#
    def _send_position_x_y_z_t_yaw_local(self, x, y, z, yaw_radians, duration):
        """
        函数名称: send_position_x_y_z_t_yaw_local
        函数功能: 在指定时间内不断发送目标位置和偏航角度指令，使无人机移动到指定位置并保持朝向。

        参数:
        - x: 目标位置的 X 坐标，单位为米。
        - y: 目标位置的 Y 坐标，单位为米。
        - z: 目标位置的 Z 坐标（高度），单位为米。
        - yaw_radians: 无人机的偏航角，单位为弧度。
        - duration: 指令发送的持续时间，单位为秒。在此时间段内会重复发送位置和偏航角指令以确保无人机执行。
        """
        # 检查当前执行模式是否为cheak，如果是则检查参数合法性
        if self.ActionExecuteType == "cheak":
            try:
                # 检查参数合法性
                param_errors = []
                param_warnings = []
                
                # 检查坐标值是否为数值类型
                if not isinstance(x, (int, float)):
                    param_errors.append(f"\033[91m错误: X坐标必须为数值类型，当前类型为 {type(x).__name__}\033[0m")
                if not isinstance(y, (int, float)):
                    param_errors.append(f"\033[91m错误: Y坐标必须为数值类型，当前类型为 {type(y).__name__}\033[0m")
                if not isinstance(z, (int, float)):
                    param_errors.append(f"\033[91m错误: Z坐标必须为数值类型，当前类型为 {type(z).__name__}\033[0m")
                if not isinstance(yaw_radians, (int, float)):
                    param_errors.append(f"\033[91m错误: 偏航角必须为数值类型，当前类型为 {type(yaw_radians).__name__}\033[0m")
                if not isinstance(duration, (int, float)):
                    param_errors.append(f"\033[91m错误: 持续时间必须为数值类型，当前类型为 {type(duration).__name__}\033[0m")

                # 检查坐标值范围
                if isinstance(x, (int, float)) and abs(x) > 10.0:
                    param_warnings.append(f"警告: X坐标值 {x} 较大，建议在±10.0米范围内以确保安全飞行")
                if isinstance(y, (int, float)) and abs(y) > 10.0:
                    param_warnings.append(f"警告: Y坐标值 {y} 较大，建议在±10.0米范围内以确保安全飞行")
                
                # 检查高度范围
                if isinstance(z, (int, float)):
                    if z < 0.1:
                        param_errors.append(f"\033[91m错误: 高度值 {z} 米过低，可能导致无人机着陆或碰撞，建议至少设置为0.5米\033[0m")
                    elif z < 0.5:
                        param_warnings.append(f"警告: 高度值 {z} 米较低，建议在0.5-3.0米范围内以确保安全飞行")
                    elif z > 3.0:
                        param_warnings.append(f"警告: 高度值 {z} 米较高，建议在0.5-3.0米范围内以确保安全飞行")

                # 检查偏航角范围
                if isinstance(yaw_radians, (int, float)):
                    yaw_degrees = math.degrees(yaw_radians)
                    if abs(yaw_degrees) > 360:
                        param_warnings.append(f"警告: 偏航角 {yaw_degrees} 度超出了±360度范围，建议使用规范化的角度值")
                else:
                    yaw_degrees = "N/A"

                # 检查持续时间
                if isinstance(duration, (int, float)):
                    if duration <= 0:
                        param_errors.append(f"\033[91m错误: 持续时间 {duration} 必须为正数\033[0m")
                    elif duration < 1.0:
                        param_warnings.append(f"警告: 持续时间 {duration} 秒过短，建议至少设置1秒以上")
                    elif duration > 30.0:
                        param_warnings.append(f"警告: 持续时间 {duration} 秒过长，建议不超过30秒")

                # 打印检查结果
                print("\n参数检查结果:")
                print(f"目标位置: X={x}米, Y={y}米, Z={z}米")
                print(f"目标偏航角: {yaw_degrees}度")
                print(f"控制持续时间: {duration}秒")

                if param_errors:
                    print("\n\033[91m存在以下错误:\033[0m")
                    for error in param_errors:
                        print(f"- {error}")
                    self.control_complete(True)
                    return False

                if param_warnings:
                    print("\n注意以下警告:")
                    for warning in param_warnings:
                        print(f"- {warning}")

                print("\n模拟执行过程:")
                print(f"1. 无人机将在{duration}秒内移动到指定位置")
                print(f"2. 控制频率: 20Hz")
                print(f"3. 预期运动轨迹: 当前位置 -> ({x}, {y}, {z})")
                print("4. 执行完成后将设置控制完成标志")
                
                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True
                
            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("建议: 请检查所有输入参数的格式是否正确")
                self.control_complete(True)
                return False

        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                rospy.logerr("error: 无人机不在OFFBOARD模式下")
                # 等待进入OFFBOARD模式
                rate = rospy.Rate(1.0)  # 设置频率为1Hz
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                    return

            target = PositionTarget()
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            target.type_mask = 0b101111111000  # 设置功能掩码
            target.position.x = x
            target.position.y = y
            target.position.z = z
            target.yaw = yaw_radians
            self.last_command = target         
            rate = rospy.Rate(20)
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                if self.stop_thread_flag.is_set():
                    rospy.logwarn("send_position_x_y_z_t_yaw_local 线程被中断")
                    break
                if self.setpoint_pub:
                    self.setpoint_pub.publish(target)
                self.log_info(f"Real模式: 发布位置指令到 ({x}, {y}, {z})")
                rate.sleep()
                
            self.control_complete(True)  # 设置控制完成标志

    def send_position_x_y_z_t_yaw_local(self, x, y, z, duration, yaw, use_thread=False):
        """
        函数名称: send_position_x_y_z_t_yaw_local
        函数功能: 在指定时间内将无人机移动到指定位置和偏航角
        参数:
        - x: 目标位置X坐标
        - y: 目标位置Y坐标
        - z: 目标位置Z坐标
        - duration: 移动持续时间（秒）
        - yaw: 目标偏航角（弧度）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 移动到(0, 0, 5)位置，偏航角为0，持续10秒: action.send_position_x_y_z_t_yaw_local(0, 0, 5, 10, 0)
        - 使用线程方式移动: action.send_position_x_y_z_t_yaw_local(0, 0, 5, 10, 0, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_send_position_x_y_z_t_yaw_local(x, y, z, duration, yaw, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.position_yaw_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 send_position_x_y_z_t_yaw_local 线程")
                    self.thread_send_position_x_y_z_t_yaw_local(x, y, z, duration, yaw, use_thread=False)
                
                # 直接执行位置和偏航角控制函数
                return self._send_position_x_y_z_t_yaw_local(x, y, z, yaw, duration)
        except Exception as e:
            self.log_err(f"位置和偏航角控制过程中发生错误: {str(e)}")
            self.position_yaw_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志
                
    def thread_send_position_x_y_z_t_yaw_local(self, x, y, z, duration, yaw, use_thread=False):
        """
        线程管理函数，用于启动或停止带偏航角的位置控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.send_position_yaw_thread_running:
                    rospy.logwarn("带偏航角的位置控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.send_position_yaw_thread and self.send_position_yaw_thread.is_alive():
                        self.send_position_yaw_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.send_position_yaw_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.send_position_yaw_thread = threading.Thread(
                        target=self._send_position_x_y_z_t_yaw_local,
                        args=(x, y, z, yaw, duration)
                    )
                    self.send_position_yaw_thread.daemon = True  # 设为守护线程
                    self.send_position_yaw_thread.start()
                    rospy.loginfo(f"带偏航角的位置控制线程已启动: x={x}, y={y}, z={z}, yaw={yaw}, 持续={duration}s")
                except Exception as e:
                    rospy.logerr(f"启动带偏航角的位置控制线程失败: {str(e)}")
                    self.send_position_yaw_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.send_position_yaw_thread_running:
                    self.send_position_yaw_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止带偏航角的位置控制线程...")
                    if self.send_position_yaw_thread and self.send_position_yaw_thread.is_alive():
                        self.send_position_yaw_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("带偏航角的位置控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的带偏航角的位置控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.send_position_yaw_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------位置控制默认local坐标系（自带飞到目标位置后yaw控制）---------------------------------------#




    # ------------------------环绕根据当前高度平面上，某个半径飞行---------------------------------------#
    def _circle_r_z_numpoints(self, radius, z, numpoints=16, tolerance=0.1):
        """
        函数名称: circle_r_z_numpoints
        函数功能: 控制无人机围绕一个柱子飞行，飞行过程中实时获取当前位置，并在一圈结束后返回起始点。

        参数:
        - radius: 无人机与柱子的距离（半径），单位为米。确保这个距离足够大，以免碰撞。
        - z: 飞行高度，单位为米。
        - numpoints: 采样点的数量，默认为16。采样点越多，环绕越细致。
        - tolerance: 容差，用于判断无人机是否返回到起点，单位为米。

        使用案例:
        - drone.circle_r_z_numpoints(radius=1.0, z=1.0, numpoints=16, tolerance=0.1)

        说明:
        - 无人机将围绕柱子飞行，飞行过程中会实时获取当前位置 (x, y)，并确保在一圈飞行完成后回到起点。
        """
        # 检查当前执行模式是否为cheak，如果是则检查参数合法性
        if self.ActionExecuteType == "cheak":
            try:
                # 参数错误列表和警告列表
                param_errors = []
                param_warnings = []
                
                # 检查半径参数
                if not isinstance(radius, (int, float)):
                    param_errors.append(f"\033[91m错误: 半径必须为数值类型，当前类型为 {type(radius).__name__}\033[0m")
                elif radius <= 0:
                    param_errors.append(f"\033[91m错误: 半径必须为正数，当前值为 {radius}\033[0m")
                elif radius < 0.5:
                    param_warnings.append(f"警告: 半径值 {radius}米 较小，可能导致无人机飞行不稳定，建议设置在0.5-5.0米范围内")
                elif radius > 5.0:
                    param_warnings.append(f"警告: 半径值 {radius}米 较大，可能超出视线范围，建议设置在0.5-5.0米范围内")

                # 检查高度参数
                if not isinstance(z, (int, float)):
                    param_errors.append(f"\033[91m错误: 高度必须为数值类型，当前类型为 {type(z).__name__}\033[0m")
                elif z < 0.1:
                    param_errors.append(f"\033[91m错误: 高度值 {z}米 过低，可能导致无人机着陆或碰撞，建议至少设置为0.5米\033[0m")
                elif z < 0.5:
                    param_warnings.append(f"警告: 高度值 {z}米 较低，建议在0.5-3.0米范围内以确保安全飞行")
                elif z > 3.0:
                    param_warnings.append(f"警告: 高度值 {z}米 较高，建议在0.5-3.0米范围内以确保安全飞行")

                # 检查采样点数量
                if not isinstance(numpoints, int):
                    param_errors.append(f"\033[91m错误: 采样点数量必须为整数，当前类型为 {type(numpoints).__name__}\033[0m")
                elif numpoints < 8:
                    param_errors.append(f"\033[91m错误: 采样点数量 {numpoints} 过少，建议设置在8-64个点之间\033[0m")
                elif numpoints > 64:
                    param_warnings.append(f"警告: 采样点数量 {numpoints} 较多，可能影响系统性能，建议设置在8-64个点之间")

                # 检查容差参数
                if not isinstance(tolerance, (int, float)):
                    param_errors.append(f"\033[91m错误: 容差必须为数值类型，当前类型为 {type(tolerance).__name__}\033[0m")
                elif tolerance <= 0:
                    param_errors.append(f"\033[91m错误: 容差必须为正数，当前值为 {tolerance}\033[0m")
                elif tolerance > 0.3:
                    param_warnings.append(f"警告: 容差值 {tolerance}米 较大，可能影响定位精度，建议设置在0.05-0.3米范围内")
                elif tolerance < 0.05:
                    param_warnings.append(f"警告: 容差值 {tolerance}米 较小，可能难以达到目标位置，建议设置在0.05-0.3米范围内")

                # 打印错误和警告信息
                if param_errors:
                    for error in param_errors:
                        print(error)
                    self.control_complete(True)
                    return False

                if param_warnings:
                    for warning in param_warnings:
                        self.log_warn(warning)

                # 模拟环绕飞行过程
                self.log_info(f"模拟环绕飞行: 半径={radius}米, 高度={z}米, 采样点数={numpoints}, 容差={tolerance}米")
                self.log_info("开始环绕飞行模拟...")
                
                for i in range(numpoints):
                    angle = 2 * math.pi * i / numpoints
                    x = radius * math.cos(angle)
                    y = radius * math.sin(angle)
                    self.log_info(f"采样点 {i+1}/{numpoints}: 位置=({x:.2f}, {y:.2f}, {z}), 角度={math.degrees(angle):.1f}°")
                
                self.log_info("环绕飞行模拟完成")
                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                self.control_complete(True)
                return True
                
            except Exception as e:
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("建议: 请检查所有输入参数的格式是否正确")
                self.control_complete(True)
                return False

        elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
            self.control_complete(False)
            if not self.offboard_mask:
                self.log_err("错误: 无人机不在OFFBOARD模式下")
                rate = rospy.Rate(1.0)
                while not rospy.is_shutdown() and not self.offboard_mask:
                    self.log_info("等待进入OFFBOARD模式...")
                    rate.sleep()
                if not self.offboard_mask:
                    self.log_err("超时: 无人机未能进入OFFBOARD模式")
                    return False

            rate = rospy.Rate(20)
            start_x, start_y, start_z = self.get_current_xyz()
            current_yaw = self.get_current_yaw()

            pillar_x = start_x + radius * math.cos(current_yaw)
            pillar_y = start_y + radius * math.sin(current_yaw)

            self.log_info(f"柱子位置: ({pillar_x:.2f}, {pillar_y:.2f}, {z})")

            angle_increment = 2 * math.pi / numpoints
            initial_theta = math.atan2(start_y - pillar_y, start_x - pillar_x)
            theta = initial_theta
            point_counter = 1

            while True:
                if self.stop_thread_flag.is_set():
                    self.log_warn("环绕飞行被中断")
                    break

                current_x, current_y, current_z = self.get_current_xyz()
                target_x = pillar_x + radius * math.cos(theta)
                target_y = pillar_y + radius * math.sin(theta)
                yaw_radians = math.atan2(pillar_y - target_y, pillar_x - target_x)

                self.log_info(f"移动到采样点 {point_counter}/{numpoints}: ({target_x:.2f}, {target_y:.2f}, {z})")
                self.send_position_x_y_z_t_yaw_local(target_x, target_y, z, yaw_radians, 0.4)
                
                point_counter += 1
                theta += angle_increment

                if point_counter > numpoints:
                    delta_x = abs(current_x - start_x)
                    delta_y = abs(current_y - start_y)
                    
                    if delta_x <= tolerance and delta_y <= tolerance:
                        self.log_info("已返回起点位置，环绕飞行完成")
                        break

            self.send_position_x_y_z_t_yaw_local(start_x, start_y, z, current_yaw, 0.8)
            self.control_complete(True)
            return True
        
    def circle_r_z_numpoints_t(self, r, z, numpoints, t, use_thread=False):
        """圆形轨迹控制函数
        
        Args:
            r: 半径
            z: 高度
            numpoints: 点数
            t: 持续时间
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                self.thread_circle_r_z_numpoints_t(r, z, numpoints, t, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.circle_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断圆形轨迹线程")
                    self.thread_circle_r_z_numpoints_t(r, z, numpoints, t, use_thread=False)
            
            # 直接执行圆形轨迹控制函数
            return self._circle_r_z_numpoints(r, z, numpoints, t)
        except Exception as e:
            self.log_err(f"圆形轨迹控制过程中发生错误: {str(e)}")
            self.circle_thread_running = False
            self.control_complete(True)
            return False
        
    def thread_circle_r_z_numpoints_t(self, r, z, numpoints, t, use_thread=False):
        """
        线程管理函数，用于启动或停止圆形轨迹控制线程
        """
        try:
            if use_thread:
                # 启动线程
                if self.circle_thread_running:
                    rospy.logwarn("圆形轨迹控制线程已在运行，将终止旧线程并启动新线程")
                    self.stop_thread_flag.set()  # 设置中断标志
                    if self.circle_thread and self.circle_thread.is_alive():
                        self.circle_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒

                # 重置状态和启动新线程
                self.stop_thread_flag.clear()  # 清除中断标志
                self.circle_thread_running = True
                self.control_complete(False)  # 明确设置为未完成

                try:
                    self.circle_thread = threading.Thread(
                        target=self._circle_r_z_numpoints,
                        args=(r, z, numpoints, t)
                    )
                    self.circle_thread.daemon = True  # 设为守护线程
                    self.circle_thread.start()
                    rospy.loginfo(f"圆形轨迹控制线程已启动: 半径={r}, 高度={z}, 点数={numpoints}, 时间={t}s")
                except Exception as e:
                    rospy.logerr(f"启动圆形轨迹控制线程失败: {str(e)}")
                    self.circle_thread_running = False
                    self.control_complete(True)
                    raise
            else:
                # 停止线程
                if self.circle_thread_running:
                    self.circle_thread_running = False  # 首先设置状态为非运行
                    self.stop_thread_flag.set()  # 设置中断标志
                    rospy.loginfo("正在停止圆形轨迹控制线程...")
                    if self.circle_thread and self.circle_thread.is_alive():
                        self.circle_thread.join(timeout=2.0)  # 等待线程结束，最多等待2秒
                    rospy.loginfo("圆形轨迹控制线程已停止")
                else:
                    rospy.logwarn("没有运行中的圆形轨迹控制线程")

                # 确保设置完成标志
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.circle_thread_running = False
            self.control_complete(True)
            raise  # 向上传递异常
    # ------------------------环绕根据当前高度平面上，某个半径飞行---------------------------------------#




    # ------------------------基于水平方向的不同朝向摄像头安装的视觉追踪（带超时和时间限制）---------------------------------------#
    def _track_velocity_direction_centertol_tout_t(self, direction, timeout, t, MAX_VEL=0.3, MIN_VEL=0, Kp=0.08, center_tolerance=20):
        """
        不同摄像头安装方向的视觉追踪（带超时和总时间限制）：使用速度控制跟踪目标
        direction参数说明：
        - "forward": 摄像头朝前（+x方向）
        - "backward": 摄像头朝后（-x方向）
        - "left": 摄像头朝左（+y方向）
        - "right": 摄像头朝右（-y方向）
        """
        try:
            # 参数检查和建议
            if self.ActionExecuteType == "cheak":
                print("\n参数检查与建议:")
                
                try:
                    has_errors = False
                    # 检查direction参数
                    valid_directions = ["forward", "backward", "left", "right"]
                    if direction not in valid_directions:
                        print(f"\033[91m错误: direction参数 '{direction}' 无效\033[0m")
                        print(f"建议: direction必须是以下值之一: {valid_directions}")
                        has_errors = True
                    
                    # 检查timeout参数
                    if not isinstance(timeout, (int, float)) or timeout <= 0:
                        print(f"\033[91m错误: timeout参数 {timeout} 必须是正数\033[0m")
                        print("建议: timeout建议设置在1-5秒之间")
                        has_errors = True
                    elif timeout > 10:
                        print(f"警告: timeout值 {timeout}秒 过长")
                        print("建议: 建议设置在1-5秒之间，过长的超时可能导致无人机悬停时间过长")
                    
                    # 检查总时间限制t
                    if not isinstance(t, (int, float)) or t <= 0:
                        print(f"\033[91m错误: 总时间限制t参数 {t} 必须是正数\033[0m")
                        print("建议: t建议设置在30-60秒之间")
                        has_errors = True
                    elif t > 120:
                        print(f"警告: 总时间限制t值 {t}秒 过长")
                        print("建议: 建议设置在30-60秒之间，过长的执行时间可能增加追踪失败风险")
                    
                    # 检查最大速度限制
                    if not isinstance(MAX_VEL, (int, float)) or MAX_VEL <= 0:
                        print(f"\033[91m错误: 最大速度MAX_VEL参数 {MAX_VEL} 必须是正数\033[0m")
                        print("建议: MAX_VEL建议设置在0.1-0.5m/s之间")
                        has_errors = True
                    elif MAX_VEL > 1.0:
                        print(f"警告: 最大速度MAX_VEL值 {MAX_VEL}m/s 过大")
                        print("建议: 建议设置在0.1-0.5m/s之间，过大的速度可能导致追踪不稳定")
                    
                    # 检查最小速度限制
                    if not isinstance(MIN_VEL, (int, float)) or MIN_VEL < 0:
                        print(f"\033[91m错误: 最小速度MIN_VEL参数 {MIN_VEL} 必须是非负数\033[0m")
                        print("建议: MIN_VEL建议设置在0-0.1m/s之间")
                        has_errors = True
                    elif MIN_VEL >= MAX_VEL:
                        print(f"\033[91m错误: 最小速度MIN_VEL值 {MIN_VEL} 不能大于等于最大速度MAX_VEL值 {MAX_VEL}\033[0m")
                        print("建议: MIN_VEL应小于MAX_VEL")
                        has_errors = True
                    
                    # 检查比例系数
                    if not isinstance(Kp, (int, float)) or Kp <= 0:
                        print(f"\033[91m错误: 比例系数Kp参数 {Kp} 必须是正数\033[0m")
                        print("建议: Kp建议设置在0.001-0.005之间")
                        has_errors = True
                    elif Kp > 0.01:
                        print(f"警告: 比例系数Kp值 {Kp} 过大")
                        print("建议: 建议设置在0.001-0.005之间，过大的比例系数可能导致控制不稳定")
                    
                    # 检查中心点容差
                    if not isinstance(center_tolerance, (int, float)) or center_tolerance <= 0:
                        print(f"\033[91m错误: 中心点容差center_tolerance参数 {center_tolerance} 必须是正数\033[0m")
                        print("建议: center_tolerance建议设置在10-30像素之间")
                        has_errors = True
                    elif center_tolerance > 50:
                        print(f"警告: 中心点容差center_tolerance值 {center_tolerance}像素 过大")
                        print("建议: 建议设置在10-30像素之间，过大的容差可能影响追踪精度")
                    
                    # 输出参数检查结果
                    if not has_errors:
                        print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                    
                    print("\n模拟执行过程:")
                    print(f"1. 开始{direction}方向追踪控制")
                    print(f"2. 设置追踪参数: 超时={timeout}秒, 总时间限制={t}秒")
                    print(f"3. 设置速度限制: 最大速度={MAX_VEL}m/s, 最小速度={MIN_VEL}m/s")
                    print(f"4. 设置控制参数: 比例系数={Kp}, 中心容差={center_tolerance}像素")
                    print("5. 模拟追踪目标...")
                    print("6. 追踪完成")
                    
                except Exception as e:
                    print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                
                self.track_timeout_flag = False
                self.control_complete(True)
                return True

            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                # 原有的实际执行代码保持不变
                if not hasattr(self, 'count'):
                    self.count = 0
                self.control_complete(False)
                # 初始化超时标志
                self.track_timeout_flag = False
                
                # 记录函数开始执行的时间
                start_time = rospy.Time.now()
                
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    rate = rospy.Rate(1.0)
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(30)
                
                # 相机分辨率参数
                frame_width = 640
                frame_height = 480
                frame_center_x = frame_width / 2
                frame_center_y = frame_height / 2
                
                # 获取初始位置和姿态
                current_x, current_y, current_z = self.get_current_xyz()
                hold_yaw = self.get_current_yaw()
                
                rospy.loginfo(f"开始{direction}方向追踪控制(带超时)，当前高度:{current_z:.2f}m, 最大运行时间:{t}秒")
                consecutive_centered_frames = 0
                required_centered_frames = 5
                target_lost_start_time = None
                
                # 当目标丢失时，只获取一次当前位置
                hover_position = None
                
                while not rospy.is_shutdown():
                    # 检查总运行时间是否超过限制
                    if (rospy.Time.now() - start_time).to_sec() > t:
                        self.log_warn(f"运行时间超过{t}秒限制，结束追踪")
                        break
                        
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn(f"track_velocity_direction_centertol_tout_t ({direction}) 线程被中断")
                        break
                    
                    # 获取当前YOLO目标位置和无人机位置姿态
                    yolo_x, yolo_y = self.get_current_yolo_xy()
                    current_yaw = self.get_current_yaw()

                    if yolo_x != -1.0 and yolo_y != -1.0:
                        # 目标被检测到
                        # 重置悬停位置
                        hover_position = None
                        target_lost_start_time = None
                        
                        # 获取当前位置用于控制
                        current_x, current_y, current_z = self.get_current_xyz()
                        
                        error_x = frame_center_x - yolo_x
                        error_y = frame_center_y - yolo_y
                        
                        # 检查目标是否在中心容差范围内
                        if abs(error_x) <= center_tolerance and abs(error_y) <= center_tolerance:
                            consecutive_centered_frames += 1
                            self.log_info(f"目标在中心范围内，已持续 {consecutive_centered_frames} 帧")
                            if consecutive_centered_frames >= required_centered_frames:
                                self.log_info("目标已稳定在中心范围内，任务完成")
                                self.track_timeout_flag = False  # 成功追踪，清除超时标志
                                break
                        else:
                            consecutive_centered_frames = 0
                        
                        # 根据摄像头方向调整控制逻辑
                        # 无人机坐标系：前进为+x，左为+y，上为+z
                        if direction == "forward":
                            # 朝前看，使用vy和vz控制
                            speed_x = 0  # 不控制前后
                            speed_y = Kp * error_x  # 左右移动（相机x轴对应无人机-y轴）
                            speed_z = Kp * error_y  # 上下移动（相机y轴对应无人机-z轴）
                            self.log_info(f"前向追踪: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m")
                        elif direction == "backward":
                            # 朝后看，使用vy和vz控制
                            speed_x = 0  # 不控制前后
                            speed_y = -Kp * error_x  # 左右移动（相机x轴对应无人机y轴）
                            speed_z = Kp * error_y  # 上下移动（相机y轴对应无人机-z轴）
                            self.log_info(f"后向追踪: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m")
                        elif direction == "left":
                            # 朝左看，使用vx和vz控制
                            speed_x = -Kp * error_x  # 前后移动（相机x轴对应无人机x轴）
                            speed_y = 0  # 不控制左右
                            speed_z = Kp * error_y  # 上下移动（相机y轴对应无人机-z轴）
                            self.log_info(f"左向追踪: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m")
                        elif direction == "right":
                            # 朝右看，使用vx和vz控制
                            speed_x = Kp * error_x  # 前后移动（相机x轴对应无人机-x轴）
                            speed_y = 0  # 不控制左右
                            speed_z = Kp * error_y  # 上下移动（相机y轴对应无人机-z轴）
                            self.log_info(f"右向追踪: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m")
                        else:
                            rospy.logerr(f"未知的摄像头方向: {direction}")
                            self.track_timeout_flag = True
                            break

                        # 应用速度限制
                        speed_x = max(min(speed_x, MAX_VEL), -MAX_VEL)
                        speed_y = max(min(speed_y, MAX_VEL), -MAX_VEL)
                        speed_z = max(min(speed_z, MAX_VEL), -MAX_VEL)
                        
                        # 应用最小速度限制
                        if abs(speed_x) > 0 and abs(speed_x) < MIN_VEL:
                            speed_x = MIN_VEL if speed_x > 0 else -MIN_VEL
                        if abs(speed_y) > 0 and abs(speed_y) < MIN_VEL:
                            speed_y = MIN_VEL if speed_y > 0 else -MIN_VEL
                        if abs(speed_z) > 0 and abs(speed_z) < MIN_VEL:
                            speed_z = MIN_VEL if speed_z > 0 else -MIN_VEL
                        
                        # 计算yaw误差和yaw_rate
                        yaw_error = hold_yaw - current_yaw
                        # 标准化yaw误差到[-pi, pi]
                        if yaw_error > np.pi:
                            yaw_error -= 2 * np.pi
                        elif yaw_error < -np.pi:
                            yaw_error += 2 * np.pi
                        yaw_rate = np.clip(yaw_error * 1.0, -0.5, 0.5)  # 使用比例控制，限制最大旋转速率
                            
                        target = PositionTarget()
                        target.header.stamp = rospy.Time.now()
                        target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                        target.type_mask = 0b011111000111  # 使用速度控制
                        target.velocity.x = speed_x
                        target.velocity.y = speed_y
                        target.velocity.z = speed_z
                        target.yaw_rate = yaw_rate  # 使用yaw_rate来维持yaw角度
                        self.setpoint_pub.publish(target)
                        self.log_info(f"发布速度控制: vx={speed_x:.2f}, vy={speed_y:.2f}, vz={speed_z:.2f}, yaw_rate={yaw_rate:.2f}")
                    else:
                        # 目标丢失
                        # 只在第一次丢失目标时获取当前位置
                        if hover_position is None:
                            hover_position = self.get_current_xyz() + (self.get_current_yaw(),)
                            self.log_warn("目标丢失，记录当前位置用于悬停")
                        
                        if target_lost_start_time is None:
                            target_lost_start_time = rospy.Time.now()
                            self.log_warn("目标丢失，开始计时")
                        elif (rospy.Time.now() - target_lost_start_time).to_sec() > timeout:
                            self.log_warn(f"目标丢失超过{timeout}秒，中断追踪")
                            self.track_timeout_flag = True  # 设置超时标志
                            break
                        
                        self.log_warn("未获取到有效的YOLO目标位置，悬停在记录位置")
                        
                        # 使用位置控制保持记录的位置
                        target = PositionTarget()
                        target.header.stamp = rospy.Time.now()
                        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                        target.type_mask = 0b101111111000  # 使用位置控制
                        target.position.x = hover_position[0]
                        target.position.y = hover_position[1]
                        target.position.z = hover_position[2]
                        target.yaw = hover_position[3]
                        self.setpoint_pub.publish(target)
                        consecutive_centered_frames = 0
                    rate.sleep()
                
                # 结束时重新获取当前位置和姿态，并发送停止命令
                try:
                    current_x, current_y, current_z = self.get_current_xyz()
                    current_yaw = self.get_current_yaw()
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    target.type_mask = 0b101111111000  # 使用位置控制
                    target.position.x = current_x
                    target.position.y = current_y
                    target.position.z = current_z
                    target.yaw = current_yaw
                    self.setpoint_pub.publish(target)
                    self.log_info(f"追踪结束：设置位置控制: x={current_x:.2f}, y={current_y:.2f}, z={current_z:.2f}, yaw={current_yaw:.2f}")
                except Exception as e2:
                    rospy.logerr(f"结束处理过程中发生错误: {str(e2)}")
        except Exception as e:
            rospy.logerr(f"{direction}方向追踪控制过程中发生错误: {str(e)}")
            self.track_timeout_flag = True  # 发生错误时也设置超时标志
            try:
                # 发生错误时，重新获取当前位置和姿态，并尝试悬停
                current_x, current_y, current_z = self.get_current_xyz()
                current_yaw = self.get_current_yaw()
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.type_mask = 0b101111111000  # 使用位置控制
                target.position.x = current_x
                target.position.y = current_y
                target.position.z = current_z
                target.yaw = current_yaw
                self.setpoint_pub.publish(target)
                self.log_info(f"错误处理：设置位置控制: x={current_x:.2f}, y={current_y:.2f}, z={current_z:.2f}, yaw={current_yaw:.2f}")
            except Exception as e2:
                rospy.logerr(f"错误处理过程中发生额外错误: {str(e2)}")
        finally:
            self.control_complete(True)

    def track_velocity_direction_centertol_tout_t(self, direction, timeout=3, t=30, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20, use_thread=False):
        """
        函数名称: track_velocity_direction_centertol_tout_t
        函数功能: 控制无人机根据不同摄像头安装方向进行追踪，带中心点容差、超时限制和总时间限制
        参数:
        - direction: 摄像头安装方向，可选值："forward"(朝前)、"backward"(朝后)、"left"(朝左)、"right"(朝右)
        - timeout: 目标丢失超时时间（秒）
        - t: 函数执行总时间限制（秒）
        - MAX_VEL: 最大速度限制（米/秒）
        - MIN_VEL: 最小速度限制（米/秒）
        - Kp: 比例系数
        - center_tolerance: 中心点容差（像素）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 前向摄像头追踪: action.track_velocity_direction_centertol_tout_t("forward", t=60)
        - 使用线程方式: action.track_velocity_direction_centertol_tout_t("forward", t=60, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_track_velocity_direction_centertol_tout_t(direction, timeout, t, MAX_VEL, MIN_VEL, Kp, center_tolerance, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.track_velocity_direction_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn(f"强制中断 track_velocity_direction_centertol_tout_t ({direction}) 线程")
                    self.thread_track_velocity_direction_centertol_tout_t(direction, timeout, t, MAX_VEL, MIN_VEL, Kp, center_tolerance, use_thread=False)
                
                # 直接执行追踪函数
                return self._track_velocity_direction_centertol_tout_t(direction, timeout, t, MAX_VEL, MIN_VEL, Kp, center_tolerance)
        except Exception as e:
            self.log_err(f"{direction}方向追踪过程中发生错误: {str(e)}")
            self.track_velocity_direction_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_track_velocity_direction_centertol_tout_t(self, direction, timeout=3, t=30, MAX_VEL=0.3, MIN_VEL=0, Kp=0.005, center_tolerance=20, use_thread=False):
        """
        线程管理函数，用于启动或停止不同摄像头方向的带超时和时间限制的追踪控制线程
        """
        try:
            if use_thread:
                # 如果有旧线程在运行，先停止它
                if self.track_velocity_direction_thread_running:
                    self.stop_thread_flag.set()
                    if self.track_velocity_direction_thread and self.track_velocity_direction_thread.is_alive():
                        self.track_velocity_direction_thread.join(timeout=2.0)
                    self.log_info("旧的方向追踪线程已停止")
                # 重置状态并启动新线程
                self.track_velocity_direction_thread_running = True
                self.stop_thread_flag.clear()  # 确保终止标志未设置
                self.control_complete(False)  # 确保控制标志设置为未完成
                self.track_velocity_direction_thread = threading.Thread(
                    target=self._track_velocity_direction_centertol_tout_t,
                    args=(direction, timeout, t, MAX_VEL, MIN_VEL, Kp, center_tolerance)
                )
                self.track_velocity_direction_thread.daemon = True
                self.track_velocity_direction_thread.start()
                self.log_info(f"{direction}方向追踪线程已启动: 中心容差={center_tolerance}像素, 超时={timeout}秒, 总时间限制={t}秒")
            else:
                # 停止线程
                if self.track_velocity_direction_thread_running:
                    self.track_velocity_direction_thread_running = False
                    self.stop_thread_flag.set()  # 设置终止标志
                    if self.track_velocity_direction_thread and self.track_velocity_direction_thread.is_alive():
                        self.track_velocity_direction_thread.join(timeout=2.0)  # 等待最多2秒
                    self.log_warn(f"{direction}方向追踪线程已终止")
                else:
                    self.log_warn("方向追踪线程未在运行")
                # 确保控制标志被设置为完成
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.track_velocity_direction_thread_running = False
            self.control_complete(True)
    # ------------------------基于不同摄像头安装方向的视觉追踪（带超时和时间限制）---------------------------------------#




    # ------------------------基于某个高度平面,摄像头朝下的视觉追踪（带超时）---------------------------------------#
    def _track_velocity_z_centertol_tout_t(self, z, timeout, t, MAX_VEL=0.3, MIN_VEL=0, Kp=0.005, center_tolerance=20):
        """
        视觉追踪（带超时和总运行时间限制）：xy速度控制，z高度采用速度控制
        """
        try:
            # 参数检查和建议
            if self.ActionExecuteType == "cheak":
                try:
                    print("\n参数检查与建议:")
                    param_errors = []
                    param_warnings = []

                    # 检查高度参数
                    if not isinstance(z, (int, float)):
                        param_errors.append(f"\033[91m错误: 高度必须为数值类型，当前类型为 {type(z).__name__}\033[0m")
                    elif z <= 0:
                        param_errors.append(f"\033[91m错误: 高度值 {z}米 必须为正数\033[0m")
                    elif z < 0.5:
                        param_warnings.append(f"警告: 高度值 {z}米 较低，建议在0.5-3.0米范围内以确保安全飞行")
                    elif z > 3.0:
                        param_warnings.append(f"警告: 高度值 {z}米 较高，建议在0.5-3.0米范围内以确保安全飞行")

                    # 检查超时参数
                    if not isinstance(timeout, (int, float)):
                        param_errors.append(f"\033[91m错误: 超时时间必须为数值类型，当前类型为 {type(timeout).__name__}\033[0m")
                    elif timeout <= 0:
                        param_errors.append(f"\033[91m错误: 超时时间 {timeout}秒 必须为正数\033[0m")
                    elif timeout < 1:
                        param_warnings.append(f"警告: 超时时间 {timeout}秒 过短，建议设置在1-5秒范围内")
                    elif timeout > 5:
                        param_warnings.append(f"警告: 超时时间 {timeout}秒 过长，建议设置在1-5秒范围内")

                    # 检查总运行时间参数
                    if not isinstance(t, (int, float)):
                        param_errors.append(f"\033[91m错误: 总运行时间必须为数值类型，当前类型为 {type(t).__name__}\033[0m")
                    elif t <= 0:
                        param_errors.append(f"\033[91m错误: 总运行时间 {t}秒 必须为正数\033[0m")
                    elif t < 5:
                        param_warnings.append(f"警告: 总运行时间 {t}秒 过短，建议设置在10-60秒范围内")
                    elif t > 60:
                        param_warnings.append(f"警告: 总运行时间 {t}秒 过长，建议设置在10-60秒范围内")

                    # 检查速度参数
                    if not isinstance(MAX_VEL, (int, float)):
                        param_errors.append(f"\033[91m错误: 最大速度必须为数值类型，当前类型为 {type(MAX_VEL).__name__}\033[0m")
                    elif MAX_VEL <= 0:
                        param_errors.append(f"\033[91m错误: 最大速度 {MAX_VEL}米/秒 必须为正数\033[0m")
                    elif MAX_VEL > 0.5:
                        param_warnings.append(f"警告: 最大速度 {MAX_VEL}米/秒 过大，建议设置在0.1-0.5米/秒范围内")

                    if not isinstance(MIN_VEL, (int, float)):
                        param_errors.append(f"\033[91m错误: 最小速度必须为数值类型，当前类型为 {type(MIN_VEL).__name__}\033[0m")
                    elif MIN_VEL < 0:
                        param_errors.append(f"\033[91m错误: 最小速度 {MIN_VEL}米/秒 不能为负数\033[0m")
                    elif MIN_VEL >= MAX_VEL:
                        param_errors.append(f"\033[91m错误: 最小速度 {MIN_VEL}米/秒 必须小于最大速度 {MAX_VEL}米/秒\033[0m")

                    # 检查比例系数
                    if not isinstance(Kp, (int, float)):
                        param_errors.append(f"\033[91m错误: 比例系数必须为数值类型，当前类型为 {type(Kp).__name__}\033[0m")
                    elif Kp <= 0:
                        param_errors.append(f"\033[91m错误: 比例系数 {Kp} 必须为正数\033[0m")
                    elif Kp > 0.01:
                        param_warnings.append(f"警告: 比例系数 {Kp} 过大，建议设置在0.001-0.005范围内")
                    elif Kp < 0.001:
                        param_warnings.append(f"警告: 比例系数 {Kp} 过小，建议设置在0.001-0.005范围内")

                    # 检查中心容差
                    if not isinstance(center_tolerance, (int, float)):
                        param_errors.append(f"\033[91m错误: 中心容差必须为数值类型，当前类型为 {type(center_tolerance).__name__}\033[0m")
                    elif center_tolerance <= 0:
                        param_errors.append(f"\033[91m错误: 中心容差 {center_tolerance}像素 必须为正数\033[0m")
                    elif center_tolerance < 10:
                        param_warnings.append(f"警告: 中心容差 {center_tolerance}像素 过小，建议设置在20-50像素范围内")
                    elif center_tolerance > 50:
                        param_warnings.append(f"警告: 中心容差 {center_tolerance}像素 过大，建议设置在20-50像素范围内")

                    # 打印所有错误和警告
                    if param_errors:
                        print("\n参数错误:")
                        for error in param_errors:
                            print(error)
                    if param_warnings:
                        print("\n参数警告:")
                        for warning in param_warnings:
                            print(warning)

                    # 如果有错误，则返回
                    if param_errors:
                        print("\033[91m\n由于存在参数错误，无法执行追踪控制\033[0m")
                        self.control_complete(True)
                        return

                    print("\n模拟执行流程:")
                    print(f"1. 设置目标高度为 {z}米")
                    print(f"2. 设置目标丢失超时时间为 {timeout}秒")
                    print(f"3. 设置最大运行时间为 {t}秒")
                    print(f"4. 速度控制参数: 最大速度={MAX_VEL}米/秒, 最小速度={MIN_VEL}米/秒")
                    print(f"5. 控制参数: 比例系数={Kp}, 中心容差={center_tolerance}像素")
                    print("6. 开始视觉追踪控制循环")
                    print("7. 实时调整飞行高度和水平位置")
                    print("8. 追踪过程中持续检测目标是否在视野中")
                    print("9. 完成追踪任务或触发终止条件后降落")
                    print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                    self.control_complete(True)
                    return
                except Exception as e:
                    print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                    print("建议: 请检查所有输入参数的格式是否正确")
                    self.control_complete(True)
                    return

            if self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                if not hasattr(self, 'count'):
                    self.count = 0
                self.control_complete(False)
                # 初始化超时标志
                self.track_timeout_flag = False
                
                # 记录函数开始执行的时间
                start_time = rospy.Time.now()
                
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    rate = rospy.Rate(1.0)
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                rate = rospy.Rate(30)
                hold_yaw = self.get_current_yaw()
                frame_center_x = 640 / 2
                frame_center_y = 480 / 2
                current_x, current_y, current_z = self.get_current_xyz()
                rospy.loginfo(f"开始追踪控制(带超时)，当前高度:{current_z:.2f}m，目标高度:{z:.2f}m，最大运行时间:{t}秒")
                consecutive_centered_frames = 0
                required_centered_frames = 5
                target_lost_start_time = None
                while not rospy.is_shutdown():
                    # 检查总运行时间是否超过限制
                    if (rospy.Time.now() - start_time).to_sec() > t:
                        self.log_warn(f"运行时间超过{t}秒限制，结束追踪")
                        break
                        
                    if self.stop_thread_flag.is_set():
                        rospy.logwarn("track_velocity_z_centertol_tout_t 线程被中断")
                        break
                    yolo_x, yolo_y = self.get_current_yolo_xy()
                    _, _, current_z = self.get_current_xyz()
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.coordinate_frame = PositionTarget.FRAME_BODY_NED
                    target.type_mask = 0b011111000111  # 使用速度控制

                    # 计算高度误差和垂直速度
                    altitude_error = z - current_z
                    vertical_velocity = np.clip(altitude_error * 2.0, -0.2, 0.2)  # 使用与nav_position_pkg.py相同的控制逻辑

                    if yolo_x != -1.0 and yolo_y != -1.0:
                        target_lost_start_time = None
                        error_x = frame_center_x - yolo_x
                        error_y = frame_center_y - yolo_y
                        if abs(error_x) <= center_tolerance and abs(error_y) <= center_tolerance:
                            consecutive_centered_frames += 1
                            self.log_info(f"目标在中心范围内，已持续 {consecutive_centered_frames} 帧")
                            if consecutive_centered_frames >= required_centered_frames:
                                self.log_info("目标已稳定在中心范围内，任务完成")
                                self.track_timeout_flag = False  # 成功追踪，清除超时标志
                                break
                        else:
                            consecutive_centered_frames = 0
                        self.log_info(f"误差: error_x={error_x}, error_y={error_y}, 当前高度={current_z:.2f}m, 目标高度={z}m, 高度误差={altitude_error:.2f}m")
                        speed_x = Kp * error_y
                        speed_y = Kp * error_x
                        speed_x = max(min(speed_x, MAX_VEL), -MAX_VEL)
                        speed_y = max(min(speed_y, MAX_VEL), -MAX_VEL)
                        if abs(speed_x) > 0 and abs(speed_x) < MIN_VEL:
                            speed_x = MIN_VEL if speed_x > 0 else -MIN_VEL
                        if abs(speed_y) > 0 and abs(speed_y) < MIN_VEL:
                            speed_y = MIN_VEL if speed_y > 0 else -MIN_VEL
                        target.velocity.x = speed_x
                        target.velocity.y = speed_y
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = hold_yaw
                        self.setpoint_pub.publish(target)
                        self.log_info(f"发布速度控制: vx={speed_x:.2f}, vy={speed_y:.2f}, vz={vertical_velocity:.2f}, yaw={hold_yaw:.2f}")
                    else:
                        if target_lost_start_time is None:
                            target_lost_start_time = rospy.Time.now()
                            self.log_warn("目标丢失，开始计时")
                        elif (rospy.Time.now() - target_lost_start_time).to_sec() > timeout:
                            self.log_warn(f"目标丢失超过{timeout}秒，中断追踪")
                            self.track_timeout_flag = True  # 设置超时标志
                            break
                        self.log_warn("未获取到有效的YOLO目标位置，悬停在当前位置")
                        target.velocity.x = 0
                        target.velocity.y = 0
                        target.velocity.z = vertical_velocity  # 使用速度控制高度
                        target.yaw = hold_yaw
                        self.setpoint_pub.publish(target)
                        consecutive_centered_frames = 0
                    rate.sleep()
                
                # 结束时或没有识别到目标时，使用当前位置设定local当前的xyz
                current_x, current_y, current_z = self.get_current_xyz()
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.type_mask = 0b101111111000  # 使用位置控制
                target.position.x = current_x
                target.position.y = current_y
                target.position.z = current_z
                target.yaw = hold_yaw
                self.setpoint_pub.publish(target)
                self.log_info(f"结束追踪，设置位置控制: x={current_x:.2f}, y={current_y:.2f}, z={current_z:.2f}, yaw={hold_yaw:.2f}")
                
                total_time = (rospy.Time.now() - start_time).to_sec()
                rospy.loginfo(f"追踪控制结束: 最终高度={current_z:.2f}m, 目标高度={z}m, 是否超时={self.track_timeout_flag}, 总用时={total_time:.2f}秒")
        except Exception as e:
            rospy.logerr(f"追踪控制过程中发生错误: {str(e)}")
            self.track_timeout_flag = True  # 发生错误时也设置超时标志
            
            # 发生错误时也使用当前位置设定
            try:
                current_x, current_y, current_z = self.get_current_xyz()
                hold_yaw = self.get_current_yaw()
                target = PositionTarget()
                target.header.stamp = rospy.Time.now()
                target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                target.type_mask = 0b101111111000  # 使用位置控制
                target.position.x = current_x
                target.position.y = current_y
                target.position.z = current_z
                target.yaw = hold_yaw
                self.setpoint_pub.publish(target)
                self.log_info(f"错误处理：设置位置控制: x={current_x:.2f}, y={current_y:.2f}, z={current_z:.2f}, yaw={hold_yaw:.2f}")
            except Exception as e2:
                rospy.logerr(f"错误处理过程中发生额外错误: {str(e2)}")
        finally:
            self.control_complete(True)

    def track_velocity_z_centertol_tout_t(self, z, timeout=3, t=10, MAX_VEL=0.3, MIN_VEL=0, Kp=0.001, center_tolerance=20, use_thread=False):
        """
        函数名称: track_velocity_z_centertol_tout_t
        函数功能: 控制无人机追踪指定高度，带中心点容差、超时限制和总运行时间限制，基于某个高度平面,摄像头朝下的视觉追踪（带超时）
        参数:
        - z: 目标高度（米）
        - timeout: 目标丢失超时时间（秒）
        - t: 总运行时间限制（秒）
        - MAX_VEL: 最大速度限制（米/秒）
        - MIN_VEL: 最小速度限制（米/秒）
        - Kp: 比例系数
        - center_tolerance: 中心点容差（像素）
        - use_thread: 是否使用线程方式执行，默认为False
        使用案例:
        - 追踪1.5米高度，最大运行20秒: action.track_velocity_z_centertol_tout_t(1.5, t=20)
        - 使用线程方式: action.track_velocity_z_centertol_tout_t(1.5, t=20, use_thread=True)
        """
        try:
            if use_thread:
                self.thread_track_velocity_z_centertol_tout_t(z, timeout, t, MAX_VEL, MIN_VEL, Kp, center_tolerance, use_thread=True)
                return True
            else:
                # 如果线程正在运行，先停止它
                if self.track_velocity_z_centertol_tout_t_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 track_velocity_z_centertol_tout_t 线程")
                    self.thread_track_velocity_z_centertol_tout_t(z, timeout, t, MAX_VEL, MIN_VEL, Kp, center_tolerance, use_thread=False)
                
                # 直接执行高度追踪函数
                return self._track_velocity_z_centertol_tout_t(z, timeout, t, MAX_VEL, MIN_VEL, Kp, center_tolerance)
        except Exception as e:
            self.log_err(f"高度追踪过程中发生错误: {str(e)}")
            self.track_velocity_z_centertol_tout_t_thread_running = False
            self.control_complete(True)
            return False
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志


    def thread_track_velocity_z_centertol_tout_t(self, z, timeout=3, t=10, MAX_VEL=0.3, MIN_VEL=0, Kp=0.005, center_tolerance=20, use_thread=False):
        """
        线程管理函数，用于启动或停止带超时和总运行时间限制的追踪控制线程
        """
        try:
            if use_thread:
                # 如果有旧线程在运行，先停止它
                if self.track_velocity_z_centertol_tout_t_thread_running:
                    self.stop_thread_flag.set()
                    if self.track_velocity_z_centertol_tout_t_thread and self.track_velocity_z_centertol_tout_t_thread.is_alive():
                        self.track_velocity_z_centertol_tout_t_thread.join(timeout=2.0)
                    self.log_info("旧的追踪线程已停止")
                # 重置状态并启动新线程
                self.track_velocity_z_centertol_tout_t_thread_running = True
                self.stop_thread_flag.clear()  # 确保终止标志未设置
                self.control_complete(False)  # 确保控制标志设置为未完成
                self.track_velocity_z_centertol_tout_t_thread = threading.Thread(
                    target=self._track_velocity_z_centertol_tout_t,
                    args=(z, timeout, t, MAX_VEL, MIN_VEL, Kp, center_tolerance)
                )
                self.track_velocity_z_centertol_tout_t_thread.daemon = True
                self.track_velocity_z_centertol_tout_t_thread.start()
                self.log_info(f"追踪线程已启动: z={z}, 中心容差={center_tolerance}像素, 超时={timeout}秒, 总运行时间限制={t}秒")
            else:
                # 停止线程
                if self.track_velocity_z_centertol_tout_t_thread_running:
                    self.track_velocity_z_centertol_tout_t_thread_running = False
                    self.stop_thread_flag.set()  # 设置终止标志
                    if self.track_velocity_z_centertol_tout_t_thread and self.track_velocity_z_centertol_tout_t_thread.is_alive():
                        self.track_velocity_z_centertol_tout_t_thread.join(timeout=2.0)  # 等待最多2秒
                    self.log_warn("追踪线程已终止")
                else:
                    self.log_warn("追踪线程未在运行")
                # 确保控制标志被设置为完成
                self.control_complete(True)
        except Exception as e:
            self.log_err(f"线程管理过程中发生错误: {str(e)}")
            self.track_velocity_z_centertol_tout_t_thread_running = False
            self.control_complete(True)
    # ------------------------基于某个高度平面,摄像头朝下的视觉追踪（带超时）---------------------------------------#





    # ------------------------发布导航目标（带位置容差）---------------------------------------#
    def _publish_nav_goal_x_y_z_yaw_tol_frame(self, x, y, z, yaw, tolerance, frame):
        """
        函数名称: publish_nav_goal_x_y_z_yaw_tol_frame
        函数功能: 发布导航目标，并等待无人机到达目标位置的容差范围内（默认0.05米）
        参数:
        - x: 目标位置的X坐标
        - y: 目标位置的Y坐标
        - z: 目标位置的Z坐标
        - yaw: 目标偏航角（度）
        - frame: 坐标系（1表示'map'，2表示'base_link'）
        """
        try:
            # 检查当前执行模式是否为cheak，如果是则检查参数合法性
            if self.ActionExecuteType == "cheak":
                print("\n参数检查与建议:")
                param_errors = []
                param_warnings = []

                # 检查坐标值是否为数值类型
                if not isinstance(x, (int, float)):
                    param_errors.append(f"\033[91m错误: X坐标必须为数值类型，当前类型为 {type(x).__name__}\033[0m")
                if not isinstance(y, (int, float)):
                    param_errors.append(f"\033[91m错误: Y坐标必须为数值类型，当前类型为 {type(y).__name__}\033[0m")
                if not isinstance(z, (int, float)):
                    param_errors.append(f"\033[91m错误: Z坐标必须为数值类型，当前类型为 {type(z).__name__}\033[0m")

                # 检查坐标值是否在合理范围内
                if isinstance(x, (int, float)) and abs(x) > 10.0:
                    param_warnings.append(f"警告: X坐标值 {x} 米较大，建议在±10.0米范围内以确保安全飞行")
                if isinstance(y, (int, float)) and abs(y) > 10.0:
                    param_warnings.append(f"警告: Y坐标值 {y} 米较大，建议在±10.0米范围内以确保安全飞行")

                # 检查高度是否在合理范围内
                if isinstance(z, (int, float)):
                    if z < 0.1:
                        param_errors.append(f"\033[91m错误: 高度值 {z} 米过低，可能导致无人机着陆或碰撞，建议至少设置为0.5米\033[0m")
                    elif z < 0.5:
                        param_warnings.append(f"警告: 高度值 {z} 米较低，建议在0.5-3.0米范围内以确保安全飞行")
                    elif z > 3.0:
                        param_warnings.append(f"警告: 高度值 {z} 米较高，建议在0.5-3.0米范围内以确保安全飞行")

                # 检查偏航角是否合法
                if not isinstance(yaw, (int, float)):
                    param_errors.append(f"\033[91m错误: 偏航角必须为数值类型，当前类型为 {type(yaw).__name__}\033[0m")
                elif abs(yaw) > 360:
                    param_warnings.append(f"警告: 偏航角 {yaw} 度超出±360度范围，建议规范化到该范围内")

                # 检查容差值是否合理
                if not isinstance(tolerance, (int, float)):
                    param_errors.append(f"\033[91m错误: 容差值必须为数值类型，当前类型为 {type(tolerance).__name__}\033[0m")
                elif tolerance <= 0:
                    param_errors.append(f"\033[91m错误: 容差值必须为正数，当前值为 {tolerance}\033[0m")
                elif tolerance < 0.01:
                    param_warnings.append(f"警告: 容差值 {tolerance} 米过小，可能导致无人机难以到达目标位置，建议设置在0.01-0.1米范围内")
                elif tolerance > 0.1:
                    param_warnings.append(f"警告: 容差值 {tolerance} 米过大，可能影响定位精度，建议设置在0.01-0.1米范围内")

                # 检查坐标系参数
                if not isinstance(frame, int):
                    param_errors.append(f"\033[91m错误: 坐标系参数必须为整数，当前类型为 {type(frame).__name__}\033[0m")
                elif frame not in [1, 2]:
                    param_errors.append(f"\033[91m错误: 坐标系参数必须为1(map)或2(base_link)，当前值为 {frame}\033[0m")

                # 打印错误和警告信息
                if param_errors:
                    print("\n\033[91m严重错误:\033[0m")
                    for error in param_errors:
                        print(error)
                    print("\n\033[91m请修正以上错误后重试\033[0m")
                    return
                
                if param_warnings:
                    print("\n注意事项:")
                    for warning in param_warnings:
                        print(warning)

                print(f"\n模拟执行: 发布导航目标到 (x: {x}, y: {y}, z: {z}, yaw: {yaw}), 容差: {tolerance}, 坐标系: {frame}")
                print("\033[92m参数检查通过，所有输入参数合法\033[0m")
                return

            elif self.ActionExecuteType in ["real", "real_nolog", "sim", "sim_nolog"]:
                self.control_complete(False)
                if not self.offboard_mask:
                    rospy.logerr("error: 无人机不在OFFBOARD模式下")
                    # 等待进入OFFBOARD模式
                    rate = rospy.Rate(1.0)  # 设置频率为1Hz
                    while not rospy.is_shutdown() and not self.offboard_mask:
                        self.log_info("等待进入OFFBOARD模式...")
                        rate.sleep()
                    if not self.offboard_mask:
                        rospy.logerr("超时: 无人机未能进入OFFBOARD模式")
                        return
                if "nolog" not in self.ActionExecuteType:
                    self.log_info(f"Publishing nav goal with tolerance: x={x}, y={y}, z={z}, yaw={yaw}, frame={frame}")

                publish_nav_goal_x_y_z_yaw_tol_frame(
                    x, y, z, yaw, tolerance, frame, self.setpoint_pub, self.nav_goal_pub, self.log_info, self.ActionExecuteType, stop_thread_flag=self.stop_thread_flag
                )
                
                self.control_complete(True)  # 设置控制完成标志

        except Exception as e:
            self.log_err(f"导航过程中发生错误: {str(e)}")
            if self.ActionExecuteType == "cheak":
                print(f"\033[91m参数检查过程中发生错误: {str(e)}\033[0m")
                print("建议: 请检查所有输入参数的格式是否正确")
            raise
        finally:
            self.control_complete(True)  # 确保在任何情况下都设置控制完成标志
            if self.nav_goal_thread_running:
                self.nav_goal_thread_running = False  # 重置线程运行标志

    def publish_nav_goal_x_y_z_yaw_tol_frame(self, x, y, z, yaw, tolerance, frame, use_thread=False):
        """
        主控制函数，用于管理带容差的导航控制过程
        """
        try:
            if use_thread:
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame, use_thread=True)
                if self.control_complete():
                    self.stop_thread_flag.set()
                    self.thread_publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame, use_thread=False)
            else:
                if self.nav_goal_thread_running:
                    self.stop_thread_flag.set()  # 设置终止标志
                    self.log_warn("强制中断 publish_nav_goal_x_y_z_yaw_tol_frame 线程")
                    self.thread_publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame, use_thread=False)
                else:
                    self.log_warn("publish_nav_goal_x_y_z_yaw_tol_frame 线程未在运行，无法中断")
        except Exception as e:
            self.log_err(f"导航控制过程中发生错误: {str(e)}")
            self.nav_goal_thread_running = False
            self.control_complete(True)
        finally:
            if not use_thread:
                self.stop_thread_flag.clear()  # 重置终止标志

    def thread_publish_nav_goal_x_y_z_yaw_tol_frame(self, x, y, z, yaw, tolerance, frame, use_thread=False):
        """发布导航目标点线程函数
        
        Args:
            x: 目标x坐标
            y: 目标y坐标
            z: 目标z坐标
            yaw: 目标偏航角
            tolerance: 容差
            frame: 坐标系
            use_thread: 是否使用线程执行
        """
        try:
            if use_thread:
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_running = True
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_stop = False
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_thread = threading.Thread(
                    target=self._publish_nav_goal_x_y_z_yaw_tol_frame,
                    args=(x, y, z, yaw, tolerance, frame)
                )
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_thread.start()
                return True
            else:
                if self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_running:
                    self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_stop = True
                    self.log_warn("发布导航目标点线程正在运行，已设置停止标志")
                else:
                    self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_stop = False
                    self._publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame)
        except Exception as e:
            self.log_err(f"发布导航目标点线程函数错误: {str(e)}")
        finally:
            if not use_thread:
                self.thread_publish_nav_goal_x_y_z_yaw_tol_frame_stop = False
     # ------------------------发布导航目标（带位置容差）---------------------------------------#




    def ensure_count_exists(self):
        """
        确保count属性存在，如果不存在则初始化为0
        """
        if not hasattr(self, 'count'):
            self.count = 0
        return self.count
    



    def stop_all_threads(self):
        """
        函数名称: stop_all_threads
        函数功能: 停止所有正在运行的线程
        使用案例:
        - 在发生错误时停止所有线程: action.stop_all_threads()
        """
        try:
            # 设置终止标志
            self.stop_thread_flag.set()
            
            # 停止所有线程
            thread_list = [
                (self.send_position_thread_running, self.send_position_thread, "位置控制"),
                (self.hover_thread_running, self.hover_thread, "悬停控制"),
                (self.unlock_thread_running, self.unlock_thread, "解锁控制"),
                (self.lock_thread_running, self.lock_thread, "锁定控制"),
                (self.takeoff_thread_running, self.takeoff_thread, "起飞控制"),
                (self.land_auto_thread_running, self.land_auto_thread, "自动降落"),
                (self.nav_goal_thread_running, self.nav_goal_thread, "导航控制"),
                (self.control_yaw_thread_running, self.control_yaw_thread, "偏航角控制"),
                (self.velocity_thread_running, self.velocity_thread, "速度控制"),
                (self.land_lock_thread_running, self.land_lock_thread, "降落锁定"),
                (self.control_points_thread_running, self.control_points_thread, "点控制"),
                (self.position_yaw_thread_running, self.position_yaw_thread, "位置偏航"),
                (self.circle_thread_running, self.circle_thread, "圆形轨迹"),
                (self.track_velocity_thread_running, self.track_velocity_thread, "速度追踪"),
                (self.time_sleep_thread_running, self.time_sleep_thread, "延时"),
                (self.tracking_thread_running, self.tracking_thread, "追踪"),
            ]
            
            for running_flag, thread, name in thread_list:
                if running_flag and thread and thread.is_alive():
                    self.log_warn(f"正在停止{name}线程...")
                    thread.join(timeout=2.0)  # 等待最多2秒
                    self.log_warn(f"{name}线程已停止")
            
            # 重置所有运行标志
            self.send_position_thread_running = False
            self.hover_thread_running = False
            self.unlock_thread_running = False
            self.lock_thread_running = False
            self.takeoff_thread_running = False
            self.land_auto_thread_running = False
            self.nav_goal_thread_running = False
            self.control_yaw_thread_running = False
            self.velocity_thread_running = False
            self.land_lock_thread_running = False
            self.control_points_thread_running = False
            self.position_yaw_thread_running = False
            self.circle_thread_running = False
            self.track_velocity_thread_running = False
            self.time_sleep_thread_running = False
            self.tracking_thread_running = False
            
            # 清除终止标志
            self.stop_thread_flag.clear()
            
            # 设置控制完成标志
            self.control_complete(True)
            
            self.log_info("所有线程已停止")
        except Exception as e:
            self.log_err(f"停止线程过程中发生错误: {str(e)}")
            raise