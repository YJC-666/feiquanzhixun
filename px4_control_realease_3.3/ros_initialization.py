#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
by:杨锦成
vx:18873833517
time:2025.5.29
file_name:ros_initialization.py
'''

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String, UInt16
from geometry_msgs.msg import Twist
import tf

# 尝试导入YoloDetections消息类型
YOLOV8_RKNN_AVAILABLE = False
try:
    from yolov8_rknn_detect.msg import YoloDetections
    YOLOV8_RKNN_AVAILABLE = True
    rospy.loginfo("成功导入yolov8_rknn_detect包")
except ImportError:
    rospy.logwarn("未能导入yolov8_rknn_detect包，相关功能将被禁用，若是仿真环境无需理会")

# 全局变量存储最新的cmd_vel消息和当前位置
cmd_vel_msg = None
current_position_x = 0.0
current_position_y = 0.0
current_position_z = 0.0  # 当前高度
current_yaw = 0.0  # 当前航向角（yaw）

# 回调函数定义
def state_callback(data, action_instance):
    action_instance.state = data  # 更新无人机状态

def odom_callback(data, action_instance):
    # 更新当前位置
    action_instance.current_position_x = data.pose.pose.position.x
    action_instance.current_position_y = data.pose.pose.position.y
    action_instance.current_position_z = data.pose.pose.position.z

    # 更新yaw角度
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, action_instance.current_yaw) = tf.transformations.euler_from_quaternion(orientation_list)

def cmd_vel_callback(msg):
    global cmd_vel_msg
    cmd_vel_msg = msg  # 保存最新的cmd_vel消息

def servo_status_callback(msg, action_instance):
    """
    舵机状态回调函数，处理来自舵机控制节点的状态反馈
    参数:
    - msg: UInt16类型消息，包含舵机状态码
    - action_instance: Action_t类的实例，用于更新类的属性
    状态码含义:
    - 1: 舵机控制成功
    - 0: 舵机控制失败
    - 2: 舵机控制超时
    """
    action_instance.servo_status = msg.data
    action_instance.servo_status_received = True
    
    # 记录最后一次的舵机状态
    action_instance.last_servo_status = msg.data
    
    # 根据状态码输出不同的日志信息
    if msg.data == 1:
        if hasattr(action_instance, 'log_info'):
            action_instance.log_info("舵机控制成功")
    elif msg.data == 0:
        if hasattr(action_instance, 'log_warn'):
            action_instance.log_warn("舵机控制失败")
    elif msg.data == 2:
        if hasattr(action_instance, 'log_err'):
            action_instance.log_err("舵机控制超时")
    else:
        if hasattr(action_instance, 'log_warn'):
            action_instance.log_warn(f"收到未知舵机状态: {msg.data}")

def yolo_callback(data, action_instance):#无需yolov8_rknn_detect包作为依赖，可以作为普通opencv识别传入的接口，也可以作为yolo识别传入的接口
    """
    回调函数：处理来自 /ai_detect_info 话题的消息，更新目标X和Y坐标和类别。
    
    参数:
    - data (String): 从 /ai_detect_info 话题接收到的消息，包含目标的X和Y坐标信息。
    - action_instance (Action_t): Action_t 类的实例，用于更新类的属性。
    """
    #在ai_detect_info话题中，无识别的类被是not_found
    try:
        if not data.data.strip():
            action_instance.yolo_x = -1.0
            action_instance.yolo_y = -1.0
            action_instance.yolo_class = 'not_found'
            rospy.logwarn("收到not_found的 /ai_detect_info 消息")
            return

        # 按分号分割每个检测结果
        detections = data.data.split(';')
        for det in detections:
            det = det.strip()
            if not det:
                continue

            # 分割每个部分，期望格式为 'circle: x:320 y:240'
            parts = det.split()
            if len(parts) < 3:
                continue

            # 提取类别
            class_part = parts[0]
            if ':' in class_part:
                yolo_class = class_part.replace(':', '').strip()
            else:
                yolo_class = 'not_found'

            # 提取x和y坐标值
            try:
                x_str = parts[1].split(':')[1]
                y_str = parts[2].split(':')[1].rstrip(';')
                x = float(x_str)
                y = float(y_str)

                # 更新目标坐标和类别
                action_instance.yolo_x = x
                action_instance.yolo_y = y
                action_instance.yolo_class = yolo_class
                #rospy.loginfo(f"更新目标坐标: x={x}, y={y}, class={yolo_class}")
                return  # 找到有效坐标后立即返回

            except (IndexError, ValueError) as e:
                rospy.logwarn(f"无法解析坐标值: {det}，错误: {e}")
                continue

        # 如果没有找到有效的检测结果，设置为无效值
        action_instance.yolo_x = -1.0
        action_instance.yolo_y = -1.0
        action_instance.yolo_class = 'not_found'

    except Exception as e:
        rospy.logwarn(f"处理 /ai_detect_info 消息时发生错误: {e}")
        action_instance.yolo_x = -1.0
        action_instance.yolo_y = -1.0
        action_instance.yolo_class = 'not_found'

if YOLOV8_RKNN_AVAILABLE:
    def yolov8_rknn_callback(data, action_instance):
        """
        回调函数：处理来自 /yolov8_rknn/detection_result 话题的消息，更新目标的中心坐标、宽度、高度和类别。
        
        参数:
        - data (YoloDetections): 从 /yolov8_rknn/detection_result 话题接收到的消息
        - action_instance (Action_t): Action_t 类的实例，用于更新类的属性
        """
        try:
            # 初始化默认值
            action_instance.yolo_result_x = -1.0
            action_instance.yolo_result_y = -1.0
            action_instance.yolo_result_width = -1.0
            action_instance.yolo_result_height = -1.0
            action_instance.yolo_result_class = 'None'#因为在yolov8的识别结果话题，无识别类别是空None的，而在ai_detect_info话题中，无识别类别是not_found
            
            # 检查是否有检测结果
            if not data.detections:
                return
            
            # 取置信度最高的一个检测结果
            best_detection = None
            highest_confidence = -1.0
            
            for detection in data.detections:
                if detection.confidence > highest_confidence:
                    highest_confidence = detection.confidence
                    best_detection = detection
            
            # 如果找到有效的检测结果，更新目标信息
            if best_detection:
                action_instance.yolo_x = best_detection.x_center
                action_instance.yolo_y = best_detection.y_center
                action_instance.yolo_result_width = best_detection.width
                action_instance.yolo_result_height = best_detection.height
                action_instance.yolo_class = best_detection.class_name
                
                # 日志输出(在非nolog模式下)
                if not hasattr(action_instance, 'ActionExecuteType') or not action_instance.ActionExecuteType.endswith("nolog"):
                    rospy.loginfo(f"检测到目标: class={best_detection.class_name}, x={best_detection.x_center}, "
                                 f"y={best_detection.y_center}, width={best_detection.width}, height={best_detection.height}, "
                                 f"confidence={best_detection.confidence}")
        
        except Exception as e:
            rospy.logwarn(f"处理 /yolov8_rknn/detection_result 消息时发生错误: {e}")
            # 出错时设置默认值
            action_instance.yolo_x = -1.0
            action_instance.yolo_y = -1.0
            action_instance.yolo_result_width = -1.0
            action_instance.yolo_result_height = -1.0
            action_instance.yolo_class = 'not_found'

# ROS初始化函数
def init_ros_publishers():
    setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    nav_goal_pub = rospy.Publisher('nav_goal', Float32MultiArray, queue_size=10)
    return setpoint_pub, nav_goal_pub

def sim_init_ros_publishers():
    setpoint_pub = rospy.Publisher('/iris_0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    nav_goal_pub = rospy.Publisher('nav_goal', Float32MultiArray, queue_size=10)
    return setpoint_pub, nav_goal_pub

def sim_init_ros_services():
    rospy.wait_for_service('/iris_0/mavros/cmd/arming')
    rospy.wait_for_service('/iris_0/mavros/set_mode')
    rospy.wait_for_service('/iris_0/mavros/cmd/command')
    arming_service = rospy.ServiceProxy('/iris_0/mavros/cmd/arming', CommandBool)
    set_mode_service = rospy.ServiceProxy('/iris_0/mavros/set_mode', SetMode)
    command_service = rospy.ServiceProxy('/iris_0/mavros/cmd/command', CommandLong)
    return arming_service, set_mode_service, command_service

def init_ros_services():
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    rospy.wait_for_service('/mavros/cmd/command')
    arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
    return arming_service, set_mode_service, command_service

def init_ros_subscribers(action_instance):
    rospy.Subscriber('/mavros/state', State, state_callback, callback_args=action_instance, queue_size=10)
    rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback, callback_args=action_instance, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback, queue_size=2)
    rospy.Subscriber('/ai_detect_info', String, yolo_callback, callback_args=action_instance, queue_size=10)
    # 只有在成功导入yolov8_rknn_detect包的情况下才订阅相关话题
    if YOLOV8_RKNN_AVAILABLE:
        rospy.Subscriber('/yolov8_rknn/detection_result', YoloDetections, yolov8_rknn_callback, callback_args=action_instance, queue_size=10)
        rospy.loginfo("已订阅yolov8_rknn检测结果话题")
    # 添加舵机状态订阅
    rospy.Subscriber('/servo_status', UInt16, servo_status_callback, callback_args=action_instance, queue_size=10)
    rospy.loginfo("已订阅舵机状态话题")

def sim_init_ros_subscribers(action_instance):
    rospy.Subscriber('/iris_0/mavros/state', State, state_callback, callback_args=action_instance, queue_size=10)
    rospy.Subscriber('/iris_0/mavros/local_position/odom', Odometry, odom_callback, callback_args=action_instance, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback, queue_size=2)
    rospy.Subscriber('/ai_detect_info', String, yolo_callback, callback_args=action_instance, queue_size=10)
    # 只有在成功导入yolov8_rknn_detect包的情况下才订阅相关话题
    if YOLOV8_RKNN_AVAILABLE:
        rospy.Subscriber('/yolov8_rknn/detection_result', YoloDetections, yolov8_rknn_callback, callback_args=action_instance, queue_size=10)
        rospy.loginfo("已订阅yolov8_rknn检测结果话题")
    # 添加舵机状态订阅
    rospy.Subscriber('/servo_status', UInt16, servo_status_callback, callback_args=action_instance, queue_size=10)
    rospy.loginfo("已订阅舵机状态话题")

def init_ros_node(node_name):
    rospy.init_node(node_name, anonymous=True)

def get_current_yaw():
    global current_yaw
    return current_yaw
