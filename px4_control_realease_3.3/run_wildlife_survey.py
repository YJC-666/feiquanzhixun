#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
import json
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import yaml
import math
import os
from datetime import datetime

class WildlifeSurveyStation(QMainWindow):
    # 添加信号定义
    update_position_signal = pyqtSignal(list)
    update_status_signal = pyqtSignal(str, int)
    update_wildlife_display_signal = pyqtSignal()
    update_map_signal = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        
        # 初始化ROS节点
        rospy.init_node('wildlife_survey_station', anonymous=True)
        
        # 创建发布者
        self.command_pub = rospy.Publisher('/mission_command', Int32, queue_size=10)
        self.drone_area_pub = rospy.Publisher('/drone_aera_id', String, queue_size=10)  # 发布当前区域ID
        
        # 创建订阅者
        rospy.Subscriber('iris_0/mavros/local_position/odom', Odometry, self.odom_callback, queue_size=10)
        
        # 网格参数 - 63个50cm×50cm方格
        # 行：B1-B7（7行，B1在底部，B7在顶部）
        # 列：A1-A9（9列，A1在左侧，A9在右侧）
        # 总计：9×7 = 63个方格，每个方格50cm×50cm
        self.grid_rows = 7  # 行数 (B1-B7，共7行)
        self.grid_cols = 9  # 列数 (A1-A9，共9列)
        self.cell_size = 0.5  # 每个方格边长50cm = 0.5米
        
        # 红点位置 (起降点) - 设置在B1 A9位置
        self.red_point = (8, 6)  # B1 A9位置 (A9列B1行，即右下角)

        # 全局坐标系设置：以红点为原点，X向前(B1->B7)，Y向左(A9->A1)
        self.origin_col, self.origin_row = self.red_point  # 原点位置设置为红点位置

        # 高度设置
        self.takeoff_height = 1.22  # 起飞高度
        self.landing_height = 0.0  # 降落高度
        self.survey_height = 1.22  # 巡查高度
        
        # 禁区列表
        self.forbidden_zones = []
        
        # 航点列表
        self.waypoints = []
        
        # 返回路径起始索引（用于区分巡查路径和返回路径）
        self.return_path_start_index = -1
        
        # 任务状态
        self.mission_active = False
        
        # 任务完成标志
        self.mission_completed = False
        
        # 无人机是否回到原点的标志
        self.drone_returned_to_origin = False
        
        # 回到原点的检测距离阈值(米)
        self.return_to_origin_threshold = 0.3
        
        # 无人机返航状态标志
        self.is_returning = False
        
        # 上一次保存数据的时间
        self.last_save_time = 0
        
        # 计时功能相关属性
        self.mission_start_time = None  # 任务开始时间
        self.mission_duration = 0  # 任务持续时间（秒）
        
        self.timer_active = False  # 计时器是否激活
        self.timer_label = None  # 计时显示标签
        self.mission_timer = QTimer()  # 任务计时器
        self.mission_timer.timeout.connect(self.update_mission_time)
        
        
        # 无人机在线状态
        self.drone_online = False
        self.last_odom_time = 0  # 上次收到odom消息的时间
        self.drone_status_label = None  # 无人机状态显示标签
        
        # 首次接收到odom消息的标志和时间
        self.first_odom_received = False
        self.first_odom_time = 0
        
        # 在线状态检测定时器
        self.online_check_timer = QTimer()
        self.online_check_timer.timeout.connect(self.check_drone_online_status)
        self.online_check_timer.start(100)  # 每100毫秒检查一次，10Hz
        
        # 定时器用于发布命令
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_command)
        self.timer.start(100)  # 10Hz

        # ROS发布器 - 用于发布航点数据
        self.waypoint_publisher = rospy.Publisher('/wildlife_survey/waypoints', String, queue_size=10)
        
        # 无人机当前位置
        self.drone_position = [0, 0, 0]  # [x, y, z] 单位：米
        
        # 无人机初始位置（用于坐标系初始化）
        self.drone_initial_position = None
        
        # 是否已初始化坐标系
        self.coordinate_system_initialized = False
        
        # 坐标系偏移量（从无人机初始位置到红点的偏移）
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        
        # 无人机轨迹记录（不限制轨迹点数量）
        self.drone_trajectory = []
        
        # 无人机上一次显示的网格位置
        self.last_displayed_grid_position = None
        
        # 动物检测记录 - 用于存储每个方格检测到的野生动物信息
        # 格式: {(grid_col, grid_row): {'detection_time': timestamp, 'animals': {'种类1': 数量1, '种类2': 数量2, ...}}}
        self.wildlife_detections = {}
        
        # 视觉检测数据缓存 - 存储所有视觉识别的结果（每次一行：方格id+类别+数量）
        # 格式: [{'grid_id': 'B1 A1', 'animal_type': '猴子', 'count': 2}, {'grid_id': 'B1 A1', 'animal_type': '大象', 'count': 1}, ...]
        self.vision_detection_cache = []
        
        # 任务ID - 用于保存历史记录
        self.mission_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 历史记录目录
        self.history_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yesheng_log')
        
        # 确保历史记录目录存在
        if not os.path.exists(self.history_dir):
            os.makedirs(self.history_dir)
            rospy.loginfo(f"创建历史记录目录: {self.history_dir}")
        
        # 已加载的历史记录ID
        self.loaded_history_id = None
        
        # 历史记录列表
        self.history_records = self.load_history_list()
        
        # 历史面板是否显示
        self.history_panel_visible = False
        
        # 当前方格位置 - 用于判断是否进入新方格
        self.current_grid_position = None
        
        # 上次检测时间 - 避免同一方格短时间内重复检测
        self.last_detection_time = 0
        
        # 动物种类中英文映射
        self.animal_name_map = {
            'hou_zi': '猴子',
            'da_xiang': '大象',
            'kong_que': '孔雀',
            'lang': '狼',
            'lao_hu': '老虎',
            'lion': '狮子',
            'monkey': '猴子',
            'elephant': '大象',
            'peacock': '孔雀',
            'wolf': '狼',
            'tiger': '老虎'
        }

        # 连接信号和槽
        self.update_position_signal.connect(self._handle_position_update)
        self.update_status_signal.connect(self.statusBar().showMessage)
        self.update_wildlife_display_signal.connect(self._update_wildlife_display_safe)
        self.update_map_signal.connect(self._update_map_safe)
        
        # 创建订阅器 - 订阅无人机位置
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback, queue_size=10)
        
        # 创建订阅器 - 订阅动物ID和值 (之前是订阅'/region_wildlife_detection')
        rospy.Subscriber('/id_dongwu_value', String, self.detection_callback, queue_size=10)
        
        # 创建订阅器 - 订阅野生动物统计信息
        rospy.Subscriber('/wildlife_statistics', String, self.wildlife_statistics_callback, queue_size=10)
        
        # 定时器用于更新无人机位置显示（20Hz，提高更新频率确保橘色点跟上无人机位置）
        self.position_timer = QTimer()
        self.position_timer.timeout.connect(self.update_drone_display)
        self.position_timer.start(50)  # 20Hz (50ms)
        
        self.init_ui()

        # 测试坐标转换（可选，用于验证）
        self.test_coordinate_conversion()
        
        # 无人机坐标首次接收标记和时间
        self.first_odom_received = False
        self.first_odom_time = 0
        self.buttons_enabled = False  # 控制按钮是否启用
        
        # 任务开始时间和最短飞行时间设置
        self.mission_start_time = None
        self.min_flight_time = 60  # 设置最短飞行时间为10秒，用于判断是否回到原点
        
    def _handle_position_update(self, position):
        """处理位置更新的槽函数（运行在UI线程）"""
        self.drone_position = position
        
        # 检查是否需要初始化坐标系
        if self.drone_initial_position is None:
            # 记录无人机初始位置并初始化坐标系，使其对应红点位置(A9 B1)
            self.drone_initial_position = position[:]
            rospy.loginfo(f"记录无人机初始位置: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})")
            self.initialize_coordinate_system()
            rospy.loginfo(f"将无人机初始位置设为A9 B1点原点，坐标偏移: X={self.offset_x:.2f}, Y={self.offset_y:.2f}, Z={self.offset_z:.2f}")
        
        # 检测无人机是否回到原点（红点位置），只有在任务启动10秒后才开始判断
        if self.mission_active and not self.mission_completed:
            # 计算当前任务时间
            current_time = datetime.now()
            elapsed_time = (current_time - self.mission_start_time).total_seconds() if self.mission_start_time else 0
            
            # 只有在飞行超过最短时间后才检查是否回到原点
            if elapsed_time >= self.min_flight_time:
                # 计算无人机到原点的距离
                drone_x, drone_y, drone_z = position
                distance_to_origin = math.sqrt(drone_x**2 + drone_y**2)
                height_near_ground = abs(drone_z) < 0.1  # 高度接近地面
                
                # 检查无人机是否靠近原点并且已经接近地面（可能已着陆）
                if distance_to_origin < self.return_to_origin_threshold and height_near_ground:
                    if not self.drone_returned_to_origin:
                        self.drone_returned_to_origin = True
                        rospy.loginfo(f"无人机已回到原点，距离: {distance_to_origin:.2f}米，高度: {drone_z:.2f}米，飞额行时间: {elapsed_time:.2f}秒")
                        
                        # 任务完成，停止任务
                        self.mission_completed = True
                        self.stop_mission()
                        
                        # 避免短时间内重复保存
                        current_time = rospy.Time.now().to_sec()
                        if current_time - self.last_save_time > 5.0 and self.wildlife_detections:
                            self.save_wildlife_data()
                            self.last_save_time = current_time
                            self.statusBar().showMessage("任务完成，已自动保存野生动物数据", 5000)
                else:
                    # 如果无人机已经离开原点，重置标志，以便下次可以再次检测返回原点事件
                    self.drone_returned_to_origin = False
    
    def _update_wildlife_display_safe(self):
        """线程安全的更新野生动物显示（运行在UI线程）"""
        self.update_wildlife_display()
    
    def _update_map_safe(self):
        """线程安全的更新地图（运行在UI线程）"""
        self.map_widget.update()
    
    def odom_callback(self, msg):
        """处理接收到的里程计数据（运行在ROS回调线程）"""
        # 获取无人机位置（x, y, z）
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # 记录最后接收到odom消息的时间，用于在线状态检测
        current_time = rospy.Time.now().to_sec()
        self.last_odom_time = current_time
        self.drone_online = True
        
        # 记录首次接收到坐标的时间
        if not self.first_odom_received:
            self.first_odom_received = True
            self.first_odom_time = self.last_odom_time
            rospy.loginfo("首次接收到无人机坐标，将在3秒后启用控制按钮")
        
        # 直接更新位置，提高实时性
        self.drone_position = [x, y, z]
        
        # 同时通过信号发送位置更新以更新UI
        # 这样确保无人机位置能立即被更新，不会有延迟
        self.update_position_signal.emit([x, y, z])
        
    def detection_callback(self, msg):
        """处理动物检测结果回调（运行在ROS回调线程）"""
        try:
            # 如果正在查看历史记录，不处理新的检测结果
            if self.loaded_history_id is not None:
                return
                
            # 如果无人机处于返航航线上，不处理新的检测结果
            if self.is_returning:
                rospy.loginfo("无人机处于返航阶段，忽略动物检测结果")
                return
                
            # 获取当前时间
            current_time = rospy.Time.now().to_sec()
            
            # 解析检测消息（JSON格式：包含区域ID和检测结果）
            # 现在从 /id_dongwu_value 话题获取，这个话题只在无人机位于正确位置时才会发布
            detection_data = msg.data.strip()
            if detection_data and detection_data != "wu_jiance_mubiao":
                try:
                    # 解析JSON格式的消息
                    detection_info = json.loads(detection_data)
                    vision_region_id = detection_info.get('region_id', '')
                    detections_data = detection_info.get('detections', {})
                    
                    # 直接将视觉检测数据确认为有效数据
                    if vision_region_id and detections_data:
                        # 转换区域ID为网格坐标
                        grid_col, grid_row = self.coord_to_grid(vision_region_id)
                        current_grid = (grid_col, grid_row)
                        
                        # 检查该区域是否已经有检测记录
                        if current_grid not in self.wildlife_detections:
                            # 转换检测结果为动物字典格式
                            confirmed_detections = {}
                            for animal_type, count in detections_data.items():
                                # 将拼音转换为中文（如果映射中有的话）
                                animal_name = self.animal_name_map.get(animal_type, animal_type)
                                confirmed_detections[animal_name] = int(count)
                            
                            # 直接保存检测结果到wildlife_detections
                            self.wildlife_detections[current_grid] = {
                                'detection_time': current_time,
                                'animals': confirmed_detections,
                                'grid_coord': vision_region_id
                            }
                            
                            # 在状态栏显示信息（通过信号）
                            animal_info = ", ".join([f"{name}: {count}只" for name, count in confirmed_detections.items()])
                            status_msg = f"在{vision_region_id}检测到: {animal_info}"
                            self.update_status_signal.emit(status_msg, 5000)
                            
                            # 更新动物信息显示区域（通过信号）
                            self.update_wildlife_display_signal.emit()
                            
                            # 更新地图显示（通过信号）
                            self.update_map_signal.emit()
                            
                            rospy.loginfo(f"已确认区域 {vision_region_id} 的检测结果: {confirmed_detections}")
                        else:
                            # 如果该区域已有检测记录，累加动物数量
                            existing_animals = self.wildlife_detections[current_grid]['animals']
                            for animal_type, count in detections_data.items():
                                animal_name = self.animal_name_map.get(animal_type, animal_type)
                                if animal_name in existing_animals:
                                    existing_animals[animal_name] += int(count)
                                else:
                                    existing_animals[animal_name] = int(count)
                            
                            # 更新检测时间
                            self.wildlife_detections[current_grid]['detection_time'] = current_time
                            
                            # 在状态栏显示信息（通过信号）
                            animal_info = ", ".join([f"{name}: {count}只" for name, count in existing_animals.items()])
                            status_msg = f"在{vision_region_id}累加检测到: {animal_info}"
                            self.update_status_signal.emit(status_msg, 5000)
                            
                            # 更新动物信息显示区域（通过信号）
                            self.update_wildlife_display_signal.emit()
                            
                            # 更新地图显示（通过信号）
                            self.update_map_signal.emit()
                            
                            rospy.loginfo(f"已累加区域 {vision_region_id} 的检测结果: {existing_animals}")
                    
                except json.JSONDecodeError:
                    rospy.logwarn(f"无法解析检测数据JSON格式: {detection_data}")
            
        except Exception as e:
            rospy.logerr(f"处理检测数据出错: {str(e)}")
            
    def wildlife_statistics_callback(self, msg):
        """处理野生动物统计信息回调（来自视觉检测节点的统计结果）"""
        try:
            # 如果正在查看历史记录，不处理新的检测结果
            if self.loaded_history_id is not None:
                return
                
            # 如果无人机处于返航航线上，不处理新的检测结果
            if self.is_returning:
                rospy.loginfo("无人机处于返航阶段，忽略动物统计结果")
                return
            
            # 获取当前时间
            current_time = rospy.Time.now().to_sec()
            
            # 解析统计信息JSON
            stats_data = msg.data.strip()
            if stats_data:
                try:
                    stats_info = json.loads(stats_data)
                    region_id = stats_info.get('region_id', '')
                    animals = stats_info.get('animals', {})
                    
                    if region_id and animals:
                        # 转换区域ID为网格坐标
                        grid_col, grid_row = self.coord_to_grid(region_id)
                        current_grid = (grid_col, grid_row)
                        
                        # 检查该区域是否已经有检测记录，每个区域只统计一次
                        if current_grid not in self.wildlife_detections:
                            # 转换检测结果为动物字典格式
                            confirmed_detections = {}
                            for animal_type, count in animals.items():
                                # 将拼音转换为中文（如果映射中有的话）
                                animal_name = self.animal_name_map.get(animal_type, animal_type)
                                confirmed_detections[animal_name] = int(count)
                            
                            # 直接保存检测结果到wildlife_detections
                            self.wildlife_detections[current_grid] = {
                                'detection_time': current_time,
                                'animals': confirmed_detections,
                                'grid_coord': region_id
                            }
                            
                            # 在状态栏显示信息（通过信号）
                            animal_info = ", ".join([f"{name}: {count}只" for name, count in confirmed_detections.items()])
                            if animal_info:
                                status_msg = f"区域{region_id}统计结果: {animal_info}"
                                self.update_status_signal.emit(status_msg, 5000)
                            else:
                                status_msg = f"区域{region_id}未检测到动物"
                                self.update_status_signal.emit(status_msg, 3000)
                            
                            # 更新动物信息显示区域（通过信号）
                            self.update_wildlife_display_signal.emit()
                            
                            # 更新地图显示（通过信号）
                            self.update_map_signal.emit()
                            
                            rospy.loginfo(f"已统计区域 {region_id} 的检测结果: {confirmed_detections}")
                        else:
                            rospy.loginfo(f"区域 {region_id} 已有检测记录，不重复统计")
                    
                except json.JSONDecodeError:
                    rospy.logwarn(f"无法解析动物统计数据JSON格式: {stats_data}")
                
        except Exception as e:
            rospy.logerr(f"处理动物统计数据出错: {str(e)}")
    
    def initialize_coordinate_system(self):
        """初始化坐标系，使无人机初始位置对应于红点位置(A9 B1)"""
        if self.drone_initial_position is None:
            rospy.logwarn("无法初始化坐标系：无人机初始位置未知")
            return
            
        rospy.loginfo("初始化坐标系，使无人机当前位置对应于红点位置(A9 B1)")
        
        # 坐标系偏移量就是无人机初始位置的负值（这样无人机初始位置在转换后对应A9 B1）
        self.offset_x = -self.drone_initial_position[0]
        self.offset_y = -self.drone_initial_position[1] 
        self.offset_z = -self.drone_initial_position[2]
        
        rospy.loginfo(f"坐标系偏移量: X={self.offset_x:.2f}, Y={self.offset_y:.2f}, Z={self.offset_z:.2f}")
        
        # 标记坐标系已初始化
        self.coordinate_system_initialized = True
        
    def update_drone_display(self):
        """更新无人机位置显示"""
        # 计算无人机当前位置对应的网格坐标
        drone_x, drone_y, drone_z = self.drone_position
        grid_col, grid_row = self.global_to_grid_coords(drone_x, drone_y)
        
        # 显示无人机位置，换行显示xyz坐标，包含相对于初始位置的信息
        if self.coordinate_system_initialized:
            self.drone_position_label.setText(
                f"无人机位置:\nX={drone_x:.2f}m\nY={drone_y:.2f}m\nZ={drone_z:.2f}m\n" +
                f"(相对初始点:\nX={(drone_x+self.offset_x):.2f}m\nY={(drone_y+self.offset_y):.2f}m\nZ={(drone_z+self.offset_z):.2f}m)"
            )
        else:
            self.drone_position_label.setText(
                f"无人机位置:\nX={drone_x:.2f}m\nY={drone_y:.2f}m\nZ={drone_z:.2f}m"
            )
        
        # 计算并显示当前区域ID，同时发布给视觉检测节点
        current_region_id = None
        if isinstance(grid_col, (int, float)) and isinstance(grid_row, (int, float)):
            # 检查是否在有效网格范围内
            if 0 <= grid_col < self.grid_cols and 0 <= grid_row < self.grid_rows:
                # 使用position_to_coord方法计算区域ID
                current_region_id = self.position_to_coord(int(round(grid_col)), int(round(grid_row)))
                if current_region_id:
                    self.current_region_label.setText(f"当前区域: {current_region_id}")
                    
                    # 发布当前区域ID给视觉检测节点
                    region_msg = String()
                    region_msg.data = current_region_id
                    self.drone_area_pub.publish(region_msg)
                else:
                    self.current_region_label.setText("当前区域: 无效")
            else:
                self.current_region_label.setText("当前区域: --")
        else:
            self.current_region_label.setText("当前区域: --")
        
        # 始终添加新的轨迹点，确保轨迹准确跟踪无人机位置
        # 判断是否需要添加新轨迹点 - 优化逻辑，减少不必要的点
        add_trajectory_point = False
        
        # 如果轨迹为空，添加第一个点
        if not self.drone_trajectory:
            add_trajectory_point = True
        elif self.drone_trajectory:
            last_point = self.drone_trajectory[-1]
            # 计算上一个轨迹点对应的网格坐标
            last_grid_col, last_grid_row = self.global_to_grid_coords(last_point[0], last_point[1])
            
            # 计算与上一个点的距离
            dx = self.drone_position[0] - last_point[0]
            dy = self.drone_position[1] - last_point[1]
            dz = self.drone_position[2] - last_point[2]
            distance = (dx*dx + dy*dy + dz*dz)**0.5
            
            # 使用更小的阈值，确保轨迹更平滑 (5mm)
            # 或者如果网格坐标发生变化，也添加新点
            if distance > 0.005 or (grid_col != last_grid_col or grid_row != last_grid_row):
                add_trajectory_point = True
        
        # 添加新的轨迹点
        if add_trajectory_point:
            # 深拷贝以确保独立的轨迹点
            self.drone_trajectory.append(self.drone_position[:])
        
        # 更新地图显示
        self.map_widget.update()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("野生动物巡查系统地面站")
        # 调整窗口大小为1280*670，减小高度以适应屏幕上方任务栏
        self.setGeometry(100, 50, 1280, 670)

        # 创建菜单栏
        self.create_menu_bar()

        # 创建状态栏
        self.statusBar().showMessage("就绪")
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(2, 2, 2, 2)  # 减小外边距
        main_layout.setSpacing(3)  # 减小间距
        
        # 创建左侧布局（地图和控制面板）
        left_panel = QWidget()
        left_layout = QHBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建地图部件
        self.map_widget = self.create_map_widget()
        left_layout.addWidget(self.map_widget, 6)  # 减小地图比例
        
        # 创建控制面板
        control_panel = self.create_control_panel()
        left_layout.addWidget(control_panel, 2)  # 控制面板占比
        
        # 初始时禁用上传和开始按钮
        for widget in self.findChildren(QPushButton):
            if widget.text() in ["上传航线", "开始"]:
                widget.setEnabled(False)
                widget.setToolTip("等待无人机连接 (3秒)")
        
        # 添加左侧面板到主布局
        main_layout.addWidget(left_panel, 4)  # 左侧占主布局的4/6
        
        # 创建右侧边栏（包含动物检测信息）
        self.right_sidebar = QWidget()
        right_layout = QVBoxLayout(self.right_sidebar)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建动物检测信息显示区域
        wildlife_panel = self.create_wildlife_panel()
        right_layout.addWidget(wildlife_panel)
        
        # 将右侧布局添加到主布局
        main_layout.addWidget(self.right_sidebar, 2)  # 右侧占主布局的2/6，增加比例
        
        # 创建历史记录面板（初始隐藏）
        self.create_history_panel()

    def create_control_panel(self):
        """创建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(3)  # 更小的垂直间距
        layout.setContentsMargins(2, 2, 2, 2)  # 更小的边距
        
        # 状态显示 - 改为垂直布局，每个信息独占一行
        status_group = QGroupBox("系统状态")
        status_layout = QVBoxLayout()  # 使用垂直布局
        status_layout.setSpacing(2)  # 组件间距
        status_layout.setContentsMargins(4, 6, 4, 6)  # 内边距
        
        # 无人机在线状态显示
        self.drone_status_label = QLabel("● 无人机离线")
        self.drone_status_label.setStyleSheet("QLabel { color: red; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        self.drone_status_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.drone_status_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.drone_status_label)
        
        # 任务状态显示
        self.status_label = QLabel("状态: 待机")
        self.status_label.setStyleSheet("QLabel { font-size: 13px; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        self.status_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.status_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.status_label)
        
        # 航点数量
        self.waypoint_count_label = QLabel("路径长度: 0 格")
        self.waypoint_count_label.setStyleSheet("QLabel { font-family: 'Monospace', 'Courier New'; }")
        self.waypoint_count_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.waypoint_count_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.waypoint_count_label)
        
        # 任务计时标签
        self.timer_label = QLabel("任务时间: 0分0秒")
        self.timer_label.setStyleSheet("QLabel { color: #FF5722; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        self.timer_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.timer_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.timer_label)
        
        # 无人机位置显示 - 设置为多行显示
        self.drone_position_label = QLabel("无人机位置:\nX=0.00m\nY=0.00m\nZ=0.00m")
        self.drone_position_label.setStyleSheet("QLabel { color: #FF8C00; font-family: 'Monospace', 'Courier New'; }")
        self.drone_position_label.setAlignment(Qt.AlignLeft)  # 左对齐
        self.drone_position_label.setMinimumHeight(80)  # 增加高度以容纳多行
        status_layout.addWidget(self.drone_position_label)
        
        # 距离方格中心的距离显示
        self.distance_to_center_label = QLabel("距离方格中心: 0.00m")
        self.distance_to_center_label.setStyleSheet("QLabel { color: #4CAF50; font-family: 'Monospace', 'Courier New'; }")
        self.distance_to_center_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.distance_to_center_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.distance_to_center_label)
        
        # 当前区域ID显示
        self.current_region_label = QLabel("当前区域: --")
        self.current_region_label.setStyleSheet("QLabel { color: #2196F3; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        self.current_region_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.current_region_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.current_region_label)
        
        # 设置布局
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 禁区设置和路径规划合并到一行
        action_panel = QHBoxLayout()
        action_panel.setSpacing(2)
        
        # 左侧：禁区设置
        forbidden_group = QGroupBox("禁区设置")
        forbidden_group.setMaximumHeight(95)  # 限制高度
        forbidden_layout = QVBoxLayout(forbidden_group)
        forbidden_layout.setSpacing(2)
        forbidden_layout.setContentsMargins(4, 6, 4, 6)
        
        forbidden_status_layout = QHBoxLayout()
        forbidden_status_layout.addWidget(QLabel("已选禁区:"))
        self.forbidden_status_label = QLabel("0/3")
        forbidden_status_layout.addWidget(self.forbidden_status_label)
        
        forbidden_layout.addLayout(forbidden_status_layout)
        
        clear_forbidden_btn = QPushButton("清除禁区")
        clear_forbidden_btn.setMinimumHeight(36)  # 增加按钮高度
        clear_forbidden_btn.setStyleSheet("QPushButton { font-size: 12px; font-weight: bold; padding: 6px; }")  # 增加字体大小和内边距
        clear_forbidden_btn.clicked.connect(self.clear_forbidden_zones)
        forbidden_layout.addWidget(clear_forbidden_btn)
        
        action_panel.addWidget(forbidden_group)
        
        # 右侧：路径规划
        planning_group = QGroupBox("路径规划")
        planning_group.setMaximumHeight(95)  # 限制高度
        planning_layout = QVBoxLayout(planning_group)
        planning_layout.setSpacing(2)
        planning_layout.setContentsMargins(4, 6, 4, 6)
        
        plan_btn = QPushButton("规划路径")
        plan_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 6px; font-size: 12px; }")
        plan_btn.setMinimumHeight(36)  # 增加按钮高度
        plan_btn.clicked.connect(self.plan_path)
        planning_layout.addWidget(plan_btn)

        upload_btn = QPushButton("上传航线")
        upload_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 6px; font-size: 12px; }")
        upload_btn.setMinimumHeight(36)  # 增加按钮高度
        upload_btn.clicked.connect(self.upload_waypoints)
        planning_layout.addWidget(upload_btn)
        
        # 添加进度条
        self.upload_progress = QProgressBar()
        self.upload_progress.setVisible(False)  # 初始隐藏
        self.upload_progress.setMaximumHeight(8)  # 进一步减小高度
        self.upload_progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid grey;
                border-radius: 2px;
                text-align: center;
                font-weight: bold;
                font-size: 7px;
            }
            QProgressBar::chunk {
                background-color: #2196F3;
                border-radius: 1px;
            }
        """)
        planning_layout.addWidget(self.upload_progress)
        
        action_panel.addWidget(planning_group)
        
        layout.addLayout(action_panel)
        
        # 任务控制 - 使用水平布局
        mission_group = QGroupBox("任务控制")
        mission_group.setMaximumHeight(95)  # 增加一点高度
        mission_layout = QHBoxLayout(mission_group)
        mission_layout.setSpacing(4)
        mission_layout.setContentsMargins(4, 6, 4, 6)
        
        self.start_btn = QPushButton("开始")
        self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; font-size: 14px; }")
        self.start_btn.setMinimumHeight(42)  # 增加按钮高度
        self.start_btn.clicked.connect(self.toggle_mission)
        mission_layout.addWidget(self.start_btn, 2)  # 比例为2
        
        button_layout = QVBoxLayout()
        button_layout.setSpacing(2)
        
        self.reset_btn = QPushButton("重置")
        self.reset_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 6px; font-size: 12px; }")
        self.reset_btn.setMinimumHeight(36)  # 增加按钮高度
        self.reset_btn.clicked.connect(self.reset_mission)
        button_layout.addWidget(self.reset_btn)
        
        self.clear_trajectory_btn = QPushButton("清除轨迹")
        self.clear_trajectory_btn.setStyleSheet("QPushButton { background-color: #FF8C00; color: white; font-weight: bold; padding: 6px; font-size: 12px; }")
        self.clear_trajectory_btn.setMinimumHeight(36)  # 增加按钮高度
        self.clear_trajectory_btn.clicked.connect(self.clear_trajectory)
        button_layout.addWidget(self.clear_trajectory_btn)
        
        mission_layout.addLayout(button_layout, 1)  # 比例为1
        
        layout.addWidget(mission_group)
        
        return panel
    
    def clear_trajectory(self):
        """清除无人机轨迹"""
        self.drone_trajectory.clear()
        self.map_widget.update()
        QMessageBox.information(self, "提示", "无人机轨迹已清除")
        
    def create_map_widget(self):
        """创建地图显示部件"""
        return MapWidget(self)
        
    def coord_to_position(self, coord_str):
        """将坐标字符串转换为网格位置"""
        # 支持格式：B行 A列 (如 "B1 A9")
        parts = coord_str.strip().split()
        if len(parts) != 2:
            return None

        try:
            # 解析行部分 (B1-B7)
            row_part = parts[0].upper()
            if not row_part.startswith('B'):
                return None
            row_num = int(row_part[1:])
            if not (1 <= row_num <= 7):
                return None
            row = 7 - row_num  # B1对应索引6，B7对应索引0

            # 解析列部分 (A1-A9)
            col_part = parts[1].upper()
            if not col_part.startswith('A'):
                return None
            col_num = int(col_part[1:])
            if not (1 <= col_num <= 9):
                return None
            col = col_num - 1  # A1对应索引0，A9对应索引8

            return (col, row)

        except ValueError:
            return None
            
    def coord_to_grid(self, coord_str):
        """将区域ID（如"B3 A5"）转换为网格坐标(col, row)"""
        return self.coord_to_position(coord_str)
        
    def position_to_coord(self, col, row):
        """将网格位置转换为坐标字符串"""
        if 0 <= col < self.grid_cols and 0 <= row < self.grid_rows:
            # 行：B1-B7（索引6-0对应B1-B7）
            row_str = f"B{7 - row}"
            # 列：A1-A9（索引0-8对应A1-A9）
            col_str = f"A{col + 1}"
            return f"{row_str} {col_str}"  # 格式：B行 A列
        return None
        
    def grid_to_global_coords(self, col, row, waypoint_index=None, total_waypoints=None):
        """将网格坐标转换为以红点为原点的全局坐标系（单位：米）
        X轴：向前（B1到B7方向）
        Y轴：向左（A9到A1方向）
        每个方格50cm×50cm
        """
        # 计算相对于红点的偏移（以方格为单位）
        # X轴：row方向，B1(row=6)到B7(row=0)，所以X = origin_row - row
        # Y轴：col方向，A9(col=8)到A1(col=0)，所以Y = origin_col - col
        grid_offset_x = self.origin_row - row  # 向前为正
        grid_offset_y = self.origin_col - col  # 向左为正

        # 转换为实际距离（米）
        global_x = grid_offset_x * self.cell_size  # 50cm = 0.5米
        global_y = grid_offset_y * self.cell_size  # 50cm = 0.5米

        # 确定高度：起飞点高度为takeoff_height，降落点高度为landing_height，其他为survey_height
        if waypoint_index is not None and total_waypoints is not None:
            if waypoint_index == 0:  # 第一个航点（起飞点）
                # 如果已经记录了无人机初始位置，使用原地起飞，否则使用传统起降点
                if self.coordinate_system_initialized:
                    # 第一个航点是原地起飞，位置与无人机初始位置相同，但高度为起飞高度
                    return -self.offset_x, -self.offset_y, self.takeoff_height
                else:
                    global_z = self.takeoff_height  # 使用起飞高度
            elif waypoint_index == total_waypoints - 1:  # 最后一个航点（降落点）
                global_z = self.landing_height  # 使用降落高度
            else:
                global_z = self.survey_height  # 巡查高度
        else:
            global_z = 0  # 默认高度
        
        # 如果坐标系已初始化，考虑坐标偏移量
        if self.coordinate_system_initialized and waypoint_index is not None:
            # 对于第一个航点，我们已经在前面特殊处理了
            if waypoint_index != 0:
                # 应用坐标系偏移（将原点从红点移动到无人机初始位置）
                global_x = global_x - self.offset_x
                global_y = global_y - self.offset_y
                # Z轴偏移在前面处理高度时已考虑

        return global_x, global_y, global_z

    def global_to_grid_coords(self, global_x, global_y):
        """将全局坐标（米）转换回网格坐标"""
        # 如果坐标系已初始化，先应用偏移量还原到红点为原点的坐标系
        if self.coordinate_system_initialized:
            # 还原到红点为原点的坐标系
            adjusted_x = global_x + self.offset_x
            adjusted_y = global_y + self.offset_y
        else:
            adjusted_x = global_x
            adjusted_y = global_y
            
        # 处理特殊情况：原点(0,0)应该精确对应红点位置
        if adjusted_x == 0.0 and adjusted_y == 0.0:
            return self.red_point
            
        # 先转换为方格偏移
        grid_offset_x = adjusted_x / self.cell_size
        grid_offset_y = adjusted_y / self.cell_size

        # 再转换为网格坐标
        col = self.origin_col - grid_offset_y
        row = self.origin_row - grid_offset_x
        
        # 计算精确的网格坐标（包括小数部分）
        exact_col = col
        exact_row = row
        
        # 计算到最近网格中心的距离
        col_center = round(col)
        row_center = round(row)
        
        # 计算中心点对应的全局坐标
        center_global_x = (self.origin_row - row_center) * self.cell_size
        center_global_y = (self.origin_col - col_center) * self.cell_size
        
        if self.coordinate_system_initialized:
            # 应用坐标偏移
            center_global_x = center_global_x - self.offset_x
            center_global_y = center_global_y - self.offset_y
        
        # 计算距离
        distance_to_center = ((global_x - center_global_x) ** 2 + (global_y - center_global_y) ** 2) ** 0.5
        
        # 保存距离和准确坐标供使用
        self.last_distance_to_center = distance_to_center
        self.exact_grid_coords = (exact_col, exact_row)
        
        # 返回最近的网格坐标（四舍五入）
        return int(round(col)), int(round(row))

    def clear_forbidden_zones(self):
        """清除所有禁区"""
        self.forbidden_zones.clear()
        self.update_forbidden_status()
        self.map_widget.update()
        
    def update_forbidden_status(self):
        """更新禁区状态显示"""
        count = len(self.forbidden_zones)
        self.forbidden_status_label.setText(f"已选择: {count}/3")
        
    def add_forbidden_zone(self, col, row):
        """添加禁区"""
        if len(self.forbidden_zones) >= 3:
            QMessageBox.warning(self, "警告", "最多只能设置3个禁区")
            return False
        
        # 检查是否已经是禁区
        if (col, row) in self.forbidden_zones:
            return False
        
        # 检查是否是红点位置
        if (col, row) == self.red_point:
            QMessageBox.warning(self, "警告", "不能在起降点设置禁区")
            return False
        
        # 添加禁区
        self.forbidden_zones.append((col, row))
        self.update_forbidden_status()
        return True
        
    def find_detour_to_landing_point_with_repeats(self, start, target):
        """寻找从起点到目标点的绕行路径，允许重复航点"""
        # 首先尝试原有的绕行策略
        detour_path = self.find_detour_to_landing_point(start, target)
        if detour_path:
            return detour_path
        
        # 如果原有策略失败，允许通过已访问的航点进行绕行
        for waypoint in self.waypoints:
            if waypoint != start and waypoint not in self.forbidden_zones:
                # 检查从起点到已访问航点，再到目标点的路径是否安全
                if (not self.path_crosses_forbidden_zone(start, waypoint) and
                    not self.path_crosses_forbidden_zone(waypoint, target)):
                    return [waypoint, target]
        
        # 如果仍然失败，尝试通过多个已访问航点的组合
        for i, waypoint1 in enumerate(self.waypoints):
            if waypoint1 != start and waypoint1 not in self.forbidden_zones:
                for j, waypoint2 in enumerate(self.waypoints):
                    if (waypoint2 != start and waypoint2 != waypoint1 and 
                        waypoint2 not in self.forbidden_zones):
                        # 检查三段路径是否都安全
                        if (not self.path_crosses_forbidden_zone(start, waypoint1) and
                            not self.path_crosses_forbidden_zone(waypoint1, waypoint2) and
                            not self.path_crosses_forbidden_zone(waypoint2, target)):
                            return [waypoint1, waypoint2, target]
        
        return None
            
        if (col, row) == self.red_point:
            QMessageBox.warning(self, "警告", "不能将红点设为禁区")
            return False
            
        if (col, row) in self.forbidden_zones:
            QMessageBox.warning(self, "警告", "该位置已是禁区")
            return False
            
        self.forbidden_zones.append((col, row))
        self.update_forbidden_status()
        self.map_widget.update()
        return True
        
    def plan_path(self):
        """规划路径 - 混合算法：相邻移动巡查，沿边缘返回"""
        if len(self.forbidden_zones) < 3:
            QMessageBox.warning(self, "警告", "请先选择3个禁区！")
            return

        # 清空之前的航点和返回路径索引
        self.waypoints = []
        self.return_path_start_index = -1

        # 生成所有需要访问的点（除了禁区）
        all_points = []
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                point = (col, row)
                if point not in self.forbidden_zones:
                    all_points.append(point)

        # 确保起降点在可访问点列表中
        if self.red_point not in all_points:
            QMessageBox.warning(self, "警告", "起降点位置被禁区阻挡，无法规划路径！")
            return

        # 从起降点开始
        self.waypoints.append(self.red_point)

        # 从待访问点中移除起始点
        remaining_points = [p for p in all_points if p != self.red_point]
        
        # 第一阶段：使用严格相邻移动进行巡查
        self.plan_adjacent_survey(remaining_points.copy())

        # 计算巡查阶段未能访问的点
        visited_in_survey = set(self.waypoints)
        unvisited_points = [p for p in all_points if p not in visited_in_survey]

        # 第二阶段：返回路径中遍历剩余点（允许灵活移动）
        if not self.add_flexible_return_path(unvisited_points):
            QMessageBox.warning(self, "警告", "无法规划返回路径！")
            return

        # 更新显示
        total_distance = self.calculate_total_distance()
        self.waypoint_count_label.setText(f"路径长度: {len(self.waypoints)} 格，总距离: {total_distance:.1f} 米")
        self.map_widget.update()

        # 验证是否遍历了所有非禁区点
        visited_points = set(self.waypoints)
        expected_points = set(all_points)
        is_closed_loop = (len(self.waypoints) >= 2 and
                         self.waypoints[0] == self.waypoints[-1] == self.red_point)

        if visited_points == expected_points and is_closed_loop:
            survey_points = len([p for p in self.waypoints[:self.return_path_start_index+1] if p in all_points])
            return_points = len(self.waypoints) - self.return_path_start_index - 1
            used_fallback = self.check_fallback_usage()

            fallback_info = "（含相邻容错）" if used_fallback else "（灵活移动）"

            QMessageBox.information(self, "成功",
                f"混合模式路径规划完成！\n"
                f"• 总航点：{len(self.waypoints)}个\n"
                f"• 巡查航点：{survey_points}个（严格相邻）\n"
                f"• 返回航点：{return_points}个{fallback_info}\n"
                f"• 遍历方块：{len(all_points)}个（50cm×50cm）\n"
                f"• 总距离：{total_distance:.1f}米\n"
                f"• 已形成闭环")
        else:
            missing_points = expected_points - visited_points
            if missing_points:
                QMessageBox.warning(self, "警告", f"路径规划不完整，遗漏了{len(missing_points)}个方块！")
            elif not is_closed_loop:
                QMessageBox.warning(self, "警告", "路径未形成闭环！")
        
    def find_optimal_start_point(self, all_points):
        """寻找最优起始点，强制从B1 A9位置开始（与降落点相同）"""
        # 强制起始点为B1 A9位置，与降落点相同 (A9列B1行，对应坐标(8, 6))
        start_point = self.red_point  # 使用与降落点相同的位置 B1 A9

        # 检查B1 A9位置是否在可访问点列表中
        if start_point in all_points:
            return start_point
        else:
            # 如果B1 A9位置不可访问（比如是禁区），返回None
            return None

    def plan_adjacent_survey(self, remaining_points):
        """第一阶段：使用严格相邻移动进行巡查"""
        unvisited = set(remaining_points)

        while unvisited:
            current_pos = self.waypoints[-1]

            # 寻找相邻的未访问点
            adjacent_point = self.find_adjacent_unvisited(current_pos, unvisited)

            if adjacent_point:
                # 直接移动到相邻点
                self.waypoints.append(adjacent_point)
                unvisited.remove(adjacent_point)
            else:
                # 寻找通过相邻移动能到达的最近未访问点
                path_to_unvisited = self.find_adjacent_path_to_unvisited(current_pos, unvisited)
                if path_to_unvisited:
                    # 添加路径上的所有点
                    for point in path_to_unvisited:
                        self.waypoints.append(point)
                        if point in unvisited:
                            unvisited.remove(point)
                else:
                    # 无法通过相邻移动到达任何未访问点，结束巡查阶段
                    break

        return len(unvisited) == 0

    def find_adjacent_unvisited(self, current_pos, unvisited):
        """寻找相邻的未访问点，优先选择与当前移动方向一致的点以减少拐弯"""
        # 获取当前移动方向
        current_direction = self.get_current_direction()
        
        # 定义方向优先级：优先选择与当前方向一致的方向，然后是直角方向，最后是反方向
        direction_priorities = self.get_direction_priorities(current_direction)
        
        # 按优先级顺序检查相邻点
        for dx, dy in direction_priorities:
            next_col = current_pos[0] + dx
            next_row = current_pos[1] + dy
            next_point = (next_col, next_row)

            if (0 <= next_col < self.grid_cols and
                0 <= next_row < self.grid_rows and
                next_point in unvisited and
                next_point not in self.forbidden_zones):
                return next_point

        return None
        
    def get_direction_to_point(self, from_point, to_point):
        """获取从一个点指向另一个点的大致方向"""
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        
        # 简化为八个基本方向
        if abs(dx) > abs(dy):
            # 水平方向为主
            return (1 if dx > 0 else -1, 0)
        elif abs(dy) > abs(dx):
            # 垂直方向为主
            return (0, 1 if dy > 0 else -1)
        else:
            # 对角线方向
            return (1 if dx > 0 else -1, 1 if dy > 0 else -1)
            
    def reorder_priorities_away_from_landing(self, priorities, landing_direction):
        """重新排序方向优先级，尽量避开朝向起降点的方向"""
        # 找出最不希望走的方向（朝向起降点）
        opposite_landing = (-landing_direction[0], -landing_direction[1])
        
        # 将landing_direction移到最后（最低优先级）
        new_priorities = [d for d in priorities if d != landing_direction]
        if landing_direction in priorities:
            new_priorities.append(landing_direction)
            
        # 如果有相反方向，提高其优先级（远离起降点）
        if opposite_landing in new_priorities:
            new_priorities.remove(opposite_landing)
            new_priorities.insert(0, opposite_landing)
            
        return new_priorities

    def get_current_direction(self):
        """获取当前移动方向，用于优化路径规划减少拐弯"""
        if len(self.waypoints) < 2:
            return None  # 没有足够的航点来确定方向
        
        # 计算最后两个航点之间的方向向量
        last_point = self.waypoints[-1]
        second_last_point = self.waypoints[-2]
        
        dx = last_point[0] - second_last_point[0]
        dy = last_point[1] - second_last_point[1]
        
        return (dx, dy)
    
    def get_direction_priorities(self, current_direction):
        """根据当前移动方向获取方向优先级列表，减少连续拐弯"""
        # 定义所有可能的移动方向：上、右、下、左
        all_directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        if current_direction is None:
            # 如果没有当前方向，使用默认顺序（优先向右和向上，便于形成规律路径）
            return [(1, 0), (0, 1), (0, -1), (-1, 0)]  # 右、上、下、左
        
        # 将当前方向放在最前面（继续直行）
        priorities = [current_direction]
        
        # 添加垂直方向（90度转弯）
        perpendicular_directions = []
        if current_direction == (0, 1):  # 当前向上
            perpendicular_directions = [(1, 0), (-1, 0)]  # 右、左
        elif current_direction == (1, 0):  # 当前向右
            perpendicular_directions = [(0, 1), (0, -1)]  # 上、下
        elif current_direction == (0, -1):  # 当前向下
            perpendicular_directions = [(1, 0), (-1, 0)]  # 右、左
        elif current_direction == (-1, 0):  # 当前向左
            perpendicular_directions = [(0, 1), (0, -1)]  # 上、下
        
        priorities.extend(perpendicular_directions)
        
        # 最后添加反方向（180度转弯，最不优先）
        opposite_direction = (-current_direction[0], -current_direction[1])
        priorities.append(opposite_direction)
        
        # 确保所有方向都包含在内，去除重复
        final_priorities = []
        for direction in priorities:
            if direction in all_directions and direction not in final_priorities:
                final_priorities.append(direction)
        
        # 添加任何遗漏的方向
        for direction in all_directions:
            if direction not in final_priorities:
                final_priorities.append(direction)
        
        return final_priorities

    def count_turns_in_path(self, path):
        """计算路径中的转弯次数"""
        if len(path) < 3:
            return 0

        turn_count = 0
        prev_direction = None

        for i in range(len(path) - 1):
            current_point = path[i]
            next_point = path[i + 1]

            dx = next_point[0] - current_point[0]
            dy = next_point[1] - current_point[1]

            # 确定当前移动方向
            if dy == 1:
                current_direction = 0  # 上
            elif dx == 1:
                current_direction = 1  # 右
            elif dy == -1:
                current_direction = 2  # 下
            elif dx == -1:
                current_direction = 3  # 左
            else:
                continue  # 无效移动，跳过

            # 检查是否转弯
            if prev_direction is not None and prev_direction != current_direction:
                turn_count += 1

            prev_direction = current_direction

        return turn_count

    def find_adjacent_path_to_unvisited(self, start, unvisited):
        """使用优化的BFS寻找通过相邻移动到达未访问点的路径，优先选择拐弯次数少的路径"""
        if not unvisited:
            return None
        
        # 为每个未访问点计算路径，并按拐弯次数和距离排序
        target_paths = []
        
        for target in unvisited:
            path = self.find_adjacent_path(start, target)
            if path:
                # 计算路径的拐弯次数
                turns = self.count_turns_in_path([start] + path)
                # 计算曼哈顿距离作为次要排序条件
                distance = abs(target[0] - start[0]) + abs(target[1] - start[1])
                target_paths.append((turns, distance, len(path), target, path))
        
        if not target_paths:
            return None
        
        # 按拐弯次数、距离、路径长度排序，优先选择拐弯少的路径
        target_paths.sort(key=lambda x: (x[0], x[1], x[2]))
        
        # 返回最优路径
        return target_paths[0][4]  # 返回path

    def find_adjacent_path(self, start, end):
        """使用优化的BFS寻找两点间的相邻移动路径，优先选择拐弯次数较少的路径"""
        import heapq

        if start == end:
            return [end]

        # 使用优先队列，优先级为：(拐弯次数, 路径长度, 当前位置, 路径, 上一个方向)
        queue = [(0, 0, start, [start], None)]
        visited = {start: (0, 0)}  # 位置 -> (最少拐弯次数, 最短路径长度)

        while queue:
            turns, length, current, path, last_direction = heapq.heappop(queue)

            # 如果到达目标点
            if current == end:
                return path[1:]  # 不包括起始点

            # 获取当前移动方向的优先级
            current_direction = self.get_current_direction() if len(self.waypoints) >= 2 else None
            if last_direction is not None:
                current_direction = last_direction
            
            direction_priorities = self.get_direction_priorities(current_direction)

            # 检查所有相邻方向，按优先级顺序
            for dx, dy in direction_priorities:
                next_col = current[0] + dx
                next_row = current[1] + dy
                next_point = (next_col, next_row)

                if (0 <= next_col < self.grid_cols and
                    0 <= next_row < self.grid_rows and
                    next_point not in self.forbidden_zones):

                    # 计算新的拐弯次数
                    new_turns = turns
                    if last_direction is not None and (dx, dy) != last_direction:
                        new_turns += 1
                    
                    new_length = length + 1
                    new_path = path + [next_point]

                    # 检查是否应该访问这个点
                    should_visit = True
                    if next_point in visited:
                        prev_turns, prev_length = visited[next_point]
                        # 只有在拐弯次数更少，或拐弯次数相同但路径更短时才访问
                        if new_turns > prev_turns or (new_turns == prev_turns and new_length >= prev_length):
                            should_visit = False

                    if should_visit:
                        visited[next_point] = (new_turns, new_length)
                        heapq.heappush(queue, (new_turns, new_length, next_point, new_path, (dx, dy)))

        return None

    def is_on_edge(self, pos):
        """检查位置是否在网格边缘"""
        col, row = pos
        return (col == 0 or col == self.grid_cols - 1 or
                row == 0 or row == self.grid_rows - 1)

    def find_nearest_edge_point(self, start_pos):
        """寻找最近的边缘点"""
        col, row = start_pos
        edge_points = []

        # 添加四个边缘的候选点
        # 上边缘
        if row > 0:
            edge_points.append((col, 0))
        # 下边缘
        if row < self.grid_rows - 1:
            edge_points.append((col, self.grid_rows - 1))
        # 左边缘
        if col > 0:
            edge_points.append((0, row))
        # 右边缘
        if col < self.grid_cols - 1:
            edge_points.append((self.grid_cols - 1, row))

        # 过滤掉禁区点，找到最近的可达边缘点
        valid_edge_points = [p for p in edge_points if p not in self.forbidden_zones]

        if not valid_edge_points:
            return None

        # 返回距离最近的边缘点
        return min(valid_edge_points,
                  key=lambda p: abs(p[0] - col) + abs(p[1] - row))

    def move_to_nearest_edge(self, start_pos):
        """移动到最近的边缘"""
        # 如果已经在边缘，直接返回
        if self.is_on_edge(start_pos):
            return []

        # 寻找最近的边缘点
        nearest_edge_point = self.find_nearest_edge_point(start_pos)
        if nearest_edge_point:
            return self.get_shortest_path_between(start_pos, nearest_edge_point)

        return None

    def find_next_edge_unvisited(self, current, remaining):
        """寻找沿边缘可到达的下一个未访问点"""
        if not remaining:
            return None

        # 按距离排序未访问点
        sorted_remaining = sorted(remaining,
                                 key=lambda p: abs(p[0] - current[0]) + abs(p[1] - current[1]))

        for target in sorted_remaining:
            # 检查是否可以沿边缘到达
            if self.can_reach_via_edge(current, target):
                return target

        return None

    def can_reach_via_edge(self, start, target):
        """检查是否可以沿边缘到达目标点"""
        # 简单实现：检查是否可以通过边缘路径到达
        edge_path = self.find_edge_path(start, target)
        return edge_path is not None

    def find_edge_path(self, start, end):
        """寻找沿边缘的路径"""
        # 如果起点或终点不在边缘，先移动到边缘
        path = []
        current = start

        # 确保起点在边缘
        if not self.is_on_edge(current):
            to_edge = self.move_to_nearest_edge(current)
            if to_edge:
                path.extend(to_edge)
                current = to_edge[-1]
            else:
                return None

        # 沿边缘移动到目标
        edge_segment = self.find_edge_path_direct(current, end)
        if edge_segment:
            path.extend(edge_segment)

        return path if path else None

    def find_edge_path_direct(self, start, end):
        """直接沿边缘寻找路径"""
        from collections import deque

        queue = deque([(start, [start])])
        visited = {start}

        while queue:
            current, path = queue.popleft()

            if current == end:
                return path[1:]  # 不包括起始点

            # 检查相邻的边缘点
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                next_col = current[0] + dx
                next_row = current[1] + dy
                next_point = (next_col, next_row)

                if (0 <= next_col < self.grid_cols and
                    0 <= next_row < self.grid_rows and
                    next_point not in visited and
                    next_point not in self.forbidden_zones and
                    self.is_on_edge(next_point)):  # 必须在边缘

                    visited.add(next_point)
                    queue.append((next_point, path + [next_point]))

        return None

    def find_nearest_edge_to_landing(self):
        """寻找距离起降点最近的边缘点"""
        edge_points = []

        # 收集所有边缘点
        for col in range(self.grid_cols):
            for row in range(self.grid_rows):
                if self.is_on_edge((col, row)) and (col, row) not in self.forbidden_zones:
                    edge_points.append((col, row))

        if not edge_points:
            return None

        # 返回距离起降点最近的边缘点
        return min(edge_points,
                  key=lambda p: abs(p[0] - self.red_point[0]) + abs(p[1] - self.red_point[1]))

    def find_edge_path_to_landing(self, start):
        """沿边缘返回起降点"""
        if start == self.red_point:
            return [self.red_point]

        # 检查1.5米优化
        if self.is_close_and_safe_to_landing(start):
            return [self.red_point]

        # 沿边缘寻找到起降点的路径
        edge_path = self.find_edge_path_direct(start, self.red_point)
        if edge_path:
            return edge_path + [self.red_point]

        # 如果沿边缘无法直接到达，先到最近的边缘点再到起降点
        nearest_to_landing = self.find_nearest_edge_to_landing()
        if nearest_to_landing and nearest_to_landing != start:
            to_nearest = self.find_edge_path_direct(start, nearest_to_landing)
            from_nearest = self.get_shortest_path_between(nearest_to_landing, self.red_point)
            if to_nearest and from_nearest:
                return to_nearest + from_nearest

        return None
        
    def find_edge_return_path(self, start_pos, unvisited_points):
        """寻找沿边缘遍历剩余点并返回起降点的路径"""
        path = []
        current = start_pos
        remaining = unvisited_points.copy()

        # 首先移动到最近的边缘
        edge_entry_path = self.move_to_nearest_edge(current)
        if edge_entry_path:
            path.extend(edge_entry_path)
            current = edge_entry_path[-1]
            # 移除路径中访问的未访问点
            for point in edge_entry_path:
                if point in remaining:
                    remaining.remove(point)

        # 沿边缘遍历剩余的未访问点
        while remaining:
            # 检查1.5米优化
            if self.is_close_and_safe_to_landing(current):
                print(f"优化：沿边缘返航时，剩余{len(remaining)}个点，但已接近起降点，直接返航")
                break

            # 寻找沿边缘到达的下一个未访问点
            next_edge_point = self.find_next_edge_unvisited(current, remaining)
            if next_edge_point:
                edge_segment = self.find_edge_path(current, next_edge_point)
                if edge_segment:
                    path.extend(edge_segment)
                    current = edge_segment[-1]
                    # 移除路径中访问的未访问点
                    for point in edge_segment:
                        if point in remaining:
                            remaining.remove(point)
                else:
                    # 无法沿边缘到达，移除该点
                    remaining.remove(next_edge_point)
            else:
                # 没有更多可沿边缘到达的点
                break

        # 沿边缘返回起降点
        edge_return_segment = self.find_edge_path_to_landing(current)
        if edge_return_segment:
            path.extend(edge_return_segment)

        return path if path else None

    def is_close_and_safe_to_landing(self, current_pos):
        """检查是否距离起降点1.5米内且直线路径安全"""
        # 计算到起降点的实际距离（每个方块50cm = 0.5米）
        grid_distance = abs(current_pos[0] - self.red_point[0]) + abs(current_pos[1] - self.red_point[1])
        distance_meters = grid_distance * self.cell_size  # 转换为实际米数

        # 检查是否在1.5米范围内
        if distance_meters <= 1.5:
            # 检查直线路径是否安全（不经过禁飞区）
            if not self.path_crosses_forbidden_zone(current_pos, self.red_point):
                print(f"优化：距离起降点{distance_meters:.1f}米，直线安全，直接返航")
                return True

        return False

    def add_edge_return_to_landing(self, current_pos):
        """沿边缘返回起降点"""
        if current_pos == self.red_point:
            return True

        edge_return_path = self.find_edge_path_to_landing(current_pos)
        if edge_return_path:
            for point in edge_return_path:
                self.waypoints.append(point)
            return True

        # 如果沿边缘失败，使用直接路径
        print("警告：无法沿边缘返回起降点，使用直接路径")
        return self.add_shortest_return_to_landing(current_pos)

    def add_flexible_fallback_return(self, start_pos, unvisited_points):
        """灵活移动的备选返回方案"""
        current_pos = start_pos
        remaining_unvisited = unvisited_points.copy()

        while remaining_unvisited:
            if self.is_close_and_safe_to_landing(current_pos):
                break

            nearest_point = min(remaining_unvisited,
                               key=lambda p: abs(p[0] - current_pos[0]) + abs(p[1] - current_pos[1]))

            path_to_point = self.get_shortest_path_between(current_pos, nearest_point)
            if path_to_point:
                for point in path_to_point:
                    self.waypoints.append(point)
                    if point in remaining_unvisited:
                        remaining_unvisited.remove(point)
                current_pos = path_to_point[-1]
            else:
                remaining_unvisited.remove(nearest_point)

        return self.add_shortest_return_to_landing(current_pos)
        
    def add_flexible_return_path(self, unvisited_points):
        """规划返航路径：直接最短路径到起降点，但最后4个点必须沿边缘"""
        if not self.waypoints:
            return False

        # 记录返回路径的起始索引
        self.return_path_start_index = len(self.waypoints) - 1
        
        current_pos = self.waypoints[-1]
        remaining_unvisited = unvisited_points.copy()

        # 如果有未访问点，先处理这些点
        if remaining_unvisited:
            # 优先使用最短路径收集剩余的未访问点
            while remaining_unvisited:
                # 找出距离当前位置最近的未访问点
                nearest_point = min(remaining_unvisited, 
                                  key=lambda p: self.manhattan_distance(current_pos, p))
                
                # 使用A*算法寻找最短路径
                path_to_point = self.find_astar_path(current_pos, nearest_point, allow_diagonal=True)
                if path_to_point:
                    for point in path_to_point:
                        self.waypoints.append(point)
                        if point in remaining_unvisited:
                            remaining_unvisited.remove(point)
                    current_pos = nearest_point
                else:
                    # 无法到达该点，从列表中移除
                    remaining_unvisited.remove(nearest_point)

        # 从当前位置规划到起降点的路径
        current_pos = self.waypoints[-1]
        
        # 如果已经在起降点，直接返回
        if current_pos == self.red_point:
            return True
            
        # 计算到起降点的距离
        distance_to_landing = self.manhattan_distance(current_pos, self.red_point)
        
        # 如果距离小于等于4格，直接使用边缘路径返回
        if distance_to_landing <= 4:
            return self.add_edge_return_to_landing(current_pos)
        
        # 尝试找到最短的返航路径
        # 1. 计算直接到起降点的最短路径
        direct_path = self.find_astar_path(current_pos, self.red_point, allow_diagonal=True)
        if not direct_path:
            # 如果无法直接到达，尝试沿边缘返回
            return self.add_edge_return_to_landing(current_pos)
            
        # 2. 找出离起降点4格距离内的所有边缘点
        edge_points = []
        for col in range(self.grid_cols):
            for row in range(self.grid_rows):
                if (self.is_on_edge((col, row)) and 
                    (col, row) not in self.forbidden_zones and
                    self.manhattan_distance((col, row), self.red_point) <= 4):
                    edge_points.append((col, row))
        
        if not edge_points:
            # 如果找不到合适的边缘点，直接使用直接路径返回
            for point in direct_path:
                self.waypoints.append(point)
            return True
                
        # 3. 计算从当前位置到每个边缘点的最短路径
        best_path = None
        min_total_distance = float('inf')
        
        for edge_point in edge_points:
            # 计算到边缘点的路径
            path_to_edge = self.find_astar_path(current_pos, edge_point, allow_diagonal=True)
            if not path_to_edge:
                continue
                
            # 计算从边缘点到起降点的路径
            edge_to_landing_path = self.find_edge_path_to_landing(edge_point)
            if not edge_to_landing_path:
                continue
                
            # 计算总路径长度
            total_path = path_to_edge + edge_to_landing_path
            total_distance = len(total_path)
            
            # 更新最短路径
            if total_distance < min_total_distance:
                min_total_distance = total_distance
                best_path = (path_to_edge, edge_to_landing_path)
        
        # 4. 比较直接路径和边缘路径的长度
        if best_path and min_total_distance <= len(direct_path) + 2:  # 允许边缘路径稍微长一点
            # 使用边缘路径
            path_to_edge, edge_to_landing_path = best_path
            for point in path_to_edge:
                self.waypoints.append(point)
            for point in edge_to_landing_path:
                self.waypoints.append(point)
        else:
            # 使用直接路径，但确保最后4个点沿边缘
            if len(direct_path) <= 4:
                # 如果路径本身就很短，直接使用边缘路径
                return self.add_edge_return_to_landing(current_pos)
            else:
                # 取路径的前部分，然后添加边缘路径
                transition_point = direct_path[len(direct_path) - 5]
                for i in range(len(direct_path) - 4):
                    self.waypoints.append(direct_path[i])
                return self.add_edge_return_to_landing(transition_point)
                
        return True
        
    def find_safe_path(self, start, end):
        """寻找安全路径，优先避开禁飞区及其顶角"""
        # 首先尝试使用普通的灵活路径
        path = self.find_flexible_path(start, end)
        if path:
            # 检查路径是否安全
            for i in range(len(path) - 1):
                if i > 0:  # 跳过起点
                    p1 = path[i]
                    p2 = path[i + 1]
                    
                    # 检查路径段是否经过禁飞区顶角
                    for forbidden_zone in self.forbidden_zones:
                        corners = [
                            (forbidden_zone[0], forbidden_zone[1]),             # 左上
                            (forbidden_zone[0] + 1, forbidden_zone[1]),         # 右上
                            (forbidden_zone[0], forbidden_zone[1] + 1),         # 左下
                            (forbidden_zone[0] + 1, forbidden_zone[1] + 1)      # 右下
                        ]
                        
                        # 检查路径是否靠近禁飞区顶角
                        for corner in corners:
                            # 对角线距离检查
                            for t in range(10):  # 将路径分成10段检查
                                t_val = t / 10.0
                                point_x = p1[0] + t_val * (p2[0] - p1[0])
                                point_y = p1[1] + t_val * (p2[1] - p1[1])
                                
                                # 如果点到顶角距离小于安全距离，则认为不安全
                                dist = ((point_x - corner[0]) ** 2 + (point_y - corner[1]) ** 2) ** 0.5
                                if dist < 0.4:  # 比line_intersects_square中的检测更严格
                                    # 不安全，尝试避开顶角的路径
                                    return self.find_detour_path(start, end)
            
            # 路径是安全的
            return path
        
        # 如果没找到路径，尝试相邻移动路径
        return self.find_adjacent_path(start, end)
    
    def find_detour_path(self, start, end):
        """尝试找一条避开禁飞区顶角的绕行路径"""
        # 获取所有禁飞区顶角点并扩大安全范围
        corner_safety_zones = []
        for forbidden_zone in self.forbidden_zones:
            corners = [
                (forbidden_zone[0], forbidden_zone[1]),             # 左上
                (forbidden_zone[0] + 1, forbidden_zone[1]),         # 右上
                (forbidden_zone[0], forbidden_zone[1] + 1),         # 左下
                (forbidden_zone[0] + 1, forbidden_zone[1] + 1)      # 右下
            ]
            for corner in corners:
                corner_safety_zones.append(corner)
        
        # 使用更严格的A*搜索，避开禁飞区顶角
        import heapq
        
        open_set = [(0, start, [])]
        closed_set = set()
        
        while open_set:
            f_score, current, path = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
                
            closed_set.add(current)
            
            if current == end:
                return path + [end]
                
            # 检查四个方向的相邻点（不使用对角线移动，更安全）
            directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 上右下左
            
            for dx, dy in directions:
                next_col = current[0] + dx
                next_row = current[1] + dy
                next_point = (next_col, next_row)
                
                if (0 <= next_col < self.grid_cols and
                    0 <= next_row < self.grid_rows and
                    next_point not in closed_set and
                    next_point not in self.forbidden_zones):
                    
                    # 检查是否远离所有禁飞区顶角
                    is_safe = True
                    for corner in corner_safety_zones:
                        dist = ((next_point[0] - corner[0]) ** 2 + (next_point[1] - corner[1]) ** 2) ** 0.5
                        if dist < 0.5:  # 安全距离更大
                            is_safe = False
                            break
                            
                    if is_safe:
                        g_score = len(path) + 1
                        h_score = abs(next_point[0] - end[0]) + abs(next_point[1] - end[1])
                        f_score = g_score + h_score
                        
                        new_path = path + [next_point]
                        heapq.heappush(open_set, (f_score, next_point, new_path))
        
        # 如果找不到安全路径，返回None
        return None

    def find_edge_middle_point(self, point1, point2):
        """在两个点之间找一个在边缘上的中间点"""
        # 计算中间点坐标
        mid_col = (point1[0] + point2[0]) // 2
        mid_row = (point1[1] + point2[1]) // 2
        mid_point = (mid_col, mid_row)
        
        # 检查中间点是否在边缘
        if self.is_on_edge(mid_point) and mid_point not in self.forbidden_zones:
            return mid_point
            
        # 如果中间点不在边缘，尝试找附近的边缘点
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nearby = (mid_col + dx, mid_row + dy)
            if (0 <= nearby[0] < self.grid_cols and 0 <= nearby[1] < self.grid_rows and
                    self.is_on_edge(nearby) and nearby not in self.forbidden_zones):
                return nearby
                
        return None
        
    def find_nearby_edge_points(self, point, max_distance=3):
        """找到给定点附近的边缘点"""
        edge_points = []
        for col in range(max(0, point[0] - max_distance), min(self.grid_cols, point[0] + max_distance + 1)):
            for row in range(max(0, point[1] - max_distance), min(self.grid_rows, point[1] + max_distance + 1)):
                if (col, row) != point and self.is_on_edge((col, row)) and (col, row) not in self.forbidden_zones:
                    edge_points.append((col, row))
        return edge_points
        
    def manhattan_distance(self, p1, p2):
        """计算两点间的曼哈顿距离"""
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
        
    def path_length(self, start, end):
        """估计从起点到终点的路径长度"""
        path = self.get_shortest_path_between(start, end)
        return len(path) if path else float('inf')
        
    def find_path_to_edge_point(self, start, edge_point):
        """找到从起点到边缘点的路径"""
        # 首先尝试A*算法
        path = self.find_astar_path(start, edge_point)
        if path:
            return [start] + path
            
        # 如果失败，尝试灵活路径
        path = self.find_flexible_path(start, edge_point)
        if path:
            return [start] + path
            
        # 最后尝试相邻路径
        path = self.find_adjacent_path(start, edge_point)
        if path:
            return [start] + path
            
        return None

    def test_coordinate_conversion(self):
        """测试坐标转换的正确性"""
        print("=== 坐标转换测试 ===")
        print(f"网格规格: {self.grid_cols}×{self.grid_rows} = {self.grid_cols*self.grid_rows}个方格")
        print(f"方格大小: {self.cell_size*100}cm × {self.cell_size*100}cm")
        print(f"高度设置: 起飞={self.takeoff_height}m, 降落={self.landing_height}m, 巡查={self.survey_height}m")
        print(f"红点位置 (起降点): {self.red_point}")
        print(f"原点设置: col={self.origin_col}, row={self.origin_row}")

        # 测试关键点的坐标转换
        test_points = [
            self.red_point,  # 红点应该是(0,0,0)
            (0, 0),          # B7 A1 - 左上角
            (8, 0),          # B7 A9 - 右上角
            (0, 6),          # B1 A1 - 左下角
        ]

        print("\n基础坐标转换（默认高度）:")
        for col, row in test_points:
            coord = self.position_to_coord(col, row)
            global_x, global_y, global_z = self.grid_to_global_coords(col, row)
            print(f"{coord}: 网格({col},{row}) -> 全局({global_x:.1f}m,{global_y:.1f}m,{global_z:.1f}m)")

        print("\n航点高度测试（模拟5个航点）:")
        for i in range(5):
            col, row = self.red_point  # 使用红点位置测试
            coord = self.position_to_coord(col, row)
            global_x, global_y, global_z = self.grid_to_global_coords(col, row, i, 5)
            waypoint_type = "起飞" if i == 0 else "降落" if i == 4 else "巡查"
            print(f"航点{i+1}({waypoint_type}): {coord} -> 全局({global_x:.1f}m,{global_y:.1f}m,{global_z:.2f}m)")

        print("=== 测试完成 ===")

    def create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()

        # 文件菜单
        file_menu = menubar.addMenu('文件')

        # 保存航线动作（传统方式）
        save_action = QAction('保存航线到文件...', self)
        save_action.setShortcut('Ctrl+S')
        save_action.setStatusTip('将航线保存到指定的YAML文件')
        save_action.triggered.connect(self.save_waypoints)
        file_menu.addAction(save_action)

        file_menu.addSeparator()

        # 退出动作
        exit_action = QAction('退出', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.setStatusTip('退出应用程序')
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # 帮助菜单
        help_menu = menubar.addMenu('帮助')

        # 关于动作
        about_action = QAction('关于', self)
        about_action.setStatusTip('关于野生动物巡查系统')
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def show_about(self):
        """显示关于对话框"""
        about_text = (
            "野生动物巡查系统地面站 v2.0\n\n"
            "功能特性：\n"
            "• 63个50cm×50cm方格网格\n"
            "• 混合模式路径规划\n"
            "• 沿边缘优先返航\n"
            "• ROS topic航点传输\n"
            "• 差异化航点高度设置\n\n"
            "坐标系：以红点为原点\n"
            "• X轴：向前 (B1→B7)\n"
            "• Y轴：向左 (A9→A1)\n"
            "• Z轴：向上\n\n"
            "高度设置：\n"
            "• 起降高度：0.0米\n"
            "• 巡查高度：1.22米"
        )
        QMessageBox.about(self, "关于", about_text)

    def publish_waypoints_to_ros(self, waypoints_data):
        """将航点数据发布到ROS topic"""
        try:
            # 将YAML数据转换为JSON字符串发布
            json_data = json.dumps(waypoints_data, ensure_ascii=False, indent=2)

            # 创建ROS消息
            msg = String()
            msg.data = json_data

            # 发布消息
            self.waypoint_publisher.publish(msg)

            print(f"航点数据已发布到ROS topic: /wildlife_survey/waypoints")
            print(f"发布数据大小: {len(json_data)} 字符")

        except Exception as e:
            print(f"发布ROS消息失败: {e}")
            QMessageBox.warning(self, "警告", f"发布ROS消息失败: {e}")

    def upload_waypoints(self):
        """上传航线到ROS系统"""
        if not self.waypoints:
            QMessageBox.warning(self, "警告", "请先规划路径！")
            return

        # 显示进度条并开始上传过程
        self.start_upload_process()

    def start_upload_process(self):
        """开始上传过程，显示进度条"""
        # 显示进度条
        self.upload_progress.setVisible(True)
        self.upload_progress.setValue(0)
        self.upload_progress.setFormat("准备上传...")

        # 更新状态栏
        self.statusBar().showMessage("正在上传航线...")

        # 创建定时器控制进度条
        self.upload_timer = QTimer()
        self.upload_timer.timeout.connect(self.update_upload_progress)
        self.upload_step = 0
        self.upload_timer.start(100)  # 每100ms更新一次

    def update_upload_progress(self):
        """更新上传进度"""
        self.upload_step += 1
        progress = min(self.upload_step * 100 // 30, 100)  # 3秒 = 30步

        # 更新进度条
        self.upload_progress.setValue(progress)

        # 更新进度文本
        if progress < 30:
            self.upload_progress.setFormat("生成航点数据...")
        elif progress < 60:
            self.upload_progress.setFormat("发布到ROS topic...")
        elif progress < 90:
            self.upload_progress.setFormat("保存本地文件...")
        else:
            self.upload_progress.setFormat("上传完成!")

        # 在不同阶段执行实际操作
        if progress == 30:
            # 第1秒：生成数据
            self.prepare_waypoint_data()
        elif progress == 60:
            # 第2秒：发布ROS
            self.publish_to_ros()
        elif progress == 90:
            # 第3秒：保存文件
            self.save_local_file()
        elif progress >= 100:
            # 完成
            self.finish_upload()

    def prepare_waypoint_data(self):
        """准备航点数据"""
        try:
            self.waypoints_data = {
                'waypoints': [],
                'metadata': {
                    'total_points': len(self.waypoints),
                    'grid_size': f"{self.grid_cols}x{self.grid_rows}",
                    'total_cells': self.grid_cols * self.grid_rows,
                    'cell_size': {
                        'width_m': self.cell_size,
                        'height_m': self.cell_size,
                        'width_cm': self.cell_size * 100,
                        'height_cm': self.cell_size * 100
                    },
                    'coordinate_system': {
                        'origin': 'red_point' if not self.coordinate_system_initialized else 'drone_initial_position',
                        'x_axis': 'forward (B1->B7)',
                        'y_axis': 'left (A9->A1)',
                        'z_axis': 'up',
                        'units': 'meters'
                    },
                    'height_settings': {
                        'takeoff_height_m': self.takeoff_height,
                        'landing_height_m': self.landing_height,
                        'survey_height_m': self.survey_height,
                        'description': 'First waypoint at takeoff height, last waypoint at landing height, others at survey height'
                    },
                    'forbidden_zones': [self.position_to_coord(col, row) for col, row in self.forbidden_zones],
                    'red_point': self.position_to_coord(*self.red_point),
                    'red_point_global': {'x': 0, 'y': 0, 'z': 0}
                }
            }
            
            # 如果坐标系已初始化，添加初始位置信息
            if self.coordinate_system_initialized and self.drone_initial_position:
                self.waypoints_data['metadata']['drone_initial_position'] = {
                    'x': self.drone_initial_position[0],
                    'y': self.drone_initial_position[1],
                    'z': self.drone_initial_position[2]
                }
                self.waypoints_data['metadata']['coordinate_offsets'] = {
                    'offset_x': self.offset_x,
                    'offset_y': self.offset_y,
                    'offset_z': self.offset_z
                }

            for i, (col, row) in enumerate(self.waypoints):
                coord = self.position_to_coord(col, row)
                global_x, global_y, global_z = self.grid_to_global_coords(col, row, i, len(self.waypoints))

                # 确定航点动作类型
                if i == 0:
                    action = 'takeoff'
                elif i == len(self.waypoints) - 1:
                    action = 'land'
                else:
                    action = 'survey'

                # 如果是第一个航点且坐标系已初始化，使用无人机初始位置作为起点
                if i == 0 and self.coordinate_system_initialized and self.drone_initial_position:
                    waypoint = {
                        'id': i + 1,
                        'coordinate': 'DRONE_POSITION',
                        'grid_position': {'col': col, 'row': row},
                        'global_position': {'x': -self.offset_x, 'y': -self.offset_y, 'z': self.survey_height},
                        'action': 'takeoff',
                        'height_info': {
                            'is_takeoff_landing': True,
                            'height_m': self.survey_height
                        },
                        'original_drone_position': {
                            'x': self.drone_initial_position[0],
                            'y': self.drone_initial_position[1],
                            'z': self.drone_initial_position[2]
                        }
                    }
                else:
                    waypoint = {
                        'id': i + 1,
                        'coordinate': coord,
                        'grid_position': {'col': col, 'row': row},
                        'global_position': {'x': global_x, 'y': global_y, 'z': global_z},
                        'action': action,
                        'height_info': {
                            'is_takeoff_landing': i == 0 or i == len(self.waypoints) - 1,
                            'height_m': global_z
                        }
                    }
                self.waypoints_data['waypoints'].append(waypoint)

        except Exception as e:
            print(f"准备数据时出错: {e}")
            self.upload_error = str(e)

    def publish_to_ros(self):
        """发布数据到ROS topic"""
        try:
            self.publish_waypoints_to_ros(self.waypoints_data)
        except Exception as e:
            print(f"发布ROS数据时出错: {e}")
            self.upload_error = str(e)

    def save_local_file(self):
        """保存本地文件"""
        try:
            # 使用默认文件名
            filename = "waypoints.yaml"
            with open(filename, 'w', encoding='utf-8') as f:
                yaml.dump(self.waypoints_data, f, default_flow_style=False, allow_unicode=True)
            self.saved_filename = filename
        except Exception as e:
            print(f"保存本地文件时出错: {e}")
            self.upload_error = str(e)

    def finish_upload(self):
        """完成上传过程"""
        # 停止定时器
        self.upload_timer.stop()

        # 隐藏进度条
        QTimer.singleShot(1000, lambda: self.upload_progress.setVisible(False))

        # 检查是否有错误
        if hasattr(self, 'upload_error'):
            self.statusBar().showMessage("上传失败", 5000)
            QMessageBox.critical(self, "上传失败", f"上传过程中出现错误：\n{self.upload_error}")
            delattr(self, 'upload_error')
            return

        # 更新状态栏
        self.statusBar().showMessage("航线上传成功", 5000)

        # 显示成功信息
        self.show_upload_success()

    def show_upload_success(self):
        """显示上传成功信息"""
        total_distance = self.calculate_total_distance()
        takeoff_landing_count = 2 if len(self.waypoints) > 1 else 1
        survey_count = len(self.waypoints) - takeoff_landing_count

        success_info = (
            f"航线上传成功！\n\n"
            f"📁 本地文件：{getattr(self, 'saved_filename', 'waypoints.yaml')}\n"
            f"📡 ROS Topic：/wildlife_survey/waypoints\n\n"
            f"网格信息：\n"
            f"• 方格数量：{self.grid_cols}×{self.grid_rows} = {self.grid_cols*self.grid_rows}个\n"
            f"• 方格大小：{self.cell_size*100:.0f}cm × {self.cell_size*100:.0f}cm\n"
            f"• 总航点：{len(self.waypoints)}个\n"
            f"• 总距离：{total_distance:.1f}米\n\n"
            f"高度信息：\n"
                            f"• 起飞航点：1个，高度{self.takeoff_height}米\n"
                f"• 降落航点：1个，高度{self.landing_height}米\n"
            f"• 巡查航点：{survey_count}个，高度{self.survey_height}米\n\n"
            f"坐标系信息：\n"
            f"• 原点：红点起降点 (0,0,0)\n"
            f"• X轴：向前 (B1→B7)\n"
            f"• Y轴：向左 (A9→A1)\n"
            f"• Z轴：向上\n"
            f"• 单位：米"
        )
        QMessageBox.information(self, "上传成功", success_info)

    def save_waypoints(self):
        """保存航线到YAML文件（保留原功能）"""
        if not self.waypoints:
            QMessageBox.warning(self, "警告", "请先规划路径！")
            return
            
        filename, _ = QFileDialog.getSaveFileName(
            self, "保存航线", "waypoints.yaml", "YAML files (*.yaml)")
            
        if filename:
            try:
                waypoints_data = {
                    'waypoints': [],
                    'metadata': {
                        'total_points': len(self.waypoints),
                        'grid_size': f"{self.grid_cols}x{self.grid_rows}",
                        'total_cells': self.grid_cols * self.grid_rows,
                        'cell_size': {
                            'width_m': self.cell_size,
                            'height_m': self.cell_size,
                            'width_cm': self.cell_size * 100,
                            'height_cm': self.cell_size * 100
                        },
                        'coordinate_system': {
                            'origin': 'red_point',
                            'x_axis': 'forward (B1->B7)',
                            'y_axis': 'left (A9->A1)',
                            'z_axis': 'up',
                            'units': 'meters'
                        },
                        'height_settings': {
                            'takeoff_height_m': self.takeoff_height,
                            'landing_height_m': self.landing_height,
                            'survey_height_m': self.survey_height,
                            'description': 'First waypoint at takeoff height, last waypoint at landing height, others at survey height'
                        },
                        'forbidden_zones': [self.position_to_coord(col, row) for col, row in self.forbidden_zones],
                        'red_point': self.position_to_coord(*self.red_point),
                        'red_point_global': {'x': 0, 'y': 0, 'z': 0}
                    }
                }

                for i, (col, row) in enumerate(self.waypoints):
                    coord = self.position_to_coord(col, row)
                    global_x, global_y, global_z = self.grid_to_global_coords(col, row, i, len(self.waypoints))

                    # 确定航点动作类型
                    if i == 0:
                        action = 'takeoff'
                    elif i == len(self.waypoints) - 1:
                        action = 'land'
                    elif i > self.return_path_start_index:
                        action = 'back'  # 返回航线使用back动作
                    else:
                        action = 'survey'

                    waypoint = {
                        'id': i + 1,
                        'coordinate': coord,
                        'grid_position': {'col': col, 'row': row},
                        'global_position': {'x': global_x, 'y': global_y, 'z': global_z},
                        'action': action,
                        'height_info': {
                            'is_takeoff_landing': i == 0 or i == len(self.waypoints) - 1,
                            'height_m': global_z
                        }
                    }
                    waypoints_data['waypoints'].append(waypoint)
                    
                with open(filename, 'w', encoding='utf-8') as f:
                    yaml.dump(waypoints_data, f, default_flow_style=False, allow_unicode=True)

                # 发布航点数据到ROS topic
                self.publish_waypoints_to_ros(waypoints_data)

                # 显示保存成功信息，包含坐标系说明
                total_distance = self.calculate_total_distance()
                takeoff_landing_count = 2 if len(self.waypoints) > 1 else 1
                survey_count = len(self.waypoints) - takeoff_landing_count

                coord_info = (
                    f"航线已保存到 {filename}\n"
                    f"数据已发布到ROS topic: /wildlife_survey/waypoints\n\n"
                    f"网格信息：\n"
                    f"• 方格数量：{self.grid_cols}×{self.grid_rows} = {self.grid_cols*self.grid_rows}个\n"
                    f"• 方格大小：{self.cell_size*100:.0f}cm × {self.cell_size*100:.0f}cm\n"
                    f"• 总航点：{len(self.waypoints)}个\n"
                    f"• 总距离：{total_distance:.1f}米\n\n"
                    f"高度信息：\n"
                    f"• 起飞航点：1个，高度{self.takeoff_height}米\n"
                f"• 降落航点：1个，高度{self.landing_height}米\n"
                    f"• 巡查航点：{survey_count}个，高度{self.survey_height}米\n\n"
                    f"坐标系信息：\n"
                    f"• 原点：红点起降点 (0,0,0)\n"
                    f"• X轴：向前 (B1→B7)\n"
                    f"• Y轴：向左 (A9→A1)\n"
                    f"• Z轴：向上\n"
                    f"• 单位：米"
                )
                QMessageBox.information(self, "成功", coord_info)
                
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败: {str(e)}")
                
    def toggle_mission(self):
        """切换任务状态"""
        self.mission_active = not self.mission_active
        
        if self.mission_active:
            # 开始任务
            self.start_btn.setText("停止")
            self.start_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 8px; font-size: 14px; }")
            self.status_label.setText("状态: 任务进行中")
            
            # 重置任务完成标志
            self.mission_completed = False
            self.drone_returned_to_origin = False
            
            # 启动任务计时器
            self.mission_start_time = datetime.now()
            self.mission_duration = 0
            self.timer_label.setText("任务时间: 0分0秒")
            self.mission_timer.start(1000)  # 每秒更新一次
            
            # 记录开始时间并输出日志
            start_time_str = self.mission_start_time.strftime("%H:%M:%S")
            rospy.loginfo(f"任务开始时间: {start_time_str}, 将在{self.min_flight_time}秒后开始检测是否回到原点")
            
            # 任务开始时生成新的任务ID（如果之前有数据且未完成，先询问是否保存）
            if self.wildlife_detections:
                reply = QMessageBox.question(self, '开始新任务', 
                    '有未保存的野生动物检测数据，是否保存后再开始新任务？',
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
                
                if reply == QMessageBox.Yes:
                    self.save_wildlife_data()
                    self.wildlife_detections.clear()
                    self.update_wildlife_display()
            
            # 生成新的任务ID
            self.mission_id = datetime.now().strftime("%Y%m%d_%H%M%S")
            rospy.loginfo(f"开始新任务，ID: {self.mission_id}")
            
            # 清空轨迹，准备新任务
            self.drone_trajectory.clear()
            self.map_widget.update()
        else:
            # 停止任务
            self.start_btn.setText("开始")
            self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 15px; }")
            self.status_label.setText("状态: 待机")
            
            # 停止计时器
            self.mission_timer.stop()
            
            # 任务结束时，自动保存动物检测数据
            if self.wildlife_detections:
                try:
                    self.save_wildlife_data()
                    QMessageBox.information(self, "任务完成", f"任务已停止，野生动物检测数据已自动保存至:\n{os.path.join(self.history_dir, self.mission_id+'.json')}")
                except Exception as e:
                    rospy.logerr(f"自动保存动物检测数据失败: {e}")
                    QMessageBox.warning(self, "数据保存警告", f"自动保存数据时出现错误: {str(e)}")
            else:
                QMessageBox.information(self, "任务完成", "任务已停止，未检测到任何野生动物数据")
    
    def reset_mission(self):
        """重置任务"""
        self.mission_active = False
        self.start_btn.setText("开始")
        self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 15px; }")
        self.status_label.setText("状态: 待机")
        
        # 停止计时器
        self.mission_timer.stop()
        self.mission_duration = 0
        self.timer_label.setText("任务时间: 0分0秒")
        
        # 清空航点
        self.waypoints.clear()
        self.return_path_start_index = -1
        self.waypoint_count_label.setText("路径长度: 0 格")
        
        # 清空禁区
        self.forbidden_zones.clear()
        self.update_forbidden_status()
        
        # 更新显示
        self.map_widget.update()
        
        QMessageBox.information(self, "信息", "任务已重置")
        
    def publish_command(self):
        """发布命令到ROS话题"""
        command = Int32()
        command.data = 1 if self.mission_active else 0
        self.command_pub.publish(command)

    def update_wildlife_display(self):
        """更新野生动物检测信息显示"""
        if not hasattr(self, 'wildlife_info_text'):
            return
            
        # 构建显示文本
        html_text = "<html><body>"
        html_text += "<h3 style='font-size:13px;margin:5px 0;'>野生动物检测结果</h3>"
        
        # 检测是否是历史记录
        if self.loaded_history_id is not None:
            html_text += f"<p style='font-size:11px;color:#2196F3;margin:5px 0;'>当前显示: 历史记录 {self.loaded_history_id}</p>"
        
        # 显示已确认的检测记录
        html_text += "<table border='0' cellspacing='2' cellpadding='3' style='font-size:12px;'>"
        html_text += "<tr><th>方格</th><th>动物种类</th><th>数量</th></tr>"
        
        # 对检测记录按方格位置排序
        sorted_detections = sorted(self.wildlife_detections.items(), 
                                  key=lambda x: x[1]['grid_coord'])
        
        # 生成显示内容
        for grid_pos, data in sorted_detections:
            grid_name = data['grid_coord']
            for animal_name, count in data['animals'].items():
                html_text += f"<tr><td>{grid_name}</td><td>{animal_name}</td><td align='center'>{count}只</td></tr>"
        
        if not self.wildlife_detections:
            html_text += "<tr><td colspan='3' align='center' style='color:#666;'>暂无检测数据</td></tr>"
        
        html_text += "</table></body></html>"
        
        # 更新显示
        self.wildlife_info_text.setHtml(html_text)

    def create_wildlife_panel(self):
        """创建野生动物检测信息显示面板"""
        group_box = QGroupBox("野生动物检测信息")
        group_box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 12px; }")
        
        layout = QVBoxLayout(group_box)
        layout.setSpacing(3)  # 减小组件间距
        layout.setContentsMargins(5, 8, 5, 5)  # 减小内边距
        
        # 创建顶部操作栏
        top_bar = QHBoxLayout()
        top_bar.setSpacing(3)
        
        # 创建历史记录按钮
        self.history_btn = QPushButton("显示历史记录")
        self.history_btn.setStyleSheet("""
            QPushButton { 
                background-color: #2196F3; 
                color: white; 
                font-weight: bold; 
                padding: 3px;
                font-size: 10px; 
            }
        """)
        self.history_btn.setMaximumHeight(24)
        self.history_btn.clicked.connect(self.toggle_history_panel)
        top_bar.addWidget(self.history_btn)
        
        # 添加保存按钮
        save_btn = QPushButton("保存检测数据")
        save_btn.setStyleSheet("""
            QPushButton { 
                background-color: #4CAF50; 
                color: white; 
                font-weight: bold; 
                padding: 3px;
                font-size: 10px; 
            }
        """)
        save_btn.setMaximumHeight(24)
        save_btn.clicked.connect(self.save_wildlife_data)
        top_bar.addWidget(save_btn)
        
        layout.addLayout(top_bar)
        
        # 创建信息标签
        info_label = QLabel("检测到的野生动物将在此显示")
        info_label.setStyleSheet("QLabel { color: #666; font-size: 11px; }")
        layout.addWidget(info_label)
        
        # 创建一个滚动区域来包含文本显示
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)  # 允许内容调整大小
        scroll_area.setFrameShape(QFrame.NoFrame)  # 无边框
        
        # 创建文本显示区域
        self.wildlife_info_text = QTextBrowser()
        self.wildlife_info_text.setMinimumHeight(220)  # 减小最小高度
        self.wildlife_info_text.setStyleSheet("""
            QTextBrowser {
                background-color: #f9f9f9;
                border: 1px solid #ddd;
                font-family: Arial, sans-serif;
                font-size: 12px;  /* 增大字体 */
            }
        """)
        
        # 设置初始HTML内容
        initial_html = """
        <html>
        <body>
        <h3 style='font-size:13px;'>野生动物检测结果</h3>
        <p style='color:#666;font-size:11px;'>当无人机到达方格中心时，将显示检测到的野生动物信息</p>
        </body>
        </html>
        """
        self.wildlife_info_text.setHtml(initial_html)
        
        # 将文本浏览器放入滚动区域
        scroll_area.setWidget(self.wildlife_info_text)
        layout.addWidget(scroll_area)
        
        return group_box
        
    def create_history_panel(self):
        """创建历史记录面板（从右侧滑出）"""
        # 创建历史记录面板
        self.history_panel = QWidget(self)
        self.history_panel.setFixedWidth(300)
        self.history_panel.setStyleSheet("background-color: white; border-left: 1px solid #ccc;")
        
        # 设置面板位置（在窗口右侧）
        self.history_panel.setGeometry(self.width(), 0, 300, self.height())
        
        # 创建布局
        layout = QVBoxLayout(self.history_panel)
        
        # 创建标题
        title_label = QLabel("历史记录")
        title_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 5px;")
        layout.addWidget(title_label)
        
        # 创建历史记录列表
        self.history_list = QListWidget()
        self.history_list.setStyleSheet("""
            QListWidget {
                font-size: 11px;
                border: 1px solid #ddd;
            }
            QListWidget::item {
                padding: 5px;
                border-bottom: 1px solid #eee;
            }
            QListWidget::item:selected {
                background-color: #e0f0ff;
                color: black;
            }
        """)
        self.history_list.itemClicked.connect(self.on_history_item_clicked)
        layout.addWidget(self.history_list)
        
        # 创建底部按钮区域
        button_layout = QHBoxLayout()
        
        # 关闭按钮
        close_btn = QPushButton("关闭")
        close_btn.clicked.connect(self.toggle_history_panel)
        button_layout.addWidget(close_btn)
        
        # 刷新按钮
        refresh_btn = QPushButton("刷新列表")
        refresh_btn.clicked.connect(lambda: (setattr(self, 'history_records', self.load_history_list()), self.update_history_list()))
        button_layout.addWidget(refresh_btn)
        
        # 添加重置按钮
        reset_btn = QPushButton("重置地面站")
        reset_btn.setStyleSheet("""
            QPushButton { 
                background-color: #FF5722; 
                color: white; 
                font-weight: bold; 
                padding: 5px;
            }
        """)
        reset_btn.clicked.connect(self.reset_ground_station)
        layout.addWidget(reset_btn)
        
        layout.addLayout(button_layout)
        
        # 初始状态为隐藏
        self.history_panel.hide()
        
    def resizeEvent(self, event):
        """重写窗口大小改变事件，确保历史面板位置正确"""
        super().resizeEvent(event)
        if hasattr(self, 'history_panel'):
            if self.history_panel_visible:
                # 如果历史面板可见，显示在窗口内
                self.history_panel.setGeometry(self.width() - 300, 0, 300, self.height())
            else:
                # 如果历史面板隐藏，位置在窗口外
                self.history_panel.setGeometry(self.width(), 0, 300, self.height())
                
    def toggle_history_panel(self):
        """切换历史记录面板的显示状态"""
        if self.history_panel_visible:
            # 隐藏历史面板
            self.history_panel.setGeometry(self.width(), 0, 300, self.height())
            self.history_panel_visible = False
            self.history_btn.setText("显示历史记录")
        else:
            # 显示历史面板
            self.history_panel.setGeometry(self.width() - 300, 0, 300, self.height())
            self.history_panel.show()
            self.history_panel_visible = True
            self.history_btn.setText("隐藏历史记录")
            # 更新历史记录列表
            self.update_history_list()
    
    def on_history_item_clicked(self, item):
        """处理历史记录项点击事件"""
        record_id = item.data(Qt.UserRole)
        if record_id:
            if self.load_wildlife_history(record_id):
                rospy.loginfo(f"已加载历史记录: {record_id}")
                # 更新列表选中状态
                self.update_history_list()

    def load_history_list(self):
        """加载历史记录列表"""
        history_records = []
        try:
            if os.path.exists(self.history_dir):
                # 获取所有历史记录文件
                for filename in sorted(os.listdir(self.history_dir), reverse=True):
                    if filename.endswith('.json'):
                        record_id = os.path.splitext(filename)[0]
                        file_path = os.path.join(self.history_dir, filename)
                        
                        try:
                            # 尝试读取记录文件获取元数据
                            with open(file_path, 'r', encoding='utf-8') as f:
                                record_data = json.load(f)
                                
                            # 提取记录信息
                            timestamp = record_id
                            detection_count = 0
                            animal_count = 0
                            
                            if 'detections' in record_data:
                                detection_count = len(record_data['detections'])
                                # 统计动物总数
                                for grid_data in record_data['detections'].values():
                                    if 'animals' in grid_data:
                                        for count in grid_data['animals'].values():
                                            animal_count += count
                            
                            # 格式化时间显示
                            try:
                                if '_' in timestamp:
                                    date_part, time_part = timestamp.split('_')
                                    formatted_time = f"{date_part[:4]}-{date_part[4:6]}-{date_part[6:]} {time_part[:2]}:{time_part[2:4]}:{time_part[4:]}"
                                else:
                                    formatted_time = timestamp
                            except:
                                formatted_time = timestamp
                            
                            # 添加到记录列表
                            history_records.append({
                                'id': record_id,
                                'file_path': file_path,
                                'time': formatted_time,
                                'detection_count': detection_count,
                                'animal_count': animal_count
                            })
                        except Exception as e:
                            rospy.logwarn(f"读取历史记录文件 {filename} 失败: {e}")
            
            return history_records
        except Exception as e:
            rospy.logerr(f"加载历史记录列表失败: {e}")
            return []
    
    def save_wildlife_data(self):
        """保存野生动物检测数据"""
        if not self.wildlife_detections:
            rospy.loginfo("没有野生动物检测数据需要保存")
            return
        
        try:
            # 准备保存数据
            save_data = {
                'mission_id': self.mission_id,
                'timestamp': datetime.now().isoformat(),
                'detections': {}
            }
            
            # 转换数据格式
            for grid_pos, data in self.wildlife_detections.items():
                grid_key = f"{grid_pos[0]}_{grid_pos[1]}"
                save_data['detections'][grid_key] = {
                    'grid_position': {'col': grid_pos[0], 'row': grid_pos[1]},
                    'grid_coord': data['grid_coord'],
                    'detection_time': data['detection_time'],
                    'animals': data['animals']
                }
            
            # 保存到文件
            filename = f"{self.mission_id}.json"
            filepath = os.path.join(self.history_dir, filename)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(save_data, f, ensure_ascii=False, indent=2)
            
            rospy.loginfo(f"野生动物检测数据已保存至: {filepath}")
            
            # 更新历史记录列表
            self.history_records = self.load_history_list()
            # 如果历史面板可见，刷新显示
            if hasattr(self, 'history_list') and self.history_panel_visible:
                self.update_history_list()
                
        except Exception as e:
            rospy.logerr(f"保存野生动物数据失败: {e}")
    
    def load_wildlife_history(self, record_id):
        """加载历史野生动物检测数据"""
        try:
            filepath = os.path.join(self.history_dir, f"{record_id}.json")
            if not os.path.exists(filepath):
                rospy.logwarn(f"历史记录文件不存在: {filepath}")
                return False
                
            # 读取历史数据
            with open(filepath, 'r', encoding='utf-8') as f:
                record_data = json.load(f)
            
            # 清空当前检测记录
            self.wildlife_detections = {}
            
            # 加载历史检测记录
            if 'detections' in record_data:
                for grid_key, data in record_data['detections'].items():
                    col, row = map(int, grid_key.split('_'))
                    grid_pos = (col, row)
                    
                    self.wildlife_detections[grid_pos] = {
                        'detection_time': data.get('detection_time', 0),
                        'animals': data.get('animals', {}),
                        'grid_coord': data.get('grid_coord', '')
                    }
            
            # 更新已加载的历史记录ID
            self.loaded_history_id = record_id
            
            # 更新显示
            self.update_wildlife_display()
            self.map_widget.update()
            
            return True

        except Exception as e:
            rospy.logerr(f"加载历史记录失败: {e}")
            return False
    
    def update_history_list(self):
        """更新历史记录列表显示"""
        if not hasattr(self, 'history_list'):
            return
            
        # 清空列表
        self.history_list.clear()
        
        # 添加记录项
        for record in self.history_records:
            item = QListWidgetItem()
            item.setData(Qt.UserRole, record['id'])
            
            # 构建显示文本
            text = f"{record['time']}\n"
            text += f"检测区域: {record['detection_count']}处, 动物总数: {record['animal_count']}只"
            
            item.setText(text)
            
            # 设置选中状态
            if record['id'] == self.loaded_history_id:
                item.setSelected(True)
                item.setBackground(QColor(230, 245, 255))
            
            self.history_list.addItem(item)
    
    def reset_ground_station(self):
        """重置地面站状态"""
        # 如果有未保存的数据，提示用户保存
        if self.wildlife_detections and not self.mission_completed:
            reply = QMessageBox.question(self, '保存数据', 
                '有未保存的野生动物检测数据，是否保存？',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            
            if reply == QMessageBox.Yes:
                self.save_wildlife_data()
        
        # 重置任务状态
        self.mission_active = False
        self.mission_completed = False
        self.drone_returned_to_origin = False
        
        # 停止计时器
        self.mission_timer.stop()
        self.mission_duration = 0
        self.timer_label.setText("任务时间: 0分0秒")
        
        # 重置命令状态
        command = Int32()
        command.data = 0  # 0表示停止任务
        self.command_pub.publish(command)
        
        # 更新UI状态
        self.start_btn.setText("开始")
        self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 15px; }")
        self.status_label.setText("状态: 待机")
        
        # 清空航点
        self.waypoints.clear()
        self.return_path_start_index = -1
        self.waypoint_count_label.setText("路径长度: 0 格")
        
        # 清空禁区
        self.forbidden_zones.clear()
        self.update_forbidden_status()
        
        # 清空野生动物检测数据（创建新的任务ID）
        self.wildlife_detections.clear()
        self.mission_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        # 重置历史记录ID
        self.loaded_history_id = None
        self.update_wildlife_display()
        
        # 清空无人机轨迹
        self.drone_trajectory.clear()
        
        # 加载新的历史记录列表
        self.history_records = self.load_history_list()
        if self.history_panel_visible:
            self.update_history_list()
        
        # 更新地图显示
        self.map_widget.update()
        
        # 通知用户重置完成
        QMessageBox.information(self, "信息", "地面站已重置，准备开始新任务")
        
        # 关闭历史记录面板
        if self.history_panel_visible:
            self.toggle_history_panel()

    def update_mission_time(self):
        """更新任务持续时间"""
        self.mission_duration += 1
        
        # 显示当前任务时间
        minutes = self.mission_duration // 60
        seconds = self.mission_duration % 60
        self.timer_label.setText(f"任务时间: {minutes}分{seconds}秒")

    def stop_mission(self):
        """停止任务"""
        # 如果任务已完成（自动回到原点），设置状态为任务完成
        if self.mission_completed:
            self.mission_active = False
            self.drone_returned_to_origin = True
            self.status_label.setText("状态: 任务完成")
            self.start_btn.setText("开始")
            self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; font-size: 14px; }")
            self.statusBar().showMessage("任务已完成，无人机已返回原点", 5000)
            self.mission_timer.stop()
            
            # 发送停止命令
            command = Int32()
            command.data = 0  # 0表示停止任务
            self.command_pub.publish(command)
            rospy.loginfo("已发送停止命令(0)，防止无人机循环飞行")
        else:
            # 如果是手动停止任务
            self.mission_active = False
            self.start_btn.setText("开始")
            self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; font-size: 14px; }")
            self.status_label.setText("状态: 任务已停止")
            self.mission_timer.stop()
            
            # 发送停止命令
            command = Int32()
            command.data = 0  # 0表示停止任务
            self.command_pub.publish(command)
            rospy.loginfo("手动停止任务，已发送停止命令(0)")
            
            # 对于手动停止，也保存动物数据
            current_time = rospy.Time.now().to_sec()
            if current_time - self.last_save_time > 5.0 and self.wildlife_detections:
                try:
                    self.save_wildlife_data()
                    self.last_save_time = current_time
                    self.statusBar().showMessage("手动停止任务，已自动保存野生动物数据", 5000)
                except Exception as e:
                    rospy.logerr(f"自动保存动物检测数据失败: {e}")
                    QMessageBox.warning(self, "数据保存警告", f"自动保存数据时出现错误: {str(e)}")

    def check_drone_online_status(self):
        """检查无人机在线状态"""
        current_time = rospy.Time.now().to_sec()
        if current_time - self.last_odom_time > 0.3:  # 降低超时时间，与10Hz频率匹配
            self.drone_online = False
            self.drone_status_label.setText("● 无人机离线")
            self.drone_status_label.setStyleSheet("QLabel { color: red; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        else:
            self.drone_online = True
            self.drone_status_label.setText("● 无人机在线")
            self.drone_status_label.setStyleSheet("QLabel { color: green; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
            
        # 更新当前时间，避免始终显示离线
        if not self.drone_online and current_time - self.last_odom_time > 10.0:
            self.last_odom_time = current_time - 5.0  # 保持一定时间差，避免立即切换为在线
            
        # 检查按钮状态
        self.check_button_status()
    
    def check_button_status(self):
        """检查是否需要启用按钮（首次收到坐标3秒后）"""
        if self.first_odom_received and not self.buttons_enabled:
            current_time = rospy.Time.now().to_sec()
            if current_time - self.first_odom_time >= 3.0:
                self.buttons_enabled = True
                # 启用上传和开始按钮
                self.enable_control_buttons()
                rospy.loginfo("已启用控制按钮")
    
    def enable_control_buttons(self):
        """启用控制按钮"""
        # 找到需要启用的按钮并启用
        for widget in self.findChildren(QPushButton):
            if widget.text() in ["上传航线", "开始"]:
                widget.setEnabled(True)
                # 添加提示信息
                widget.setToolTip("已启用 - 无人机在线")
    
    def create_control_panel(self):
        """创建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(3)  # 更小的垂直间距
        layout.setContentsMargins(2, 2, 2, 2)  # 更小的边距
        
        # 状态显示 - 改为垂直布局，每个信息独占一行
        status_group = QGroupBox("系统状态")
        status_layout = QVBoxLayout()  # 使用垂直布局
        status_layout.setSpacing(2)  # 组件间距
        status_layout.setContentsMargins(4, 6, 4, 6)  # 内边距
        
        # 无人机在线状态显示
        self.drone_status_label = QLabel("● 无人机离线")
        self.drone_status_label.setStyleSheet("QLabel { color: red; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        self.drone_status_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.drone_status_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.drone_status_label)
        
        # 任务状态显示
        self.status_label = QLabel("状态: 待机")
        self.status_label.setStyleSheet("QLabel { font-size: 13px; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        self.status_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.status_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.status_label)
        
        # 航点数量
        self.waypoint_count_label = QLabel("路径长度: 0 格")
        self.waypoint_count_label.setStyleSheet("QLabel { font-family: 'Monospace', 'Courier New'; }")
        self.waypoint_count_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.waypoint_count_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.waypoint_count_label)
        
        # 任务计时标签
        self.timer_label = QLabel("任务时间: 0分0秒")
        self.timer_label.setStyleSheet("QLabel { color: #FF5722; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        self.timer_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.timer_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.timer_label)
        
        # 无人机位置显示
        self.drone_position_label = QLabel("无人机位置: X=0.00m Y=0.00m Z=0.00m")
        self.drone_position_label.setStyleSheet("QLabel { color: #FF8C00; font-family: 'Monospace', 'Courier New'; }")
        self.drone_position_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.drone_position_label)
        
        # 距离方格中心的距离显示
        self.distance_to_center_label = QLabel("距离方格中心: 0.00m")
        self.distance_to_center_label.setStyleSheet("QLabel { color: #4CAF50; font-family: 'Monospace', 'Courier New'; }")
        self.distance_to_center_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.distance_to_center_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.distance_to_center_label)
        
        # 当前区域ID显示
        self.current_region_label = QLabel("当前区域: --")
        self.current_region_label.setStyleSheet("QLabel { color: #2196F3; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")
        self.current_region_label.setFixedWidth(220)  # 固定宽度，防止抖动
        self.current_region_label.setAlignment(Qt.AlignLeft)  # 左对齐
        status_layout.addWidget(self.current_region_label)
        
        # 设置布局
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 禁区设置和路径规划合并到一行
        action_panel = QHBoxLayout()
        action_panel.setSpacing(2)
        
        # 左侧：禁区设置
        forbidden_group = QGroupBox("禁区设置")
        forbidden_group.setMaximumHeight(95)  # 限制高度
        forbidden_layout = QVBoxLayout(forbidden_group)
        forbidden_layout.setSpacing(2)
        forbidden_layout.setContentsMargins(4, 6, 4, 6)
        
        forbidden_status_layout = QHBoxLayout()
        forbidden_status_layout.addWidget(QLabel("已选禁区:"))
        self.forbidden_status_label = QLabel("0/3")
        forbidden_status_layout.addWidget(self.forbidden_status_label)
        
        forbidden_layout.addLayout(forbidden_status_layout)
        
        clear_forbidden_btn = QPushButton("清除禁区")
        clear_forbidden_btn.setMinimumHeight(36)  # 增加按钮高度
        clear_forbidden_btn.setStyleSheet("QPushButton { font-size: 12px; font-weight: bold; padding: 6px; }")  # 增加字体大小和内边距
        clear_forbidden_btn.clicked.connect(self.clear_forbidden_zones)
        forbidden_layout.addWidget(clear_forbidden_btn)
        
        action_panel.addWidget(forbidden_group)
        
        # 右侧：路径规划
        planning_group = QGroupBox("路径规划")
        planning_group.setMaximumHeight(95)  # 限制高度
        planning_layout = QVBoxLayout(planning_group)
        planning_layout.setSpacing(2)
        planning_layout.setContentsMargins(4, 6, 4, 6)
        
        plan_btn = QPushButton("规划路径")
        plan_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 6px; font-size: 12px; }")
        plan_btn.setMinimumHeight(36)  # 增加按钮高度
        plan_btn.clicked.connect(self.plan_path)
        planning_layout.addWidget(plan_btn)

        upload_btn = QPushButton("上传航线")
        upload_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 6px; font-size: 12px; }")
        upload_btn.setMinimumHeight(36)  # 增加按钮高度
        upload_btn.clicked.connect(self.upload_waypoints)
        planning_layout.addWidget(upload_btn)
        
        # 添加进度条
        self.upload_progress = QProgressBar()
        self.upload_progress.setVisible(False)  # 初始隐藏
        self.upload_progress.setMaximumHeight(8)  # 进一步减小高度
        self.upload_progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid grey;
                border-radius: 2px;
                text-align: center;
                font-weight: bold;
                font-size: 7px;
            }
            QProgressBar::chunk {
                background-color: #2196F3;
                border-radius: 1px;
            }
        """)
        planning_layout.addWidget(self.upload_progress)
        
        action_panel.addWidget(planning_group)
        
        layout.addLayout(action_panel)
        
        # 任务控制 - 使用水平布局
        mission_group = QGroupBox("任务控制")
        mission_group.setMaximumHeight(95)  # 增加一点高度
        mission_layout = QHBoxLayout(mission_group)
        mission_layout.setSpacing(4)
        mission_layout.setContentsMargins(4, 6, 4, 6)
        
        self.start_btn = QPushButton("开始")
        self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 8px; font-size: 14px; }")
        self.start_btn.setMinimumHeight(42)  # 增加按钮高度
        self.start_btn.clicked.connect(self.toggle_mission)
        mission_layout.addWidget(self.start_btn, 2)  # 比例为2
        
        button_layout = QVBoxLayout()
        button_layout.setSpacing(2)
        
        self.reset_btn = QPushButton("重置")
        self.reset_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 6px; font-size: 12px; }")
        self.reset_btn.setMinimumHeight(36)  # 增加按钮高度
        self.reset_btn.clicked.connect(self.reset_mission)
        button_layout.addWidget(self.reset_btn)
        
        self.clear_trajectory_btn = QPushButton("清除轨迹")
        self.clear_trajectory_btn.setStyleSheet("QPushButton { background-color: #FF8C00; color: white; font-weight: bold; padding: 6px; font-size: 12px; }")
        self.clear_trajectory_btn.setMinimumHeight(36)  # 增加按钮高度
        self.clear_trajectory_btn.clicked.connect(self.clear_trajectory)
        button_layout.addWidget(self.clear_trajectory_btn)
        
        mission_layout.addLayout(button_layout, 1)  # 比例为1
        
        layout.addWidget(mission_group)
        
        return panel

    def find_vertical_edge_point(self, start_pos):
        """寻找垂直方向上最近的边缘点"""
        col, row = start_pos
        
        # 检查四个方向的垂直路径（上下左右）
        # 上方 - 保持col不变，寻找最小的行索引(row=0)
        top_edge = (col, 0)
        top_path_safe = True
        for r in range(row-1, -1, -1):  # 从当前行向上检查
            if (col, r) in self.forbidden_zones:
                top_path_safe = False
                break
        
        # 下方 - 保持col不变，寻找最大的行索引(row=self.grid_rows-1)
        bottom_edge = (col, self.grid_rows-1)
        bottom_path_safe = True
        for r in range(row+1, self.grid_rows):  # 从当前行向下检查
            if (col, r) in self.forbidden_zones:
                bottom_path_safe = False
                break
        
        # 左方 - 保持row不变，寻找最小的列索引(col=0)
        left_edge = (0, row)
        left_path_safe = True
        for c in range(col-1, -1, -1):  # 从当前列向左检查
            if (c, row) in self.forbidden_zones:
                left_path_safe = False
                break
                
        # 右方 - 保持row不变，寻找最大的列索引(col=self.grid_cols-1)
        right_edge = (self.grid_cols-1, row)
        right_path_safe = True
        for c in range(col+1, self.grid_cols):  # 从当前列向右检查
            if (c, row) in self.forbidden_zones:
                right_path_safe = False
                break
                
        # 收集所有安全的边缘点及其距离
        edge_candidates = []
        
        if top_path_safe and top_edge not in self.forbidden_zones:
            distance = abs(row - 0)  # 到上边缘的距离
            edge_candidates.append((top_edge, distance))
            
        if bottom_path_safe and bottom_edge not in self.forbidden_zones:
            distance = abs(row - (self.grid_rows-1))  # 到下边缘的距离
            edge_candidates.append((bottom_edge, distance))
            
        if left_path_safe and left_edge not in self.forbidden_zones:
            distance = abs(col - 0)  # 到左边缘的距离
            edge_candidates.append((left_edge, distance))
            
        if right_path_safe and right_edge not in self.forbidden_zones:
            distance = abs(col - (self.grid_cols-1))  # 到右边缘的距离
            edge_candidates.append((right_edge, distance))
            
        # 选择距离最短的安全边缘点
        if edge_candidates:
            edge_candidates.sort(key=lambda x: x[1])  # 按距离排序
            return edge_candidates[0][0]  # 返回最近的边缘点
            
        return None  # 如果没有找到安全的垂直边缘点

    def find_optimized_edge_return_path(self, start_pos, unvisited_points):
        """寻找优化的沿边缘返回路径，使总路径最短"""
        if not unvisited_points:
            # 如果没有剩余点，直接找到边缘到起降点的路径
            return self.find_edge_path_to_landing(start_pos)
            
        # 如果起点不在边缘，先移动到边缘
        if not self.is_on_edge(start_pos):
            edge_point = self.find_vertical_edge_point(start_pos)
            if not edge_point:
                edge_point = self.find_nearest_edge_point(start_pos)
                
            if not edge_point:
                return None
                
            start_pos = edge_point
            
        # 收集所有边缘点
        edge_points = []
        for col in range(self.grid_cols):
            edge_points.append((col, 0))  # 上边缘
            edge_points.append((col, self.grid_rows-1))  # 下边缘
            
        for row in range(self.grid_rows):
            edge_points.append((0, row))  # 左边缘
            edge_points.append((self.grid_cols-1, row))  # 右边缘
            
        # 过滤掉禁区点和重复点
        edge_points = list(set(edge_points))
        edge_points = [p for p in edge_points if p not in self.forbidden_zones]
        
        # 添加起降点（如果它在边缘）
        if self.is_on_edge(self.red_point) and self.red_point not in edge_points:
            edge_points.append(self.red_point)
            
        # 创建边缘图（相邻边缘点之间的连接）
        edge_graph = {}
        for p1 in edge_points:
            edge_graph[p1] = []
            for p2 in edge_points:
                if p1 != p2 and self.are_adjacent_edge_points(p1, p2):
                    edge_graph[p1].append(p2)
        
        # 寻找最短路径，覆盖所有剩余的未访问点，优先沿边缘
        path = self.solve_edge_tsp(start_pos, self.red_point, unvisited_points, edge_graph)
        
        # 如果找不到覆盖所有点的路径，至少确保返回到起降点
        if not path:
            # 尝试直接沿边缘返回起降点
            landing_path = self.find_edge_path_direct(start_pos, self.red_point)
            if landing_path:
                path = landing_path + [self.red_point]
            else:
                # 如果无法沿边缘直接返回，尝试通过中间边缘点返回
                nearest_to_landing = self.find_nearest_edge_to_landing()
                if nearest_to_landing:
                    mid_path = self.find_edge_path_direct(start_pos, nearest_to_landing)
                    final_path = self.get_shortest_path_between(nearest_to_landing, self.red_point)
                    if mid_path and final_path:
                        path = mid_path + final_path
        
        return path
        
    def are_adjacent_edge_points(self, p1, p2):
        """检查两个边缘点是否相邻（允许斜线相邻）"""
        if p1 == p2:
            return False
            
        col1, row1 = p1
        col2, row2 = p2
        
        # 检查是否在同一边缘上的相邻点
        if col1 == col2 == 0 or col1 == col2 == self.grid_cols-1:  # 左边缘或右边缘
            return abs(row1 - row2) == 1
            
        if row1 == row2 == 0 or row1 == row2 == self.grid_rows-1:  # 上边缘或下边缘
            return abs(col1 - col2) == 1
            
        # 检查是否在相邻边缘的交点（四个角点）
        if ((col1 == 0 or col1 == self.grid_cols-1) and 
            (row2 == 0 or row2 == self.grid_rows-1) and
            (abs(col1 - col2) == 1 or abs(row1 - row2) == 1)):
            return True

        if ((col2 == 0 or col2 == self.grid_cols-1) and 
            (row1 == 0 or row1 == self.grid_rows-1) and
            (abs(col1 - col2) == 1 or abs(row1 - row2) == 1)):
            return True
            
        # 其他情况，不相邻
        return False
        
    def solve_edge_tsp(self, start, end, unvisited_points, edge_graph):
        """解决边缘TSP问题，寻找覆盖所有未访问点的最短边缘路径"""
        # 如果未访问点过多，使用贪心算法；否则使用动态规划
        if len(unvisited_points) > 8:  # 阈值可根据实际性能调整
            return self.greedy_edge_path(start, end, unvisited_points, edge_graph)
        else:
            return self.optimal_edge_path(start, end, unvisited_points, edge_graph)
            
    def greedy_edge_path(self, start, end, unvisited_points, edge_graph):
        """使用贪心算法寻找沿边缘的近似最优路径"""
        path = []
        current = start
        remaining = unvisited_points.copy()
        
        # 如果起点不在边缘图中，先移动到最近的边缘点
        if current not in edge_graph:
            nearest_edge = min(edge_graph.keys(), 
                              key=lambda p: abs(p[0]-current[0]) + abs(p[1]-current[1]))
            path_to_edge = self.get_shortest_path_between(current, nearest_edge)
            if path_to_edge:
                path.extend(path_to_edge[1:])  # 不包括当前点
                current = nearest_edge
            else:
                return None
                
        # 贪心选择下一个点，直到所有点都被覆盖
        while remaining:
            # 寻找能覆盖最多未访问点的边缘点
            best_next = None
            best_path = None
            max_covered = -1
            
            for next_edge in edge_graph.get(current, []):
                edge_path = [next_edge]  # 先考虑单步路径
                
                # 计算这条路径能覆盖多少未访问点
                covered_points = [p for p in remaining if self.point_covered_by_path(p, edge_path)]
                covered_count = len(covered_points)
                
                # 考虑距离因素
                distance = abs(next_edge[0] - current[0]) + abs(next_edge[1] - current[1])
                distance_to_end = abs(next_edge[0] - end[0]) + abs(next_edge[1] - end[1])
                
                # 使用加权评分：覆盖点数 * 10 - 距离 - 到终点距离 * 0.5
                score = covered_count * 10 - distance - distance_to_end * 0.5
                
                if score > max_covered:
                    max_covered = score
                    best_next = next_edge
                    best_path = edge_path
            
            # 如果找不到更好的点，尝试更长的路径搜索
            if best_next is None:
                # 尝试两步路径
                for next_edge in edge_graph.get(current, []):
                    for next_next_edge in edge_graph.get(next_edge, []):
                        if next_next_edge != current:  # 避免回到起点
                            edge_path = [next_edge, next_next_edge]
                            
                            covered_points = [p for p in remaining if self.point_covered_by_path(p, edge_path)]
                            covered_count = len(covered_points)
                            
                            if covered_count > max_covered:
                                max_covered = covered_count
                                best_next = next_next_edge
                                best_path = edge_path
            
            # 如果仍找不到好的路径，直接前往终点
            if best_next is None:
                # 尝试找到边缘上到终点的路径
                to_end_path = self.find_edge_path_direct(current, end)
                if to_end_path:
                    path.extend(to_end_path)
                    break
                else:
                    # 如果无法沿边缘到达终点，尝试直接到达终点
                    direct_path = self.get_shortest_path_between(current, end)
                    if direct_path:
                        path.extend(direct_path[1:])  # 不包括当前点
                    break
            
            # 添加找到的路径
            if best_path:
                path.extend(best_path)
                
                # 更新当前位置和剩余未访问点
                current = best_path[-1]
                
                # 移除已覆盖的点
                for point in list(remaining):
                    if self.point_covered_by_path(point, best_path):
                        remaining.remove(point)
        
        # 最后确保到达终点
        if current != end:
            final_path = self.find_edge_path_direct(current, end)
            if final_path:
                path.extend(final_path)
            else:
                # 如果无法沿边缘到达终点，尝试直接到达终点
                direct_path = self.get_shortest_path_between(current, end)
                if direct_path:
                    path.extend(direct_path[1:])  # 不包括当前点
        
        return path
        
    def optimal_edge_path(self, start, end, unvisited_points, edge_graph):
        """使用动态规划寻找沿边缘的最优路径（适用于少量点）"""
        # 如果未访问点太多，回退到贪心算法
        if len(unvisited_points) > 8:  # 阈值可调整
            return self.greedy_edge_path(start, end, unvisited_points, edge_graph)
            
        # 收集所有必须访问的点：起点、终点和未访问点
        all_points = list(unvisited_points) + [end]
        
        # 如果起点不在all_points中且不等于终点，添加起点
        if start != end and start not in all_points:
            all_points.append(start)
            
        # 构建距离矩阵
        n = len(all_points)
        dist = [[float('inf')] * n for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    p1 = all_points[i]
                    p2 = all_points[j]
                    
                    # 如果两点都在边缘上且相邻或存在边缘路径，使用边缘距离
                    if p1 in edge_graph and p2 in edge_graph:
                        edge_path = self.find_edge_path_direct(p1, p2)
                        if edge_path:
                            dist[i][j] = len(edge_path) + 1  # +1表示包括终点
                        else:
                            # 如果没有边缘路径，使用曼哈顿距离
                            dist[i][j] = abs(p2[0] - p1[0]) + abs(p2[1] - p1[1])
                    else:
                        # 对于非边缘点，使用最短路径距离
                        path = self.get_shortest_path_between(p1, p2)
                        if path:
                            dist[i][j] = len(path)
                        else:
                            dist[i][j] = float('inf')  # 不可达
                            
        # 使用动态规划求解TSP
        # 创建状态数组：dp[mask][i] 表示访问mask中的城市并以i结束的最小距离
        start_idx = all_points.index(start)
        end_idx = all_points.index(end)
        
        # 仅考虑必须访问的点（未访问点），不含起点和终点
        must_visit = [i for i, p in enumerate(all_points) if p in unvisited_points]
        
        if not must_visit:  # 如果没有必须访问的点，直接返回起点到终点的路径
            edge_path = self.find_edge_path_direct(start, end)
            if edge_path:
                return edge_path
            else:
                return self.get_shortest_path_between(start, end)
                
        # 使用贪心算法替代完整的DP
        # 从起点开始，每次选择距离最近的未访问点
        current_idx = start_idx
        path = [start]
        visited = {start_idx}
        
        while len(visited) < len(must_visit) + 1:  # +1是因为起点已访问
                         # 找出最近的未访问点
            next_idx = min([i for i in must_visit if i not in visited], key=lambda i: dist[current_idx][i])
            
            # 找到从当前点到下一个点的实际路径
            current = all_points[current_idx]
            next_point = all_points[next_idx]
            
            if current in edge_graph and next_point in edge_graph:
                edge_path = self.find_edge_path_direct(current, next_point)
                if edge_path:
                    path.extend(edge_path)
                else:
                    # 如果没有边缘路径，使用最短路径
                    shortest_path = self.get_shortest_path_between(current, next_point)
                    if shortest_path:
                        path.extend(shortest_path[1:])  # 不包括当前点
            else:
                # 对于非边缘点，使用最短路径
                shortest_path = self.get_shortest_path_between(current, next_point)
                if shortest_path:
                    path.extend(shortest_path[1:])  # 不包括当前点
                    
            # 更新当前点和访问状态
            current_idx = next_idx
            visited.add(next_idx)
            
        # 最后从最后一个访问的点到终点
        current = all_points[current_idx]
        
        if current != end:
            if current in edge_graph and end in edge_graph:
                edge_path = self.find_edge_path_direct(current, end)
                if edge_path:
                    path.extend(edge_path)
                else:
                    # 如果没有边缘路径，使用最短路径
                    shortest_path = self.get_shortest_path_between(current, end)
                    if shortest_path:
                        path.extend(shortest_path[1:])  # 不包括当前点
            else:
                # 对于非边缘点，使用最短路径
                shortest_path = self.get_shortest_path_between(current, end)
                if shortest_path:
                    path.extend(shortest_path[1:])  # 不包括当前点
                    
        # 移除起点，只返回路径部分
        return path[1:] if path else None
        
    def point_covered_by_path(self, point, path):
        """检查一个点是否被路径覆盖（点在路径上或与路径上的点相邻）"""
        if not path:
            return False
            
        if point in path:
            return True
            
        # 检查点是否与路径上的任何点相邻
        for path_point in path:
            if (abs(point[0] - path_point[0]) + abs(point[1] - path_point[1])) == 1:
                return True
                
        return False

    def find_astar_path(self, start, end, allow_diagonal=False):
        """使用A*算法寻找最短路径，可选是否允许对角线移动，避开禁飞区及其顶角"""
        import heapq
        
        # 如果起点和终点相同，直接返回终点
        if start == end:
            return [end]
        
        # 定义启发式函数（曼哈顿距离）
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        # 定义可能的移动方向
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 上右下左
        if allow_diagonal:
            directions.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])  # 对角线
        
        # 初始化开放列表和关闭列表
        open_set = []
        heapq.heappush(open_set, (0, start))  # (f_score, 位置)
        
        came_from = {}  # 路径追踪
        
        g_score = {start: 0}  # 从起点到当前点的实际距离
        f_score = {start: heuristic(start, end)}  # 估计的总距离
        
        open_set_hash = {start}  # 用于快速查找开放列表中的节点
        
        while open_set:
            # 获取f_score最小的节点
            current = heapq.heappop(open_set)[1]
            open_set_hash.remove(current)
            
            # 如果到达终点
            if current == end:
                # 构建路径
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()  # 反转路径（从起点到终点）
                path.append(end)  # 确保终点在路径中
                return path[1:]  # 不包括起点
            
            # 检查相邻节点
            for dx, dy in directions:
                neighbor_col = current[0] + dx
                neighbor_row = current[1] + dy
                neighbor = (neighbor_col, neighbor_row)
                
                # 检查边界
                if not (0 <= neighbor_col < self.grid_cols and 0 <= neighbor_row < self.grid_rows):
                    continue
                    
                # 检查是否是禁区
                if neighbor in self.forbidden_zones:
                    continue
                
                # 检查对角线移动是否安全（不穿过禁区）
                if abs(dx) == 1 and abs(dy) == 1:  # 对角线移动
                    # 检查相邻的两个格子是否都不是禁区
                    if (current[0] + dx, current[1]) in self.forbidden_zones or (current[0], current[1] + dy) in self.forbidden_zones:
                        continue
                
                # 检查移动是否会经过禁飞区顶角
                if self.path_crosses_forbidden_zone(current, neighbor):
                    continue
                    
                # 计算新的g_score
                # 对角线移动的距离为1.414（根号2），直线移动为1
                move_cost = 1.414 if (abs(dx) == 1 and abs(dy) == 1) else 1
                tentative_g_score = g_score.get(current, float('inf')) + move_cost
                
                # 如果找到了更好的路径
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # 更新路径信息
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    
                    # 如果邻居不在开放列表中，添加它
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        
        # 如果没有找到路径
        return None

    def find_edge_landing_path(self, start_pos):
        """生成最后3个在相邻边缘的航点，用于无人机斜降落
        
        确保航点路径为：当前位置 -> 边缘航点1 -> 边缘航点2 -> 红点(起降点)
        其中边缘航点1、边缘航点2和红点在边缘上且相邻
        """
        red_point = self.red_point
        
        # 如果起点已经在边缘上，并且与红点相邻或隔一个点，可以直接使用
        if self.is_on_edge(start_pos) and self.manhattan_distance(start_pos, red_point) <= 2:
            # 如果起点和红点相邻，找一个中间点
            if self.manhattan_distance(start_pos, red_point) == 1:
                # 使用起点和红点作为前两个点
                return [red_point]
            # 如果距离为2，需要找一个中间点
            elif self.manhattan_distance(start_pos, red_point) == 2:
                # 尝试找一个在边缘的中间点
                middle_point = self.find_edge_middle_point(start_pos, red_point)
                if middle_point:
                    return [middle_point, red_point]
        
        # 找到离红点最近的两个边缘点
        edge_points = self.find_nearby_edge_points(red_point)
        if not edge_points or len(edge_points) < 2:
            return None
            
        # 选择最近的两个边缘点
        edge_points.sort(key=lambda p: self.manhattan_distance(p, red_point))
        edge_point1, edge_point2 = edge_points[0], edge_points[1]
        
        # 确保edge_point1和edge_point2相邻且都在边缘上
        if not (self.is_on_edge(edge_point1) and self.is_on_edge(edge_point2) and 
                self.manhattan_distance(edge_point1, edge_point2) == 1):
            # 如果不满足条件，尝试找其他符合条件的点
            valid_pairs = []
            for i, p1 in enumerate(edge_points):
                for p2 in edge_points[i+1:]:
                    if (self.is_on_edge(p1) and self.is_on_edge(p2) and 
                            self.manhattan_distance(p1, p2) == 1 and
                            self.manhattan_distance(p2, red_point) == 1):
                        valid_pairs.append((p1, p2))
                        
            if valid_pairs:
                # 选择到起点距离最短的一对
                valid_pairs.sort(key=lambda pair: self.path_length(start_pos, pair[0]))
                edge_point1, edge_point2 = valid_pairs[0]
            else:
                # 没有符合条件的点对，选择离红点最近的点
                nearest = min(edge_points, key=lambda p: self.manhattan_distance(p, red_point))
                if self.manhattan_distance(nearest, red_point) == 1:
                    # 如果最近点与红点相邻，直接用它
                    return [nearest, red_point]
                else:
                    # 否则无法满足要求
                    return None
        
        # 验证路径是否安全（不穿过禁区）
        if (self.path_crosses_forbidden_zone(edge_point1, edge_point2) or
                self.path_crosses_forbidden_zone(edge_point2, red_point)):
            return None
            
        # 找到从起点到edge_point1的路径
        path_to_edge = self.find_path_to_edge_point(start_pos, edge_point1)
        if not path_to_edge:
            # 如果无法到达edge_point1，尝试edge_point2
            path_to_edge = self.find_path_to_edge_point(start_pos, edge_point2)
            if path_to_edge:
                # 交换点顺序
                edge_point1, edge_point2 = edge_point2, edge_point1
            else:
                # 两个点都无法到达
                return None
        
        # 组装最终路径：start_pos -> path_to_edge -> edge_point1 -> edge_point2 -> red_point
        # 但我们不需要返回start_pos和path_to_edge，因为它们会在调用函数中处理
        if path_to_edge[-1] == edge_point1:
            return [edge_point2, red_point]
        else:
            # 如果path_to_edge已经包含了edge_point1，直接返回剩余点
            path_to_edge_end = path_to_edge[-1]
            if path_to_edge_end == edge_point2:
                return [red_point]
            else:
                return [edge_point1, edge_point2, red_point]

    def find_edge_middle_point(self, point1, point2):
        """在两个点之间找一个在边缘上的中间点"""
        # 计算中间点坐标
        mid_col = (point1[0] + point2[0]) // 2
        mid_row = (point1[1] + point2[1]) // 2
        mid_point = (mid_col, mid_row)
        
        # 检查中间点是否在边缘
        if self.is_on_edge(mid_point) and mid_point not in self.forbidden_zones:
            return mid_point
            
        # 如果中间点不在边缘，尝试找附近的边缘点
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nearby = (mid_col + dx, mid_row + dy)
            if (0 <= nearby[0] < self.grid_cols and 0 <= nearby[1] < self.grid_rows and
                    self.is_on_edge(nearby) and nearby not in self.forbidden_zones):
                return nearby
                
        return None
        
    def find_nearby_edge_points(self, point, max_distance=3):
        """找到给定点附近的边缘点"""
        edge_points = []
        for col in range(max(0, point[0] - max_distance), min(self.grid_cols, point[0] + max_distance + 1)):
            for row in range(max(0, point[1] - max_distance), min(self.grid_rows, point[1] + max_distance + 1)):
                if (col, row) != point and self.is_on_edge((col, row)) and (col, row) not in self.forbidden_zones:
                    edge_points.append((col, row))
        return edge_points
        
    def manhattan_distance(self, p1, p2):
        """计算两点间的曼哈顿距离"""
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
        
    def path_length(self, start, end):
        """估计从起点到终点的路径长度"""
        path = self.get_shortest_path_between(start, end)
        return len(path) if path else float('inf')
        
    def find_path_to_edge_point(self, start, edge_point):
        """找到从起点到边缘点的路径"""
        # 首先尝试A*算法
        path = self.find_astar_path(start, edge_point)
        if path:
            return [start] + path
            
        # 如果失败，尝试灵活路径
        path = self.find_flexible_path(start, edge_point)
        if path:
            return [start] + path
            
        # 最后尝试相邻路径
        path = self.find_adjacent_path(start, edge_point)
        if path:
            return [start] + path
            
        return None

    def collect_optimized_end_paths(self, start_pos, unvisited, max_paths=5, max_depth=10):
        """收集从起点经过所有剩余未访问点并尽量靠近起降点的路径，最小化拐弯次数
        
        参数:
            start_pos: 起始位置
            unvisited: 未访问点集合
            max_paths: 最多收集的路径数量
            max_depth: 最大搜索深度（防止搜索空间过大）
            
        返回:
            一个包含可能路径的列表，每个路径是一系列点的序列
        """
        # 如果未访问点太多，选择部分靠近起降点的点
        points_to_visit = list(unvisited)
        if len(points_to_visit) > max_depth:
            # 按到起降点距离排序
            points_to_visit.sort(key=lambda p: self.manhattan_distance(p, self.red_point))
            points_to_visit = points_to_visit[:max_depth]
        
        # 获取当前方向
        current_direction = self.get_current_direction()
        
        # 使用修改版的优先搜索来找出多条拐弯最少的路径
        paths = []
        queue = [(0, start_pos, [start_pos], set(points_to_visit), current_direction, 0)]  # (优先级, 当前位置, 路径, 剩余点, 当前方向, 拐弯数)
        visited_states = set()  # (当前位置, 剩余点集合的哈希) - 避免重复状态
        
        while queue and len(paths) < max_paths:
            _, current, path, remaining, direction, turns = queue.pop(0)
            
            # 生成状态哈希，避免重复访问
            state_hash = (current, frozenset(remaining))
            if state_hash in visited_states:
                continue
            visited_states.add(state_hash)
            
            # 如果已经访问了所有选定的点，添加到结果
            if not remaining:
                paths.append((path[1:], turns))  # 不包括起始点，并记录拐弯数
                continue
            
            # 获取方向优先级，大幅优先考虑直行
            direction_priorities = self.get_direction_priorities(direction)
            
            # 尝试下一步移动
            neighbors = []
            for dx, dy in direction_priorities:
                next_pos = (current[0] + dx, current[1] + dy)
                
                if (0 <= next_pos[0] < self.grid_cols and 
                    0 <= next_pos[1] < self.grid_rows and
                    next_pos not in self.forbidden_zones and
                    next_pos not in path):  # 避免循环
                    
                    # 计算新方向和拐弯
                    new_direction = (dx, dy)
                    new_turns = turns
                    if direction is not None and new_direction != direction:
                        new_turns += 1
                    
                    # 计算到起降点的距离作为评分因素
                    distance_to_landing = self.manhattan_distance(next_pos, self.red_point)
                    
                    # 计算是否能减少未访问点
                    new_remaining = remaining.copy()
                    if next_pos in new_remaining:
                        new_remaining.remove(next_pos)
                    
                    # 计算评分：
                    # - 拐弯越少越好 (最主要，权重100)
                    # - 访问的点越多越好 (其次，权重10)
                    # - 距离起降点越近越好 (第三，权重1)
                    points_visited = len(remaining) - len(new_remaining)
                    score = -new_turns * 100 + points_visited * 10 - distance_to_landing
                    
                    # 优先级: 拐弯越少优先级越高
                    priority = new_turns
                    
                    neighbors.append((priority, score, next_pos, new_direction, new_turns, new_remaining))
            
            # 先按优先级(拐弯数)排序，再按评分排序
            neighbors.sort(key=lambda x: (-x[1], x[0]))
            
            # 选择最多3个最佳的下一步
            for i, (priority, _, next_pos, new_direction, new_turns, new_remaining) in enumerate(neighbors[:3]):
                queue.append((priority, next_pos, path + [next_pos], new_remaining, new_direction, new_turns))
        
        # 如果找到了路径，选择拐弯最少的路径
        if paths:
            # 按拐弯次数和路径长度排序
            paths.sort(key=lambda p: (p[1], len(p[0])))
            return [path[0] for path in paths]
        
        # 如果找不到完整的路径，尝试优化的贪心方法
        if not paths and points_to_visit:
            greedy_path = self.greedy_collect_path_minimize_turns(start_pos, points_to_visit)
            if greedy_path:
                paths.append(greedy_path)
        
        return [path for path in paths]
        
    def greedy_collect_path_minimize_turns(self, start_pos, points_to_visit):
        """使用贪心算法收集路径，优先选择最少拐弯并靠近起降点的路径"""
        path = []
        current = start_pos
        direction = self.get_current_direction()
        remaining = set(points_to_visit)
        
        while remaining:
            # 获取当前点可能的移动方向，优先考虑直线移动
            possible_moves = self.get_direction_priorities(direction)
            
            best_next = None
            best_score = float('-inf')
            best_direction = None
            
            # 考虑所有可能的移动方向
            for dx, dy in possible_moves:
                # 沿这个方向一直走，直到遇到障碍或边界
                temp_pos = current
                segment = []
                next_direction = (dx, dy)
                
                # 尝试沿直线移动，查找途径的未访问点
                while True:
                    next_pos = (temp_pos[0] + dx, temp_pos[1] + dy)
                    
                    # 检查边界和障碍
                    if not (0 <= next_pos[0] < self.grid_cols and 
                            0 <= next_pos[1] < self.grid_rows and
                            next_pos not in self.forbidden_zones):
                        break
                        
                    segment.append(next_pos)
                    temp_pos = next_pos
                    
                    # 如果找到3个点或更多，已经足够评估
                    if len(segment) >= 3:
                        break
                
                # 如果这个方向没有可走的点，跳过
                if not segment:
                    continue
                    
                # 计算这条路径访问的未访问点数量
                visited_points = [p for p in segment if p in remaining]
                points_count = len(visited_points)
                
                # 计算末端点到起降点的距离
                end_pos = segment[-1]
                distance_to_landing = self.manhattan_distance(end_pos, self.red_point)
                
                # 判断是否需要拐弯
                is_turn = direction is not None and next_direction != direction
                turn_penalty = 100 if is_turn else 0
                
                # 评分：访问点数×10 - 拐弯惩罚 - 到起降点距离
                score = points_count * 10 - turn_penalty - distance_to_landing * 0.5
                
                if score > best_score:
                    best_score = score
                    best_next = segment
                    best_direction = next_direction
            
            # 如果找到了下一步的最佳路径
            if best_next:
                # 添加路径段
                for pos in best_next:
                    path.append(pos)
                    if pos in remaining:
                        remaining.remove(pos)
                current = best_next[-1]
                direction = best_direction
            else:
                # 如果找不到直线路径，尝试最短路径到任一剩余点
                nearest_point = min(remaining, key=lambda p: self.manhattan_distance(current, p))
                shortest_path = self.get_shortest_path_between(current, nearest_point)
                
                if shortest_path:
                    for pos in shortest_path:
                        if pos != current:
                            path.append(pos)
                            if pos in remaining:
                                remaining.remove(pos)
                    current = shortest_path[-1]
                    
                    # 更新方向
                    if len(shortest_path) >= 2:
                        dx = shortest_path[-1][0] - shortest_path[-2][0]
                        dy = shortest_path[-1][1] - shortest_path[-2][1]
                        direction = (dx, dy) if dx != 0 or dy != 0 else direction
                else:
                    # 无法到达，移除该点
                    remaining.remove(nearest_point)
        
        return path
    
    def greedy_collect_path(self, start_pos, points_to_visit):
        """使用贪心算法收集路径，优先选择靠近起降点的方向"""
        path = []
        current = start_pos
        remaining = set(points_to_visit)
        
        while remaining:
            # 找出最近的下一个点
            candidates = []
            for point in remaining:
                # 计算到当前点的距离
                distance_to_current = self.manhattan_distance(current, point)
                # 计算到起降点的距离
                distance_to_landing = self.manhattan_distance(point, self.red_point)
                # 评分：距当前点越近越好，距起降点越近也越好
                score = distance_to_current + distance_to_landing * 0.5
                candidates.append((point, score))
            
            # 选择评分最低的点
            if candidates:
                candidates.sort(key=lambda x: x[1])
                next_point = candidates[0][0]
                
                # 找到一条到达next_point的路径
                segment = self.get_shortest_path_between(current, next_point)
                if segment:
                    # 添加路径段
                    for p in segment:
                        if p != current:
                            path.append(p)
                            if p in remaining:
                                remaining.remove(p)
                    current = segment[-1]
                else:
                    # 无法到达，移除该点
                    remaining.remove(next_point)
            else:
                break
        
        return path

    def simple_bfs_path(self, start, end):
        """简单的BFS路径搜索，作为备选算法"""
        from collections import deque
        
        if start == end:
            return [end]
            
        queue = deque([(start, [start])])
        visited = {start}
        
        while queue:
            current, path = queue.popleft()
            
            if current == end:
                return path[1:]  # 不包括起始点
                
            # 优先考虑水平垂直方向
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                next_col = current[0] + dx
                next_row = current[1] + dy
                next_point = (next_col, next_row)
                
                if (0 <= next_col < self.grid_cols and
                    0 <= next_row < self.grid_rows and
                    next_point not in visited and
                    next_point not in self.forbidden_zones):
                    
                    visited.add(next_point)
                    queue.append((next_point, path + [next_point]))
                    
        return None
        
    def is_on_edge(self, pos):
        """检查位置是否在网格边缘"""
        col, row = pos
        return (col == 0 or col == self.grid_cols - 1 or
                row == 0 or row == self.grid_rows - 1)
                
    def find_nearest_edge_point(self, start_pos):
        """寻找最近的边缘点"""
        col, row = start_pos
        edge_points = []

        # 添加四个边缘的候选点
        # 上边缘
        if row > 0:
            edge_points.append((col, 0))
        # 下边缘
        if row < self.grid_rows - 1:
            edge_points.append((col, self.grid_rows - 1))
        # 左边缘
        if col > 0:
            edge_points.append((0, row))
        # 右边缘
        if col < self.grid_cols - 1:
            edge_points.append((self.grid_cols - 1, row))

        # 过滤掉禁区点，找到最近的可达边缘点
        valid_edge_points = [p for p in edge_points if p not in self.forbidden_zones]

        if not valid_edge_points:
            return None

        # 返回距离最近的边缘点
        return min(valid_edge_points,
                  key=lambda p: abs(p[0] - col) + abs(p[1] - row))

    def plan_adjacent_survey(self, remaining_points):
        """第一阶段：使用严格相邻移动进行巡查"""
        unvisited = set(remaining_points)

        while unvisited:
            current_pos = self.waypoints[-1]

            # 寻找相邻的未访问点
            adjacent_point = self.find_adjacent_unvisited(current_pos, unvisited)

            if adjacent_point:
                # 直接移动到相邻点
                self.waypoints.append(adjacent_point)
                unvisited.remove(adjacent_point)
            else:
                # 寻找通过相邻移动能到达的最近未访问点
                path_to_unvisited = self.find_adjacent_path_to_unvisited(current_pos, unvisited)
                if path_to_unvisited:
                    # 添加路径上的所有点
                    for point in path_to_unvisited:
                        self.waypoints.append(point)
                        if point in unvisited:
                            unvisited.remove(point)
                else:
                    # 无法通过相邻移动到达任何未访问点，结束巡查阶段
                    break

        return len(unvisited) == 0

    def get_shortest_path_between(self, start, end):
        """获取两点间的最短路径"""
        path = self.find_flexible_path(start, end)
        if path:
            return path
        return self.find_adjacent_path(start, end)
        
    def get_shortest_path_to_landing(self, start):
        """获取到起降点的最短路径"""
        if start == self.red_point:
            return [self.red_point]

        if self.is_close_and_safe_to_landing(start):
            return [self.red_point]

        path = self.find_flexible_path(start, self.red_point)
        if path:
            return path

        return self.find_adjacent_path(start, self.red_point)
        
    def find_flexible_path(self, start, end):
        """寻找灵活移动路径（允许对角线等），但不能经过禁区顶点"""
        # 检查是否可以直接到达
        if not self.path_crosses_forbidden_zone(start, end):
            return [end]

        # 使用A*算法寻找避开禁区的路径
        from collections import deque
        import heapq

        open_set = [(0, start, [])]
        closed_set = set()

        while open_set:
            f_score, current, path = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)

            if current == end:
                return path + [end]

            # 检查8个方向的移动（包括对角线）
            directions = [
                (0, 1), (1, 0), (0, -1), (-1, 0),    # 上右下左
                (1, 1), (1, -1), (-1, 1), (-1, -1)   # 对角线
            ]

            for dx, dy in directions:
                next_col = current[0] + dx
                next_row = current[1] + dy
                next_point = (next_col, next_row)

                if (0 <= next_col < self.grid_cols and
                    0 <= next_row < self.grid_rows and
                    next_point not in closed_set and
                    next_point not in self.forbidden_zones and
                    not self.path_crosses_forbidden_zone(current, next_point)):

                    g_score = len(path) + 1
                    h_score = abs(next_point[0] - end[0]) + abs(next_point[1] - end[1])
                    f_score = g_score + h_score

                    new_path = path + [next_point] if next_point != end else path
                    heapq.heappush(open_set, (f_score, next_point, new_path))

        return None

    def calculate_total_distance(self):
        """计算总路程（实际米数）"""
        if len(self.waypoints) < 2:
            return 0.0

        total_distance = 0
        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]
            grid_distance = abs(p2[0] - p1[0]) + abs(p2[1] - p1[1])
            total_distance += grid_distance

        # 转换为实际米数
        return total_distance * self.cell_size
        
    def check_fallback_usage(self):
        """检查是否使用了相邻移动容错机制"""
        # 这里可以通过检查路径特征来判断是否使用了容错
        # 简单实现：检查返回路径中是否有只能通过相邻移动到达的路径段
        if self.return_path_start_index < 0 or len(self.waypoints) <= self.return_path_start_index + 1:
            return False

        # 检查返回路径中的移动是否都是相邻的
        return_waypoints = self.waypoints[self.return_path_start_index + 1:]
        if len(return_waypoints) < 2:
            return False

        # 如果返回路径中有非相邻移动，说明使用了灵活移动
        # 如果全部都是相邻移动，可能使用了容错机制
        all_adjacent = True
        for i in range(len(return_waypoints) - 1):
            p1 = return_waypoints[i]
            p2 = return_waypoints[i + 1]
            dx = abs(p2[0] - p1[0])
            dy = abs(p2[1] - p1[1])

            # 如果不是相邻移动（对角线或更远距离）
            if not ((dx == 1 and dy == 0) or (dx == 0 and dy == 1)):
                all_adjacent = False
                break

        # 如果返回路径较长且全部是相邻移动，可能使用了容错
        return all_adjacent and len(return_waypoints) > 3

    def add_shortest_return_to_landing(self, current_pos):
        """添加到起降点的最短路径"""
        if current_pos == self.red_point:
            return True

        return_path = self.get_shortest_path_to_landing(current_pos)
        if return_path:
            for point in return_path:
                self.waypoints.append(point)
            return True

        return False

    def path_crosses_forbidden_zone(self, start, end):
        """检查路径是否穿过禁区（包括禁区的边缘和顶角）"""
        start_col, start_row = start
        end_col, end_row = end

        # 如果起点或终点在禁区内，直接返回True
        if start in self.forbidden_zones or end in self.forbidden_zones:
            return True

        # 直接检查路径线段是否与任何禁区方块相交
        for forbidden_col, forbidden_row in self.forbidden_zones:
            # 检查路径是否与禁区方块相交
            if self.line_intersects_square(start_col, start_row, end_col, end_row,
                                         forbidden_col, forbidden_row):
                return True

        return False

    def line_intersects_square(self, x1, y1, x2, y2, square_col, square_row):
        """检查线段是否与方块相交（包括边缘和顶角）"""
        # 方块的边界（每个方块占据从(col, row)到(col+1, row+1)的区域）
        square_left = square_col
        square_right = square_col + 1
        square_top = square_row
        square_bottom = square_row + 1

        # 使用线段与矩形相交的算法
        # 检查线段是否与矩形的任何边相交

        # 如果线段的两个端点都在矩形的同一侧，则不相交
        if ((x1 < square_left and x2 < square_left) or
            (x1 > square_right and x2 > square_right) or
            (y1 < square_top and y2 < square_top) or
            (y1 > square_bottom and y2 > square_bottom)):
            return False

        # 如果线段的任一端点在矩形内，则相交
        if (square_left <= x1 <= square_right and square_top <= y1 <= square_bottom):
            return True
        if (square_left <= x2 <= square_right and square_top <= y2 <= square_bottom):
            return True

        # 检查线段是否与矩形的边相交
        # 使用参数方程检查相交
        dx = x2 - x1
        dy = y2 - y1

        if dx != 0:
            # 检查与左边和右边的相交
            t_left = (square_left - x1) / dx
            t_right = (square_right - x1) / dx

            for t in [t_left, t_right]:
                if 0 <= t <= 1:
                    y_intersect = y1 + t * dy
                    if square_top <= y_intersect <= square_bottom:
                        return True

        if dy != 0:
            # 检查与上边和下边的相交
            t_top = (square_top - y1) / dy
            t_bottom = (square_bottom - y1) / dy

            for t in [t_top, t_bottom]:
                if 0 <= t <= 1:
                    x_intersect = x1 + t * dx
                    if square_left <= x_intersect <= square_right:
                        return True

        # 检查是否从顶角通过
        # 定义禁飞区的四个顶角，并为每个顶角扩展检测范围
        corners = [
            (square_left, square_top),     # 左上角
            (square_right, square_top),    # 右上角
            (square_left, square_bottom),  # 左下角
            (square_right, square_bottom)  # 右下角
        ]
        
        # 检查路径是否经过顶角，增加安全距离
        for corner_x, corner_y in corners:
            # 计算到线段的距离
            if dx == 0 and dy == 0:  # 如果起点和终点是同一个点
                distance = ((corner_x - x1)**2 + (corner_y - y1)**2)**0.5
            else:
                # 计算点到线段的距离
                t = ((corner_x - x1) * dx + (corner_y - y1) * dy) / (dx**2 + dy**2)
                t = max(0, min(1, t))
                
                # 线段上最近点
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
                
                # 计算距离
                distance = ((corner_x - closest_x)**2 + (corner_y - closest_y)**2)**0.5
            
            # 如果距离小于等于0.5（增大安全距离，完全避开顶角）
            if distance <= 0.5:
                return True

        return False

class MapWidget(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.setMinimumSize(450, 320)  # 缩小地图控件的最小尺寸
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 计算绘制区域，缩小边距
        margin = 40
        draw_width = self.width() - 2 * margin
        draw_height = self.height() - 2 * margin
        
        # 计算网格大小
        cell_width = draw_width / self.parent.grid_cols
        cell_height = draw_height / self.parent.grid_rows
        
        # 创建已访问网格集合
        visited_grids = set()
        if hasattr(self.parent, 'drone_trajectory'):
            for point in self.parent.drone_trajectory:
                # 将轨迹点转换为网格坐标
                grid_col, grid_row = self.parent.global_to_grid_coords(point[0], point[1])
                if isinstance(grid_col, (int, float)) and isinstance(grid_row, (int, float)):
                    grid_col = int(round(grid_col))
                    grid_row = int(round(grid_row))
                    if 0 <= grid_col < self.parent.grid_cols and 0 <= grid_row < self.parent.grid_rows:
                        visited_grids.add((grid_col, grid_row))
        
        # 绘制经过的网格（浅橙色填充）
        painter.setBrush(QBrush(QColor(255, 200, 150, 180)))  # 浅橙色，增加不透明度为180
        painter.setPen(QPen(QColor(255, 160, 100, 200), 1))  # 橙色边框，增加不透明度
        for col, row in visited_grids:
            x = margin + col * cell_width
            y = margin + row * cell_height
            painter.drawRect(int(x), int(y), int(cell_width), int(cell_height))
        
        # 绘制网格
        painter.setPen(QPen(Qt.black, 1))
        for i in range(self.parent.grid_cols + 1):
            x = margin + i * cell_width
            painter.drawLine(int(x), margin, int(x), int(margin + draw_height))
            
        for i in range(self.parent.grid_rows + 1):
            y = margin + i * cell_height
            painter.drawLine(margin, int(y), int(margin + draw_width), int(y))
            
        # 绘制坐标标签
        painter.setPen(QPen(Qt.black, 1))
        font = QFont()
        font.setPointSize(9)  # 减小字体大小
        painter.setFont(font)
        
        # 列标签 (A1-A9) - 在底部水平显示
        for i in range(self.parent.grid_cols):
            x = margin + (i + 0.5) * cell_width - 8
            y = margin + draw_height + 15  # 减小与网格的间距
            painter.drawText(int(x), int(y), f"A{i + 1}")
            
        # 行标签 (B7-B1) - 在左侧垂直显示，从上到下为B7到B1
        for i in range(self.parent.grid_rows):
            x = margin - 20  # 减小与网格的间距
            y = margin + (i + 0.5) * cell_height + 5
            painter.drawText(int(x), int(y), f"B{7 - i}")
            
        # 绘制禁区
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.setPen(QPen(Qt.darkRed, 2))
        for col, row in self.parent.forbidden_zones:
            x = margin + col * cell_width
            y = margin + row * cell_height  # 直接使用row，因为索引0对应A9（顶部），索引8对应A1（底部）
            painter.drawRect(int(x), int(y), int(cell_width), int(cell_height))
        
        # 绘制检测到野生动物的方格 - 使用淡绿色填充，并添加动物图标
        if hasattr(self.parent, 'wildlife_detections') and self.parent.wildlife_detections:
            for grid_pos, data in self.parent.wildlife_detections.items():
                col, row = grid_pos
                x = margin + col * cell_width
                y = margin + row * cell_height
                
                # 检查当前网格是否已经被标记为访问过（避免重复绘制）
                grid_is_visited = (col, row) in visited_grids
                
                # 如果方格已经被标记为访问过，直接在上面覆盖动物检测信息
                # 否则使用淡绿色半透明填充表示有动物检测
                if not grid_is_visited:
                    painter.setBrush(QBrush(QColor(100, 200, 100, 80), Qt.SolidPattern))
                    painter.setPen(QPen(QColor(50, 150, 50), 2))
                    painter.drawRect(int(x), int(y), int(cell_width), int(cell_height))
                
                # 在方格中央添加动物数量标记
                center_x = x + cell_width / 2
                center_y = y + cell_height / 2
                
                # 绘制一个圆形背景
                icon_size = min(cell_width, cell_height) * 0.5
                painter.setBrush(QBrush(QColor(50, 150, 50), Qt.SolidPattern))  # 浅绿色背景，和以前一样
                painter.setPen(QPen(QColor(255, 255, 255), 1))
                painter.drawEllipse(int(center_x - icon_size/2), int(center_y - icon_size/2), 
                                   int(icon_size), int(icon_size))
                
                # 在方格内显示动物总数
                total_animals = sum(data['animals'].values())
                painter.setPen(QPen(Qt.white, 1))
                font = QFont()
                font.setBold(True)
                font.setPointSize(12)  # 增大字体
                painter.setFont(font)
                text_width = painter.fontMetrics().width(str(total_animals))
                painter.drawText(int(center_x - text_width/2), int(center_y + 5), str(total_animals))
            
        # 绘制红点(起降点)
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.setPen(QPen(Qt.darkRed, 3))
        red_col, red_row = self.parent.red_point
        x = margin + (red_col + 0.5) * cell_width
        y = margin + (red_row + 0.5) * cell_height  # 直接使用red_row
        
        # 在调试信息中输出红点位置的坐标信息，便于确认
        if hasattr(self.parent, 'origin_col') and hasattr(self.parent, 'origin_row'):
            # 确保红点位置就是原点(0,0,0)对应的网格位置
            self.parent.origin_col = red_col
            self.parent.origin_row = red_row
        
        # 绘制红点
        painter.drawEllipse(int(x-10), int(y-10), 20, 20)
        
        # 绘制航线
        if len(self.parent.waypoints) > 1:
            for i in range(len(self.parent.waypoints) - 1):
                col1, row1 = self.parent.waypoints[i]
                col2, row2 = self.parent.waypoints[i + 1]
                
                x1 = margin + (col1 + 0.5) * cell_width
                y1 = margin + (row1 + 0.5) * cell_height  # 直接使用row1
                x2 = margin + (col2 + 0.5) * cell_width
                y2 = margin + (row2 + 0.5) * cell_height  # 直接使用row2
                
                # 判断是巡查路径还是返回路径
                if (self.parent.return_path_start_index >= 0 and 
                    i >= self.parent.return_path_start_index):
                    # 返回路径用紫色
                    painter.setPen(QPen(QColor(128, 0, 128), 3))  # 紫色
                else:
                    # 巡查路径用绿色
                    painter.setPen(QPen(Qt.green, 3))
                
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))
                
                # 为巡查路径绘制箭头
                if (self.parent.return_path_start_index < 0 or 
                    i < self.parent.return_path_start_index):
                    self.draw_arrow(painter, x1, y1, x2, y2)
                
        # 绘制航点
        painter.setBrush(QBrush(Qt.green, Qt.SolidPattern))
        painter.setPen(QPen(Qt.darkGreen, 2))
        for i, (col, row) in enumerate(self.parent.waypoints):
            x = margin + (col + 0.5) * cell_width
            y = margin + (row + 0.5) * cell_height  # 直接使用row
            painter.drawEllipse(int(x-5), int(y-5), 10, 10)
            
            # 绘制航点编号
            painter.setPen(QPen(Qt.black, 1))
            painter.drawText(int(x+8), int(y+5), str(i+1))
            painter.setPen(QPen(Qt.darkGreen, 2))
            
        # 绘制无人机轨迹
        if len(self.parent.drone_trajectory) > 1:
            # 使用更明显的橘色轨迹
            painter.setPen(QPen(QColor(255, 140, 0, 200), 2.5))  # 更不透明的橘色，增加线宽
            
            prev_point = None
            for point in self.parent.drone_trajectory:
                # 将全局坐标转换为网格坐标
                grid_col, grid_row = self.parent.global_to_grid_coords(point[0], point[1])
                
                # 转换为屏幕坐标
                screen_x = margin + (grid_col + 0.5) * cell_width
                screen_y = margin + (grid_row + 0.5) * cell_height
                
                if prev_point is not None:
                    painter.drawLine(int(prev_point[0]), int(prev_point[1]), int(screen_x), int(screen_y))
                
                prev_point = (screen_x, screen_y)
            
        # 绘制无人机当前位置（橘色圆点）
        if hasattr(self.parent, 'drone_position'):
            # 将全局坐标转换为网格坐标
            drone_x, drone_y, drone_z = self.parent.drone_position
            
            # 处理特殊情况：如果无人机位于原点(0,0,0)附近
            if abs(drone_x) < 0.01 and abs(drone_y) < 0.01:
                # 使用红点位置但稍微偏移一点，确保橙色点能够显示在红点上方
                grid_col, grid_row = self.parent.red_point
                self.parent.last_displayed_grid_position = (grid_col, grid_row)
            else:
                # 直接使用最新的坐标，不做任何延迟或平滑处理
                # 这样能确保橘色点始终显示在无人机的实际位置
                grid_col, grid_row = self.parent.global_to_grid_coords(drone_x, drone_y)
                
                # 无条件更新为新位置
                self.parent.last_displayed_grid_position = (grid_col, grid_row)
                
                # 更新距离标签显示（如果需要）
                if hasattr(self.parent, 'last_distance_to_center'):
                    distance = self.parent.last_distance_to_center
                    
                    if hasattr(self.parent, 'distance_to_center_label'):
                        status_text = f"距离方格中心: {distance:.2f}m"
                        if distance < 0.1:
                            status_text += " (已到达)"
                            self.parent.distance_to_center_label.setStyleSheet("QLabel { color: #4CAF50; font-weight: bold; font-family: 'Monospace', 'Courier New'; }")  # 绿色加粗
                        else:
                            self.parent.distance_to_center_label.setStyleSheet("QLabel { color: #FF8C00; font-family: 'Monospace', 'Courier New'; }")  # 橘色
                        self.parent.distance_to_center_label.setText(status_text)
            
            # 转换为屏幕坐标
            screen_x = margin + (grid_col + 0.5) * cell_width
            screen_y = margin + (grid_row + 0.5) * cell_height
            
            # 绘制无人机位置（橘色圆点）
            painter.setBrush(QBrush(QColor(255, 140, 0), Qt.SolidPattern))  # 橘色
            painter.setPen(QPen(QColor(204, 85, 0), 2))  # 深橘色边框
            painter.drawEllipse(int(screen_x-8), int(screen_y-8), 16, 16)
    
    def draw_arrow(self, painter, x1, y1, x2, y2):
        """在线段上绘制箭头"""
        # 计算线段的方向向量
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        
        if length == 0:
            return
            
        # 单位方向向量
        ux = dx / length
        uy = dy / length
        
        # 箭头参数
        arrow_length = 15
        arrow_angle = math.pi / 6  # 30度
        
        # 箭头位置（线段中点）
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        
        # 计算箭头的两个端点
        cos_angle = math.cos(arrow_angle)
        sin_angle = math.sin(arrow_angle)
        
        # 箭头左端点
        left_x = mid_x - arrow_length * (ux * cos_angle + uy * sin_angle)
        left_y = mid_y - arrow_length * (uy * cos_angle - ux * sin_angle)
        
        # 箭头右端点
        right_x = mid_x - arrow_length * (ux * cos_angle - uy * sin_angle)
        right_y = mid_y - arrow_length * (uy * cos_angle + ux * sin_angle)
        
        # 绘制箭头线条（确保使用整数坐标）
        arrow_points = QPolygon()
        arrow_points.append(QPoint(int(mid_x), int(mid_y)))
        arrow_points.append(QPoint(int(left_x), int(left_y)))
        arrow_points.append(QPoint(int(right_x), int(right_y)))
        
        # 填充箭头
        painter.setBrush(QBrush(Qt.green, Qt.SolidPattern))
        painter.drawPolygon(arrow_points)
        
    def mousePressEvent(self, event):
        """处理鼠标点击事件"""
        if event.button() == Qt.LeftButton:
            # 计算点击位置对应的网格坐标
            margin = 50
            draw_width = self.width() - 2 * margin
            draw_height = self.height() - 2 * margin
            
            cell_width = draw_width / self.parent.grid_cols
            cell_height = draw_height / self.parent.grid_rows
            
            # 转换鼠标坐标到网格坐标
            x = event.x() - margin
            y = event.y() - margin
            
            if 0 <= x <= draw_width and 0 <= y <= draw_height:
                col = int(x / cell_width)
                row = int(y / cell_height)  # 直接使用y坐标，因为A9在顶部（索引0），A1在底部（索引8）
                
                # 确保坐标在有效范围内
                if 0 <= col < self.parent.grid_cols and 0 <= row < self.parent.grid_rows:
                    # 添加禁区
                    self.parent.add_forbidden_zone(col, row)
                    self.update()  # 重新绘制地图
             
if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    try:
        window = WildlifeSurveyStation()
        window.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
