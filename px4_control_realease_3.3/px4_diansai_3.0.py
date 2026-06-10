#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
文件名：px4_diansai_3.0.py
功能：电赛航点跟随任务 - 优化版本
说明：读取mission_log目录下最新的waypoints_*.yaml文件，并控制无人机按航点飞行
      需要收到地面站发送的起飞指令才开始执行任务
      第一个航点5s用来起飞到指定高度
      连续超过3个以上的航点使用端到端控制忽略中间，时间为航点数量+3.2s
      3个及以内的航点使用采样点飞行控制函数
      返航过程中直接使用端到端飞行
      距离起降点只剩下两个航点距离时省略其他航点，直接返回0,0,0
'''

import rospy
import os
import yaml
import re
import glob
import time
import math
from action_t import Action_t
from std_msgs.msg import Int32  # 导入Int32消息类型用于接收起飞指令

def is_ros_initialized():
    """检查ROS是否已经初始化"""
    try:
        # 尝试获取ROS时间，如果ROS未初始化会抛出异常
        rospy.get_rostime()
        return True
    except:
        return False

class WaypointMission:
    def __init__(self, drone):
        """
        初始化航点任务控制器
        参数：
        - drone: Action_t对象
        """
        self.drone = drone
        self.waypoints = []
        self.mission_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mission_log')
        
        # 确保mission_log目录存在
        if not os.path.exists(self.mission_dir):
            os.makedirs(self.mission_dir)
            rospy.logwarn(f"创建了航点目录：{self.mission_dir}")
        
        # 任务执行标志
        self.mission_active = False
        self.mission_received = False
        
        # 起降点坐标（默认为0,0,0）
        self.home_position = [0.0, 0.0, 0.0]
        
        # 创建订阅器，监听来自地面站的起飞指令 - 确保ROS节点已初始化
        try:
            self.command_sub = rospy.Subscriber('/mission_command', Int32, self.command_callback)
            rospy.loginfo("等待地面站发送起飞指令...")
        except Exception as e:
            rospy.logerr(f"创建命令订阅器失败: {e}，请确保ROS已正确初始化")
    
    def command_callback(self, msg):
        """
        处理来自地面站的命令
        msg.data: 1表示执行任务，0表示停止任务
        """
        command = msg.data
        self.mission_received = True
        
        if command == 1 and not self.mission_active:
            rospy.loginfo("收到起飞指令，准备读取最新航点文件并执行任务")
            # 收到起飞指令后，重新读取最新的航点文件
            waypoint_file = self.find_latest_waypoint_file()
            if waypoint_file and self.load_waypoints(waypoint_file):
                self.mission_active = True
                rospy.loginfo("航点文件加载成功，任务激活")
            else:
                rospy.logerr("航点文件加载失败，任务未激活")
        elif command == 0 and self.mission_active:
            rospy.loginfo("收到停止指令，任务已停止")
            self.mission_active = False
            # 紧急停止处理
            self.drone.stop_all_threads()
    
    def find_latest_waypoint_file(self):
        """
        查找mission_log目录下后缀数字最大的yaml文件
        返回：文件路径或None
        """
        waypoint_files = glob.glob(os.path.join(self.mission_dir, 'waypoints_*.yaml'))
        if not waypoint_files:
            rospy.logerr("未找到任何航点文件")
            return None
        
        # 提取文件名中的数字后缀并排序
        sorted_files = []
        for filepath in waypoint_files:
            filename = os.path.basename(filepath)
            match = re.search(r'waypoints_(\d+)\.yaml', filename)
            if match:
                number = int(match.group(1))
                sorted_files.append((filepath, number))
        
        # 按数字大小排序
        sorted_files.sort(key=lambda x: x[1], reverse=True)
        
        if sorted_files:
            latest_file = sorted_files[0][0]
            rospy.loginfo(f"找到最新航点文件：{latest_file} (序号: {sorted_files[0][1]})")
            return latest_file
        else:
            rospy.logerr("未找到有效的航点文件")
            return None
    
    def load_waypoints(self, filepath):
        """
        从yaml文件加载航点数据
        参数：
        - filepath: yaml文件路径
        返回：是否成功加载
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' not in data:
                rospy.logerr("航点文件格式错误：缺少waypoints字段")
                return False
            
            self.waypoints = data['waypoints']
            rospy.loginfo(f"成功加载{len(self.waypoints)}个航点")
            
            # 打印前三个航点信息（如果存在）
            for i, wp in enumerate(self.waypoints[:3]):
                pos = wp.get('global_position', {})
                coord = wp.get('coordinate', 'N/A')
                action = wp.get('action', 'N/A')
                rospy.loginfo(f"航点{i+1}: {coord} ({pos.get('x', 'N/A')}, {pos.get('y', 'N/A')}, {pos.get('z', 'N/A')}) - {action}")
            
            if len(self.waypoints) > 3:
                rospy.loginfo("...")
            
            return True
        except Exception as e:
            rospy.logerr(f"加载航点文件失败: {e}")
            return False
    
    def calculate_distance(self, pos1, pos2):
        """
        计算两个位置之间的欧几里得距离
        参数：
        - pos1: 位置1 [x, y, z]
        - pos2: 位置2 [x, y, z]
        返回：距离值
        """
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        dz = pos1[2] - pos2[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def should_return_home(self, current_index):
        """
        判断是否应该直接返航
        如果当前航点距离起降点只剩下两个航点的距离，则直接返航
        参数：
        - current_index: 当前航点索引
        返回：是否应该直接返航
        """
        if current_index >= len(self.waypoints):
            return True
        
        # 获取当前航点位置
        current_wp = self.waypoints[current_index]
        current_pos = current_wp.get('global_position', {})
        current_position = [current_pos.get('x', 0.0), current_pos.get('y', 0.0), current_pos.get('z', 0.0)]
        
        # 计算到起降点的距离
        distance_to_home = self.calculate_distance(current_position, self.home_position)
        
        # 计算两个航点的平均距离作为阈值
        if len(self.waypoints) >= 2:
            # 计算前两个航点之间的距离作为参考
            wp1_pos = self.waypoints[0].get('global_position', {})
            wp2_pos = self.waypoints[1].get('global_position', {})
            wp1_position = [wp1_pos.get('x', 0.0), wp1_pos.get('y', 0.0), wp1_pos.get('z', 0.0)]
            wp2_position = [wp2_pos.get('x', 0.0), wp2_pos.get('y', 0.0), wp2_pos.get('z', 0.0)]
            reference_distance = self.calculate_distance(wp1_position, wp2_position)
            
            # 如果距离起降点小于两个航点距离，则直接返航
            threshold_distance = reference_distance * 2.0
            rospy.loginfo(f"当前距离起降点: {distance_to_home:.2f}m, 阈值: {threshold_distance:.2f}m")
            
            if distance_to_home <= threshold_distance:
                rospy.loginfo("距离起降点较近，准备直接返航")
                return True
        
        return False
    
    def calculate_straight_line_waypoints(self, start_index):
        """
        计算从指定索引开始的直线段中的航点数量
        返回直线段中的航点数量和直线段结束的索引
        """
        if start_index >= len(self.waypoints) - 1:
            return 1, start_index
        
        # 获取起始航点和下一个航点的位置
        start_waypoint = self.waypoints[start_index]
        next_waypoint = self.waypoints[start_index + 1]
        
        start_pos = start_waypoint.get('global_position', {})
        next_pos = next_waypoint.get('global_position', {})
        
        # 计算初始方向向量
        dx = next_pos.get('x', 0) - start_pos.get('x', 0)
        dy = next_pos.get('y', 0) - start_pos.get('y', 0)
        
        # 如果初始方向向量为零，返回单个航点
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 1, start_index
        
        # 归一化方向向量
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            dx /= length
            dy /= length
        
        waypoint_count = 2  # 至少包含起始点和下一个点
        current_index = start_index + 1
        
        # 继续检查后续航点是否在同一直线上
        for i in range(start_index + 2, len(self.waypoints)):
            current_waypoint = self.waypoints[current_index]
            check_waypoint = self.waypoints[i]
            
            current_pos = current_waypoint.get('global_position', {})
            check_pos = check_waypoint.get('global_position', {})
            
            # 计算当前段的方向向量
            check_dx = check_pos.get('x', 0) - current_pos.get('x', 0)
            check_dy = check_pos.get('y', 0) - current_pos.get('y', 0)
            
            # 归一化当前段方向向量
            check_length = math.sqrt(check_dx*check_dx + check_dy*check_dy)
            if check_length > 0:
                check_dx /= check_length
                check_dy /= check_length
            
            # 计算方向向量的点积，判断是否在同一直线上
            # 点积接近1表示方向相同（同一直线）
            dot_product = dx * check_dx + dy * check_dy
            
            # 设置容差，允许小幅度的方向变化
            if dot_product > 0.95:  # 约18度的容差
                waypoint_count += 1
                current_index = i
            else:
                # 方向发生明显变化，直线段结束
                break
        
        return waypoint_count, current_index
    
    def execute_mission(self):
        """
        执行航点任务
        """
        if not self.waypoints:
            rospy.logerr("没有航点可执行")
            return False
        
        try:
            # 首先解锁无人机
            self.unlock_drone()
            
            # 分析航点数量，决定飞行策略
            total_waypoints = len(self.waypoints)
            rospy.loginfo(f"总共{total_waypoints}个航点，开始执行任务")
            
            # 遍历每个航点（使用while循环以便正确处理航点跳跃）
            i = 0
            while i < len(self.waypoints):
                if rospy.is_shutdown() or not self.mission_active:
                    rospy.logwarn("任务被中断")
                    break
                
                waypoint = self.waypoints[i]
                
                # 检查是否应该直接返航
                if i > 0 and self.should_return_home(i):
                    rospy.loginfo("满足返航条件，直接返回起降点")
                    self.return_home()
                    break
                
                # 获取航点位置
                pos = waypoint.get('global_position', {})
                x = pos.get('x', 0.0)
                y = pos.get('y', 0.0)
                z = pos.get('z', 0.0)
                
                # 获取航点信息
                wp_id = waypoint.get('id', i+1)
                coord = waypoint.get('coordinate', 'Unknown')
                action = waypoint.get('action', 'survey')
                
                rospy.loginfo(f"执行航点 {wp_id}/{total_waypoints}: {coord} ({x}, {y}, {z}) - {action}")
                
                # 判断是否是最后一个航点
                is_last_waypoint = (i == total_waypoints - 1)
                # 判断是否是第一个航点
                is_first_waypoint = (i == 0)
                
                if is_last_waypoint:
                    # 如果是最后一个航点，直接执行自动降落
                    rospy.loginfo(f"到达最后一个航点，执行自动降落")
                    self.land_drone()
                    i += 1  # 移动到下一个航点
                elif action == 'takeoff' or is_first_waypoint:
                    # 起飞点 - 第一次起飞用5秒
                    duration = 5.0
                    self.fly_to_waypoint_position_control(x, y, z, f"起飞到航点{wp_id}", duration=duration)
                    i += 1  # 移动到下一个航点
                elif action == 'land':
                    # 降落点（中间的降落点仍按原逻辑处理）
                    self.land_drone(x, y)
                    i += 1  # 移动到下一个航点
                else:
                    # 普通巡航点 - 根据当前直线段中的航点数量决定控制策略
                    straight_line_count, end_index = self.calculate_straight_line_waypoints(i)
                    rospy.loginfo(f"当前直线段包含{straight_line_count}个航点（从航点{i+1}到航点{end_index+1}）")
                    
                    if straight_line_count > 3:
                        # 直线段超过3个航点，使用端到端控制，直接飞到直线段的最后一个航点
                        end_waypoint = self.waypoints[end_index]
                        end_pos = end_waypoint.get('global_position', {})
                        end_x = end_pos.get('x', 0.0)
                        end_y = end_pos.get('y', 0.0)
                        end_z = end_pos.get('z', 0.0)
                        duration = straight_line_count + 3.2
                        rospy.loginfo(f"使用端到端控制，直接飞向直线段末端航点{end_index+1}，航点数{straight_line_count}，飞行时间{duration}秒")
                        self.fly_to_waypoint_position_control(end_x, end_y, end_z, f"端到端飞向航点{end_index+1}", duration=duration, local_v=0.5)
                        # 跳过直线段中的中间航点，直接到达直线段末端
                        # 记录跳过的航点
                        for skip_i in range(i + 1, end_index + 1):
                            if skip_i < len(self.waypoints):
                                skip_waypoint = self.waypoints[skip_i]
                                skip_wp_id = skip_waypoint.get('id', skip_i+1)
                                rospy.loginfo(f"跳过中间航点{skip_wp_id}（端到端控制）")
                        
                        # 设置索引到直线段末端的下一个航点
                        i = end_index + 1
                        
                        # 如果还有下一个航点，检查是否需要转弯控制
                        if i < len(self.waypoints):
                            next_waypoint = self.waypoints[i]
                            next_pos = next_waypoint.get('global_position', {})
                            next_x = next_pos.get('x', 0.0)
                            next_y = next_pos.get('y', 0.0)
                            next_z = next_pos.get('z', 0.0)
                            next_wp_id = next_waypoint.get('id', i+1)
                            next_action = next_waypoint.get('action', 'survey')
                            
                            # 判断下一个航点是否是最后一个航点或特殊动作航点
                            is_next_last = (i == len(self.waypoints) - 1)
                            
                            if not is_next_last and next_action not in ['takeoff', 'land']:
                                # 如果下一个航点不是最后一个且不是特殊动作，使用位置控制确保转弯精度
                                rospy.loginfo(f"端到端控制后，使用位置控制到转弯航点{next_wp_id}确保精度")
                                self.fly_to_waypoint_position_control(next_x, next_y, next_z, f"转弯飞向航点{next_wp_id}", duration=1.3, local_v=0.25)
                                i += 1  # 移动到转弯航点的下一个航点
                            # 如果是最后一个航点或特殊动作，保持当前索引，让主循环正常处理
                        # 如果没有下一个航点，i会超出范围，主循环会自然结束
                    else:
                        # 直线段3个及以内航点，使用位置控制
                        rospy.loginfo(f"使用位置控制飞向航点{wp_id}")
                        self.fly_to_waypoint_position_control(x, y, z, f"飞向航点{wp_id}", duration=1.3, local_v=0.25)
                        i += 1  # 移动到下一个航点
            
            rospy.loginfo("所有航点执行完成，任务结束")
            return True
        except Exception as e:
            rospy.logerr(f"执行航点任务失败: {e}")
            return False
    
    def unlock_drone(self):
        """解锁无人机"""
        rospy.loginfo("解锁无人机")
        self.drone.unlock(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def fly_to_waypoint_position_control(self, x, y, z, description="", duration=2.0, local_v=0.5):
        """
        使用位置控制飞到指定航点
        参数：
        - x, y, z: 目标位置坐标
        - description: 描述信息
        - duration: 飞行时间(秒)，默认2.0秒
        - local_v: 飞行速度，默认0.5
        """
        if description:
            rospy.loginfo(description)
            
        # 输出飞行时间和速度信息
        rospy.loginfo(f"位置控制飞行时间: {duration}秒, 速度: {local_v}")
        
        # 使用位置控制指令（通过frame参数设置速度）
        frame_with_speed = f"local_v={local_v}"
        self.drone.send_position_x_y_z_t_frame(x, y, z, duration, frame=frame_with_speed, use_thread=True)
        
        # 等待控制完成
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        current_xyz = self.drone.get_current_xyz()
        rospy.loginfo(f"到达航点: 当前位置 ({current_xyz[0]:.2f}, {current_xyz[1]:.2f}, {current_xyz[2]:.2f})")
    

    
    def return_home(self):
        """
        返航到起降点(0,0,0)，使用端到端飞行
        """
        rospy.loginfo("开始返航到起降点(0,0,0)")
        
        # 使用端到端控制直接飞回起降点，设置目标点的时候直接使用4s
        self.drone.send_position_x_y_z_t_frame(
            self.home_position[0], 
            self.home_position[1], 
            self.home_position[2], 
            4.0,  # 4秒飞行时间
            frame="local", 
            use_thread=True
        )
        
        # 等待控制完成
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo("返航完成，准备强制上锁")
        
        # 直接使用action_t.py里面的强制上锁lock函数
        self.drone.lock(use_thread=True)
        
        # 等待上锁完成
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo("强制上锁完成")
    
    def land_drone(self, x=None, y=None):
        """
        控制无人机降落
        参数：
        - x, y: 可选的降落位置坐标
        """
        rospy.loginfo("执行降落")
        
        if x is not None and y is not None:
            # 先飞到降落位置上方，用2秒时间
            current_xyz = self.drone.get_current_xyz()
            rospy.loginfo(f"飞行时间: 2.0秒")
            self.drone.send_position_x_y_z_t_frame(x, y, current_xyz[2], 2.0, frame="local", use_thread=True)
            # 等待控制完成
            while not self.drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
        
        # 执行降落
        self.drone.land_auto(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo("降落完成")
    
    def wait_for_command(self, timeout=None):
        """
        等待地面站发送起飞指令
        参数：
        - timeout: 超时时间(秒)，None表示无限等待
        返回：是否收到起飞指令
        """
        rospy.loginfo("等待地面站发送起飞指令...")
        
        # 检查ROS是否已初始化
        if not is_ros_initialized():
            rospy.logerr("ROS未初始化，无法等待命令")
            return False
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.mission_active:
                return True
            
            if timeout is not None:
                # 确保ROS已初始化后再访问时间
                current_time = rospy.Time.now()
                elapsed = (current_time - start_time).to_sec()
                if elapsed > timeout:
                    rospy.logwarn(f"等待起飞指令超时({timeout}秒)")
                    return False
            
            rospy.sleep(0.2)  # 休眠200ms，避免CPU占用过高
        
        return False
    
    def run(self, wait_for_command_timeout=None, test_mode=False):
        """
        运行航点任务
        参数：
        - wait_for_command_timeout: 等待命令的超时时间（秒），None表示无限等待
        - test_mode: 是否为测试模式，若为True，则不等待命令直接执行
        返回：是否成功执行任务
        """
        # 等待地面站发送起飞指令（除非处于测试模式）
        if not test_mode:
            if not self.wait_for_command(wait_for_command_timeout):
                rospy.logwarn("未收到起飞指令，任务终止")
                return False
        else:
            rospy.loginfo("测试模式：跳过等待命令阶段，读取最新航点文件")
            # 测试模式下手动读取最新航点文件
            waypoint_file = self.find_latest_waypoint_file()
            if not waypoint_file or not self.load_waypoints(waypoint_file):
                rospy.logerr("测试模式下航点文件加载失败")
                return False
            self.mission_active = True
        
        # 执行航点任务（航点文件已在command_callback或测试模式中加载）
        return self.execute_mission()


def main():
    """主函数"""
    # 不再需要初始化ROS节点，因为Action_t会自行初始化
    # rospy.init_node('px4_diansai', anonymous=True)
    
    # 检查是否开启测试模式
    import sys
    test_mode = '--test' in sys.argv
    
    # 创建无人机控制对象（使用仿真模式）
    drone = Action_t("sim_nolog")
    mission = WaypointMission(drone)
    
    try:
        # 使用测试模式运行任务
        if test_mode:
            rospy.loginfo("以测试模式启动，不等待地面站指令")
        
        result = mission.run(test_mode=test_mode)
        if result:
            rospy.loginfo("航点任务执行成功")
        else:
            rospy.logerr("航点任务执行失败")
    
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
        # 确保中断时安全降落
        drone.land_auto()
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")
        # 确保出错时安全降落
        drone.land_auto()
    finally:
        # 停止所有控制线程
        drone.stop_all_threads()
        rospy.loginfo("程序结束")


if __name__ == '__main__':
    main()