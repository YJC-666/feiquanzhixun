di#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
文件名：px4_diansai_3.2.py
功能：电赛航点跟随任务（高级版）
说明：读取mission_log目录下最新的waypoints_*.yaml文件，并控制无人机按航点飞行
      需要收到地面站发送的起飞指令才开始执行任务
      具有端到端飞行能力，可以直接跳过中间航点
      实现了45°角降落返航功能
'''

import rospy
import os
import yaml
import re
import glob
import time
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
            rospy.loginfo("收到起飞指令，准备执行航点任务")
            self.mission_active = True
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
    
    def unlock_drone(self):
        """解锁无人机"""
        rospy.loginfo("解锁无人机")
        self.drone.unlock(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def calculate_flight_duration(self, waypoints_count, is_first_waypoint=False, is_direct_return=False):
        """
        计算飞行时间
        - 第一个航点固定5秒(用于起飞到指定高度)
        - 如果是直接返回，使用4秒
        - 如果是连续多个航点的端到端飞行，时间为航点数量+2.5秒
        - 否则使用2秒
        """
        if is_first_waypoint:
            rospy.loginfo("第一个航点，使用5秒起飞时间")
            return 5.0
        elif is_direct_return:
            rospy.loginfo("直接返航，使用4秒飞行时间")
            return 4.0
        elif waypoints_count > 3:  # 超过3个航点使用端到端控制
            duration = waypoints_count + 2.5
            rospy.loginfo(f"端到端飞行跳过{waypoints_count}个航点，使用{duration}秒飞行时间")
            return duration
        else:
            rospy.loginfo("普通航点，使用2秒飞行时间")
            return 2.0
    
    def fly_to_waypoint(self, x, y, z, description="", duration=2.0):
        """
        控制无人机飞到指定航点
        参数：
        - x, y, z: 目标位置坐标
        - description: 描述信息
        - duration: 飞行时间(秒)，默认2.0秒
        """
        if description:
            rospy.loginfo(description)
            
        # 输出飞行时间和目标信息
        rospy.loginfo(f"飞行目标: ({x:.3f}, {y:.3f}, {z:.3f})，飞行时间: {duration:.1f}秒")
        
        # 记录起始位置
        start_xyz = self.drone.get_current_xyz()
        rospy.loginfo(f"起始位置: ({start_xyz[0]:.3f}, {start_xyz[1]:.3f}, {start_xyz[2]:.3f})")
        
        # 使用位置控制指令
        self.drone._send_position_x_y_z_t_frame(x, y, z, duration, frame="local")
        
        # 等待控制完成，并每秒输出当前位置
        start_time = time.time()
        while not self.drone.control_complete() and not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if int(elapsed) % 1 == 0 and int(elapsed) > 0 and int(elapsed) <= duration:
                current_xyz = self.drone.get_current_xyz()
                progress = min(elapsed / duration * 100, 99)
                rospy.loginfo(f"飞行进度: {progress:.0f}% - 当前位置: ({current_xyz[0]:.3f}, {current_xyz[1]:.3f}, {current_xyz[2]:.3f})")
            rospy.sleep(0.1)
        
        # 输出最终位置和飞行结果
        current_xyz = self.drone.get_current_xyz()
        dist_error = ((current_xyz[0]-x)**2 + (current_xyz[1]-y)**2 + (current_xyz[2]-z)**2)**0.5
        rospy.loginfo(f"到达航点: 当前位置 ({current_xyz[0]:.3f}, {current_xyz[1]:.3f}, {current_xyz[2]:.3f}), 误差: {dist_error:.3f}米")
    
    def fly_points_sampling(self, x, y, z, description=""):
        """
        使用采样点飞行到指定位置
        参数：
        - x, y, z: 目标位置坐标
        - description: 描述信息
        """
        if description:
            rospy.loginfo(description)
        
        rospy.loginfo(f"采样点飞行到 ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # 使用采样点飞行方法（调整参数）
        self.drone.control_points_x_y_z_stepsize_frame_tolerance_axisTolerance(
            x, y, z, 
            stepsize=0.2,  # 增大采样步长
            frame="local",  # 本地坐标系
            tolerance=0.1,  # 增大目标位置容差
            axisTolerance=0.15,  # 增大轴容差
            use_thread=True
        )
        
        # 等待控制完成
        start_time = time.time()
        while not self.drone.control_complete() and not rospy.is_shutdown():
            # 每5秒输出一次当前位置信息
            if int(time.time() - start_time) % 5 == 0 and int(time.time() - start_time) > 0:
                current_xyz = self.drone.get_current_xyz()
                rospy.loginfo(f"飞行中: 当前位置 ({current_xyz[0]:.2f}, {current_xyz[1]:.2f}, {current_xyz[2]:.2f})")
            rospy.sleep(0.1)
            
        current_xyz = self.drone.get_current_xyz()
        rospy.loginfo(f"到达采样点: 当前位置 ({current_xyz[0]:.2f}, {current_xyz[1]:.2f}, {current_xyz[2]:.2f})")
    
    def land_drone(self):
        """控制无人机降落并上锁"""
        rospy.loginfo("执行降落和上锁")
        
        # 执行降落
        self.drone.land_auto(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # 执行上锁
        self.drone.lock(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rospy.loginfo("降落和上锁完成")
    
    def execute_mission(self):
        """
        执行航点任务
        """
        if not self.waypoints:
            rospy.logerr("没有航点可执行")
            return False
        
        rospy.loginfo(f"开始执行航点任务，共{len(self.waypoints)}个航点")
        
        # 打印所有航点信息（便于调试）
        rospy.loginfo("航点列表:")
        for i, wp in enumerate(self.waypoints):
            pos = wp.get('global_position', {})
            coord = wp.get('coordinate', 'N/A')
            action = wp.get('action', 'N/A')
            x = pos.get('x', 'N/A')
            y = pos.get('y', 'N/A')
            z = pos.get('z', 'N/A')
            rospy.loginfo(f"  航点{i+1}: {coord} ({x}, {y}, {z}) - {action}")
        
        try:
            # 首先解锁无人机
            rospy.loginfo("开始解锁无人机...")
            self.unlock_drone()
            rospy.loginfo("解锁完成，准备执行航点任务")
            
            # 获取起降点位置
            takeoff_pos = None
            for wp in self.waypoints:
                if wp.get('action') == 'takeoff':
                    takeoff_pos = wp.get('global_position', {})
                    break
            
            if not takeoff_pos:
                takeoff_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
                rospy.logwarn("未找到起飞点，使用默认起飞位置(0, 0, 0)")
            
            # 开始执行航点任务
            i = 0
            while i < len(self.waypoints):
                if rospy.is_shutdown() or not self.mission_active:
                    rospy.logwarn("任务被中断")
                    break
                
                # 获取当前航点信息
                waypoint = self.waypoints[i]
                pos = waypoint.get('global_position', {})
                x = pos.get('x', 0.0)
                y = pos.get('y', 0.0)
                z = pos.get('z', 0.0)
                wp_id = waypoint.get('id', i+1)
                coord = waypoint.get('coordinate', 'Unknown')
                action = waypoint.get('action', 'survey')
                
                # 判断是否是第一个航点
                is_first_waypoint = (i == 0)
                
                # 判断是否是返航阶段（最后两个航点）
                is_near_home = (i >= len(self.waypoints) - 2)
                
                # 检查是否应该直接返回原点
                if is_near_home and action == 'survey':
                    rospy.loginfo(f"临近返航点，直接返回起降点")
                    x, y, z = 0.0, 0.0, 0.0  # 设置返回原点
                    duration = 4.0  # 固定使用4秒返回
                    self.fly_to_waypoint(x, y, z, f"直接返航到起点", duration)
                    
                    # 执行降落和上锁
                    self.land_drone()
                    return True
                
                # 判断是否是最后一个航点
                is_last_waypoint = (i == len(self.waypoints) - 1)
                
                rospy.loginfo(f"执行航点 {wp_id}/{len(self.waypoints)}: {coord} ({x}, {y}, {z}) - {action}")
                
                # 航点执行逻辑
                if is_last_waypoint or action == 'land':
                    # 如果是最后一个航点或降落点，执行降落和上锁
                    self.land_drone()
                    return True
                elif action == 'takeoff':
                    # 起飞点 - 第一次起飞用5秒
                    duration = 5.0 if is_first_waypoint else 2.0
                    self.fly_to_waypoint(x, y, z, f"起飞到航点{wp_id}", duration)
                    i += 1  # 移至下一个航点
                else:
                    # 普通巡航点 - 检查直线段上连续航点的数量
                    
                    # 分析连续直线段上的航点
                    # 计算当前位置到后续各点的方向向量
                    continuous_waypoints = 1  # 包括当前点
                    direction_vector = None
                    current_pos = pos
                    
                    # 遍历后续航点，计算直线段
                    for j in range(i+1, len(self.waypoints)):
                        if self.waypoints[j].get('action') != 'survey':
                            break  # 遇到非survey类型航点就停止
                        
                        next_pos = self.waypoints[j].get('global_position', {})
                        next_x = next_pos.get('x', 0.0)
                        next_y = next_pos.get('y', 0.0)
                        
                        # 计算向量
                        if direction_vector is None:
                            # 第一次计算，设置初始方向向量
                            dx = next_x - current_pos.get('x', 0.0)
                            dy = next_y - current_pos.get('y', 0.0)
                            direction_vector = (dx, dy)
                            continuous_waypoints += 1
                        else:
                            # 判断是否在同一直线上
                            new_dx = next_x - current_pos.get('x', 0.0)
                            new_dy = next_y - current_pos.get('y', 0.0)
                            
                            # 计算两个向量的夹角余弦值，判断是否接近1（同向）
                            dot_product = (direction_vector[0] * new_dx + direction_vector[1] * new_dy)
                            magnitude1 = (direction_vector[0]**2 + direction_vector[1]**2)**0.5
                            magnitude2 = (new_dx**2 + new_dy**2)**0.5
                            
                            if magnitude1 > 0 and magnitude2 > 0:
                                cos_angle = dot_product / (magnitude1 * magnitude2)
                                if cos_angle > 0.99:  # 夹角接近0，即同向
                                    continuous_waypoints += 1
                                else:
                                    break  # 方向改变，航线拐弯
                            else:
                                break  # 向量计算异常
                    
                    rospy.loginfo(f"检测到连续直线段上有 {continuous_waypoints} 个航点")
                    
                    if continuous_waypoints > 3:
                        # 直线段上有超过3个航点，使用端到端控制
                        end_idx = min(i + continuous_waypoints, len(self.waypoints))
                        end_waypoint = self.waypoints[end_idx - 1]
                        end_pos = end_waypoint.get('global_position', {})
                        end_x = end_pos.get('x', 0.0)
                        end_y = end_pos.get('y', 0.0)
                        end_z = end_pos.get('z', 0.0)
                        
                        rospy.loginfo(f"直线段航点数超过3，使用端到端飞行从航点{wp_id}到{end_waypoint.get('id')}")
                        # 飞行时间为航点数量+2.5秒
                        duration = continuous_waypoints + 2.5
                        self.fly_to_waypoint(end_x, end_y, end_z, f"端到端飞行到航点{end_waypoint.get('id')}", duration)
                        
                        i = i + continuous_waypoints - 1  # 更新索引到最后一个执行的航点
                    else:
                        # 少于或等于3个航点，使用采样点飞行
                        self.fly_points_sampling(x, y, z, f"采样点飞行到航点{wp_id}")
                        i += 1  # 移至下一个航点
                
                # 不需要再增加i，因为在各个分支中已经处理了索引更新
                # i += 1
            
            return True
        except Exception as e:
            rospy.logerr(f"执行航点任务失败: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False
    
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
            rospy.loginfo("等待地面站发送起飞指令...")
            if not self.wait_for_command(wait_for_command_timeout):
                rospy.logwarn("未收到起飞指令，任务终止")
                return False
            rospy.loginfo("已收到起飞指令，开始查找航点文件")
        else:
            rospy.loginfo("测试模式：跳过等待命令阶段")
            self.mission_active = True
        
        # 查找最新的航点文件
        rospy.loginfo("正在查找最新的航点文件...")
        waypoint_file = self.find_latest_waypoint_file()
        if not waypoint_file:
            rospy.logerr("无法找到航点文件，任务终止")
            return False
        
        rospy.loginfo(f"找到航点文件: {waypoint_file}")
        
        # 加载航点
        rospy.loginfo("正在加载航点数据...")
        if not self.load_waypoints(waypoint_file):
            rospy.logerr("加载航点数据失败，任务终止")
            return False
        
        rospy.loginfo(f"成功加载 {len(self.waypoints)} 个航点，开始执行任务")
        
        # 执行航点任务
        return self.execute_mission()


def main():
    """主函数"""
    # 不再需要初始化ROS节点，因为Action_t会自行初始化
    # rospy.init_node('px4_diansai', anonymous=True)
    
    # 检查是否开启测试模式
    import sys
    test_mode = '--test' in sys.argv
    
    # 创建无人机控制对象（使用仿真模式）
    drone = Action_t("sim")
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
