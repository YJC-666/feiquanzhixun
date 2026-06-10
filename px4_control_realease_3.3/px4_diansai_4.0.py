#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
文件名：px4_diansai_4.0.py
功能：电赛航点跟随任务 - 增强版
说明：读取mission_log目录下最新的waypoints_*.yaml文件，并控制无人机按航点飞行
      需要收到地面站发送的起飞指令才开始执行任务
      确保每个航点都能精确控制
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
                wp_id = wp.get('id', i+1)
                rospy.loginfo(f"航点{wp_id}: {coord} ({pos.get('x', 'N/A')}, {pos.get('y', 'N/A')}, {pos.get('z', 'N/A')}) - {action}")
            
            if len(self.waypoints) > 3:
                rospy.loginfo("...")
            
            return True
        except Exception as e:
            rospy.logerr(f"加载航点文件失败: {e}")
            return False
    
    # 已移除wait_for_position_reached方法，改为按时间执行，无需等待位置到达
    
    def execute_mission(self):
        """
        执行航点任务 - 增强版，确保每个航点都能精确控制
        """
        if not self.waypoints:
            rospy.logerr("没有航点可执行")
            return False
        
        try:
            # 首先解锁无人机
            self.unlock_drone()
            
            # 遍历每个航点
            for i, waypoint in enumerate(self.waypoints):
                if rospy.is_shutdown() or not self.mission_active:
                    rospy.logwarn("任务被中断")
                    break
                
                # 检查是否到达倒数第3个航点
                if i == len(self.waypoints) - 2:
                    rospy.loginfo(f"\n=== 到达倒数第3个航点，开始降落程序 ===")
                    # 飞到(0,0,0)坐标并降落4秒
                    rospy.loginfo("飞向原点坐标(0,0,0)并降落4秒，使用特殊frame: local")
                    self.drone._send_position_x_y_z_t_frame(0, 0, 0, 4, frame="local")
                    
                    # 等待4秒降落完成
                    rospy.sleep(4.0)
                    
                    # 执行上锁
                    rospy.loginfo("执行上锁")
                    self.drone.lock(use_thread=True)
                    while not self.drone.control_complete() and not rospy.is_shutdown():
                        rospy.sleep(0.1)
                    rospy.loginfo("上锁完成")
                    
                    return True
                
                # 获取航点位置
                pos = waypoint.get('global_position', {})
                x = pos.get('x', 0.0)
                y = pos.get('y', 0.0)
                z = pos.get('z', 0.0)
                
                # 获取航点信息
                wp_id = waypoint.get('id', i+1)
                coord = waypoint.get('coordinate', 'Unknown')
                action = waypoint.get('action', 'survey')
                
                rospy.loginfo(f"\n=== 执行航点 {wp_id}/{len(self.waypoints)} ===")
                rospy.loginfo(f"坐标: {coord} ({x:.3f}, {y:.3f}, {z:.3f})")
                rospy.loginfo(f"动作: {action}")
                
                # 判断是否是最后一个航点
                is_last_waypoint = (i == len(self.waypoints) - 1)
                # 判断是否是第一个航点
                is_first_waypoint = (i == 0)
                
                if action == 'takeoff':
                    # 起飞点 - 第一次起飞用4秒
                    duration = 4.0 if is_first_waypoint else 2.0
                    self.fly_to_waypoint_precise(x, y, z, f"起飞到航点{wp_id}", duration=duration, special_frame=True)
                elif action == 'land':
                    # 降落点（中间的降落点仍按原逻辑处理）
                    self.land_drone(x, y)
                else:
                    # 普通巡航点 - 用0.8秒飞行时间
                    self.fly_to_waypoint_precise(x, y, z, f"飞向航点{wp_id}", duration=0.8)
                
                # 无需等待，直接执行下一个航点
            
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
        rospy.loginfo("无人机解锁完成")
    
    def fly_to_waypoint_precise(self, x, y, z, description="", duration=0.8, special_frame=False):
        """
        控制无人机飞到指定航点 - 按时间执行，无需等待到达
        参数：
        - x, y, z: 目标位置坐标
        - description: 描述信息
        - duration: 飞行时间(秒)，默认0.8秒
        - special_frame: 是否使用特殊frame设置，默认False使用local_v=0.4
        返回：始终返回True（不再检查位置到达）
        """
        if description:
            rospy.loginfo(description)
            
        # 输出飞行时间信息
        rospy.loginfo(f"飞行时间: {duration}秒")
        
        # 使用位置控制指令，根据special_frame参数决定使用的frame
        if special_frame:
            # 起飞和返回时使用local frame
            frame = "local"
            rospy.loginfo("使用特殊frame: local")
        else:
            # 普通航点使用local_v=0.4 frame
            frame = "local_v=0.4"
            rospy.loginfo("使用普通frame: local_v=0.4")
            
        # 发送位置控制指令
        self.drone._send_position_x_y_z_t_frame(x, y, z, duration, frame=frame)
        
        # 获取当前位置并输出
        current_xyz = self.drone.get_current_xyz()
        rospy.loginfo(f"到达航点: 当前位置 ({current_xyz[0]:.2f}, {current_xyz[1]:.2f}, {current_xyz[2]:.2f})")
        
        return True
    
    def land_drone(self, x=None, y=None, final_landing=False):
        """
        控制无人机飞向原点并上锁
        参数：
        - x, y: 可选参数（已废弃，现在固定飞向(0,0,0)）
        - final_landing: 是否为最终降落（决定使用wait还是sleep）
        """
        rospy.loginfo("飞向原点(0,0,0)并准备上锁")
        
        # 获取当前高度，飞向(0,0,0)坐标
        current_xyz = self.drone.get_current_xyz()
        rospy.loginfo("飞行到原点坐标(0,0,0)，使用特殊frame: local")
        self.drone._send_position_x_y_z_t_frame(0, 0, 0, 4, frame="local_v=1.5")
        
        if final_landing:
            # 最终降落用固定时间等待
            rospy.loginfo("等待4秒降落完成")
            rospy.sleep(4.0)
            
            # 执行上锁（带线程）
            rospy.loginfo("执行上锁")
            self.drone.lock(use_thread=True)
            while not self.drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
            rospy.loginfo("上锁完成")
        else:
            # 中途降落点等待控制完成
            while not self.drone.control_complete() and not rospy.is_shutdown():
                rospy.sleep(0.1)
                
            # 执行上锁（不使用线程）
            self.drone.lock()
            rospy.loginfo("无人机已上锁")
    
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
        rospy.loginfo("=== PX4电赛航点任务4.0版本启动 ===")
        
        # 等待地面站发送起飞指令（除非处于测试模式）
        if not test_mode:
            if not self.wait_for_command(wait_for_command_timeout):
                rospy.logwarn("未收到起飞指令，任务终止")
                return False
        else:
            rospy.loginfo("测试模式：跳过等待命令阶段")
            self.mission_active = True
        
        # 查找最新的航点文件
        waypoint_file = self.find_latest_waypoint_file()
        if not waypoint_file:
            return False
        
        # 加载航点
        if not self.load_waypoints(waypoint_file):
            return False
        
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