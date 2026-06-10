#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2024.6.3
说明：无人机解锁、起飞到1.2m、导航避障往前飞3m旋转60度、自动降落上锁
'''

import rospy
import sys
import time
import math
from action_t import Action_t

class DroneController:
    def __init__(self, drone):
        """初始化无人机控制器"""
        self.drone = drone
    
    def unlock(self):
        """解锁无人机"""
        rospy.loginfo("解锁无人机")
        self.drone.unlock(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def takeoff(self, height=1.2):
        """起飞到指定高度"""
        rospy.loginfo(f"起飞到高度 {height}米")
        # 获取当前位置
        current_x, current_y, _ = self.drone.get_current_xyz()
        # 从当前位置起飞到指定高度
        self.drone.send_position_x_y_z_t_frame(current_x, current_y, height, 5, "local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def navigate_forward_with_rotation(self, distance=3.0, yaw_angle=60, height=1.2):
        """使用导航功能前进指定距离并旋转指定角度"""
        rospy.loginfo(f"使用导航功能前进 {distance} 米并旋转 {yaw_angle} 度")
        
        # 获取当前位置
        current_x, current_y, _ = self.drone.get_current_xyz()
        
        # 计算目标位置（直接前进distance米）
        target_x = current_x + distance
        target_y = current_y
        
        # 使用publish_nav_goal导航功能前进并旋转，frame设为1(local)，时间设为6秒
        self.drone.publish_nav_goal_x_y_z_yaw_t_frame(
            target_x, target_y, height, yaw_angle, 6, "local", use_thread=True
        )
        
        # 等待导航完成
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def land(self):
        """自动降落"""
        rospy.loginfo("执行自动降落")
        self.drone.land_auto(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)

def main():
    """
    主函数 - 执行仿真飞行和程控示例6
    任务流程：
    1. 无人机解锁
    2. 无人机起飞到1.2m高
    3. 无人机导航避障往前飞3m旋转60度
    4. 无人机自动降落
    """
    # 创建无人机控制对象
    drone = Action_t("sim")  # 使用仿真模式
    controller = DroneController(drone)
    
    try:
        # 1. 解锁无人机
        controller.unlock()
        
        # 2. 起飞到1.2m高度
        controller.takeoff(1.2)
        
        # 3. 无人机导航避障往前飞3m旋转60度
        controller.navigate_forward_with_rotation(3.0, 0, 1.22)
        
        # 4. 无人机自动降落
        controller.land()
        
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
        drone.land_auto()
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")
        drone.land_auto()
    finally:
        rospy.loginfo("程序结束")

if __name__ == '__main__':
    main()