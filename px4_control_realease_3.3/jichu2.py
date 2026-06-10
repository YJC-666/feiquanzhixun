#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2024.6.3
说明：无人机解锁、无人机起飞到1.2m（函数运行时间6s）、无人机按照固定速度降落并上锁
'''

import rospy
import sys
import time
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
    
    def takeoff(self, height=1.2, duration=5):
        """起飞到指定高度
        
        参数:
        height - 目标高度，单位米
        duration - 执行时间，单位秒
        """
        rospy.loginfo(f"起飞到高度 {height}米")
        # 获取当前位置
        current_x, current_y, _ = self.drone.get_current_xyz()
        # 在当前xy位置起飞到指定高度
        self.drone.send_position_x_y_z_t_frame(current_x, current_y, height, duration, frame="local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def land(self):
        """降落"""
        rospy.loginfo("执行降落")
        # 停止所有控制线程
        self.drone.stop_all_threads()
        # 执行降落
        self.drone.land_lock_vz_t(-0.4, 3.2, use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("降落完成")

def main():
    """
    主函数 - 无人机基础控制示例
    执行解锁、起飞到1.2m、降落的简单任务
    """
    # 创建无人机控制对象 - 使用仿真模式
    drone = Action_t("sim_nolog")
    controller = DroneController(drone)
    
    try:
        # 1. 解锁无人机
        controller.unlock()
        
        # 2. 起飞到1.2米高度，执行时间6秒
        controller.takeoff(1.2, 6)
        
        # 3. 降落
        controller.land()
        
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
        # 确保中断时也能安全降落
        drone.land_auto()
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")
        # 确保出错时也能安全降落
        drone.land_auto()
    finally:
        rospy.loginfo("程序结束")

if __name__ == '__main__':
    main()