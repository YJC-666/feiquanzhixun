#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2024.6.3
说明：无人机基础控制示例3 - 解锁、起飞、前进、旋转、高度变化、降落
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
    
    def takeoff(self, height=1.2):
        """起飞到指定高度"""
        rospy.loginfo(f"起飞到高度 {height}米")
        # 获取当前位置
        current_x, current_y, _ = self.drone.get_current_xyz()
        # 在当前位置起飞到指定高度
        self.drone.send_position_x_y_z_t_frame(current_x, current_y, height, 5, "local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def move_forward(self, distance, height=1.2):
        """向前移动指定距离"""
        rospy.loginfo(f"向前移动 {distance}米")
        # 使用机体坐标系前进
        self.drone.send_position_x_y_z_t_frame(distance, 0, height, 5, "body", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def rotate_yaw(self, angle, height=1.2):
        """旋转指定角度（角度制）"""
        rospy.loginfo(f"旋转 {angle}度")
        # 控制偏航角，保持高度
        self.drone.control_yaw_z_t(angle, height, 5)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def change_height(self, height):
        """改变高度到指定值"""
        rospy.loginfo(f"改变高度到 {height}米")
        # 获取当前位置
        current_x, current_y, _ = self.drone.get_current_xyz()
        # 在当前水平位置改变高度
        self.drone.send_position_x_y_z_t_frame(current_x, current_y, height, 3, "local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def land(self):
        """降落并上锁"""
        rospy.loginfo("执行降落")
        # 执行自动降落
        self.drone.land_auto(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("降落完成")

def main():
    """
    主函数 - 无人机基础控制示例3
    实现：无人机解锁、起飞1.2m、向前飞行1米、顺时针旋转90度，下降0.5m、逆时针旋转90度，最后降落
    """
    # 创建无人机控制对象
    drone = Action_t("real_nolog")
    controller = DroneController(drone)
    
    try:
        # 1. 解锁无人机
        controller.unlock()
        
        # 2. 起飞到1.2米高度
        controller.takeoff(1.2)
        
        # 3. 向前飞行1米
        controller.move_forward(1.0, 1.2)
        
        # 4. 顺时针旋转90度
        controller.rotate_yaw(90, 1.2)
        
        # 5. 下降0.5米到0.7米高度
        controller.change_height(0.7)
        
        # 6. 逆时针旋转90度
        controller.rotate_yaw(-90, 0.7)
        
        # 7. 降落
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

