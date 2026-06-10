#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：AI助手
时间：2023-10-25
说明：无人机解锁、无人机起飞、无人机向前飞行到1.2m高、
根据0.5米每秒速度往前飞2s(local)、顺时针旋转50度、
根据0.5米每秒往前飞2s(body)、自动降落上锁
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
        # 从当前位置起飞到指定高度
        self.drone.send_position_x_y_z_t_frame(current_x, current_y, height, 5, "local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def fly_forward_local(self, speed, duration, height=1.2):
        """在局部坐标系中以指定速度向前飞行指定时间"""
        rospy.loginfo(f"以 {speed} m/s 的速度在局部坐标系中向前飞行 {duration} 秒")
        self.drone.send_velocity_vx_vy_z_t_frame(speed, 0, height, duration, "local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def fly_forward_body(self, speed, duration, height=1.2):
        """在机体坐标系中以指定速度向前飞行指定时间"""
        rospy.loginfo(f"以 {speed} m/s 的速度在机体坐标系中向前飞行 {duration} 秒")
        self.drone.send_velocity_vx_vy_z_t_frame(speed, 0, height, duration, "body", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def rotate_yaw(self, angle, height=1.2, duration=3):
        """旋转指定角度"""
        rospy.loginfo(f"旋转 {angle} 度")
        self.drone.control_yaw_z_t(angle, height, duration, use_thread=True)
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
    主函数 - 执行仿真飞行和程控示例5
    任务流程：
    1. 无人机解锁
    2. 无人机起飞到1.2m高
    3. 根据0.5m/s往前飞2s(local坐标系)
    4. 顺时针旋转50度
    5. 根据0.5m/s往前飞2s(body坐标系)
    6. 无人机自动降落
    """
    # 创建无人机控制对象
    drone = Action_t("sim_nolog")  # 使用仿真模式
    controller = DroneController(drone)
    
    try:
        # 1. 解锁无人机
        controller.unlock()
        
        # 2. 起飞到1.2m高度
        controller.takeoff(1.2)
        
        # 3. 根据0.5m/s往前飞2s(local坐标系)
        controller.fly_forward_local(0.5, 2, 1.2)
        
        # 4. 顺时针旋转50度
        controller.rotate_yaw(50, 1.2, 3)
        
        # 5. 根据0.5m/s往前飞2s(body坐标系)
        controller.fly_forward_body(0.5, 2, 1.2)
        
        # 6. 无人机自动降落
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
