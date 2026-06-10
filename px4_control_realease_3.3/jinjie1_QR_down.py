#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2024.5.3
说明：起飞到1.2m后，往前飞2m的过程中检测二维码，检测到就停止并降落
'''

import rospy
import time
from action_t import Action_t
from std_msgs.msg import String as RosString

class TaskController:
    def __init__(self, drone):
        self.drone = drone
        self.qr_detected = False
        # 订阅二维码识别结果
        self.qr_sub = rospy.Subscriber("/qr_code_info", RosString, self.qr_callback)

    def qr_callback(self, msg):
        if msg.data.strip():  # 如果收到非空的二维码信息
            self.qr_detected = True
            self.drone.stop_all_threads()  # 立即停止所有控制线程

    def takeoff_and_detect(self):
        # 解锁并起飞到1.2米
        self.drone.unlock(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            time.sleep(0.1)
        
        self.drone.send_position_x_y_z_t_frame(0, 0, 1.2, 8, frame="local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            time.sleep(0.1)

        # 向前飞行2米
        self.drone.send_position_x_y_z_t_frame(2, 0, 1.2, 5, frame="local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            if self.qr_detected:  # 如果检测到二维码
                break
            time.sleep(0.1)

        # 自动降落
        self.drone.land_auto()
        while not self.drone.control_complete() and not rospy.is_shutdown():
            time.sleep(0.1)

def main():
    try:
        # 创建无人机对象（使用仿真模式）
        drone = Action_t("sim")
        controller = TaskController(drone)
        controller.takeoff_and_detect()

    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")
        drone.land_auto()
    finally:
        rospy.loginfo("程序结束")
        drone.land_auto()

if __name__ == "__main__":
    main()