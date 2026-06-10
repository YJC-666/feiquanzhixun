#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2024.5.3
说明：起飞到1.2m后，往前飞3m后（无中断打断）才去追踪执行下降
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
        if msg.data.strip() and not self.qr_detected:  # 如果收到非空的二维码信息且是第一次检测到
            self.qr_detected = True
            self.drone.log_info("检测到QR码")
        elif msg.data.strip():
            # 已经检测到过了，只需更新状态
            self.qr_detected = True

    def takeoff_and_detect(self):
        # 解锁并起飞到1.2米
        self.drone.unlock(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            time.sleep(0.1)
        
        self.drone.send_position_x_y_z_t_frame(0, 0, 1.2, 8, frame="local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            time.sleep(0.1)

        # 向前飞行3米，不中断
        self.drone.log_info("开始向前飞行3米...")
        self.drone.send_position_x_y_z_t_frame(2.5, 0.8, 1.2, 6, frame="local", use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            time.sleep(0.1)
        self.drone.log_info("完成3米前进飞行")

        # 完成3米飞行后开始追踪
        self.drone.log_info("开始执行追踪...")
        # 使用track_velocity_direction_centertol_tout_t，方向设为"forward"(朝前)
        self.drone.track_velocity_direction_centertol_tout_t("forward", timeout=3, t=10, use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            time.sleep(0.1)
        self.drone.log_info("追踪完成，准备降落")

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
