#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2024.5.3
说明：起飞到1.2m后，往前飞2m的过程中检测二维码，检测到就停止并用track_velocity_z_centertol_tout_t追踪10秒后降落上锁
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
            self.drone.stop_all_threads()  # 仅在首次检测到时停止线程
            self.drone.log_info("首次检测到QR码，已停止当前控制线程")
        elif msg.data.strip():
            # 已经检测到过了，只需更新状态不需要重复停止线程
            self.qr_detected = True

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

        # 如果检测到二维码，则使用track_velocity_z_centertol_tout_t追踪10秒
        if self.qr_detected:
            self.drone.log_info("检测到二维码，开始追踪...")
            # 使用当前高度作为追踪高度
            current_z = self.drone.get_current_xyz()[2]
            self.drone.track_velocity_z_centertol_tout_t(current_z, timeout=3, t=10, use_thread=True)
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