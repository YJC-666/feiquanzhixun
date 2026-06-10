#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2023.5.1
说明：仿真环境下无人机解锁与上锁示例（程控示例1）
'''

import rospy
import sys
import time
from action_t import Action_t

class TaskController:
    def __init__(self, drone):
        """
        初始化任务控制器
        参数：
        - drone: Action_t对象
        """
        self.drone = drone

    def unlock(self):
        """
        解锁无人机
        """
        rospy.loginfo("解锁无人机...")
        self.drone.unlock(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("无人机已解锁")

    def lock(self):
        """
        上锁无人机
        """
        rospy.loginfo("上锁无人机...")
        self.drone.lock(use_thread=True)
        while not self.drone.control_complete() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("无人机已上锁")

    def stop_all_threads(self):
        """
        停止所有控制线程
        """
        self.drone.stop_all_threads()


def main():
    '''
    主函数：仿真环境下无人机解锁与上锁示例
    '''
    # 创建仿真无人机控制对象
    drone = Action_t("real")
    controller = TaskController(drone)

    try:
        # 1. 解锁无人机
        controller.unlock()
        # 2. 上锁无人机
        controller.lock()
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
        controller.stop_all_threads()
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")
        controller.stop_all_threads()
    finally:
        rospy.loginfo("任务结束")

if __name__ == '__main__':
    main()