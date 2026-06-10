#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机视觉导航控制主程序

作者：无人机控制团队
日期：2024
"""

# 导入必要的库和模块
import rospy
import math
import time
import sys
from std_msgs.msg import String
from action_t import Action_t
import threading

class DroneController:
    def __init__(self, drone):
        self.drone = drone
        self.current_height = 1.3  # 记录当前高度
        self.allow_H_detect = True
        
        # 基础状态变量
        self.flag = 1
        self.next_nav_flag = 0
        self.current_nav_x = None
        self.current_nav_y = None
        self.circle_count = 0  # 记录圆周运动的圈数
        
        # 目标检测相关变量
        self.processed_targets = set()  # 记录已处理的目标
        self.special_targets = ['qiao', 'diao_bao', 'zhuang_jia_che', 'tan_ke', 'H', 'zhang_pen']
        self.is_processing_target = False  # 是否正在处理目标
        self.target_frames_count = {}  # 记录每个目标连续被识别的帧数
        self.MIN_REQUIRED_FRAMES = 6  # 至少需要连续识别6帧才算有效
        
        # 二维码相关
        self.qr_info_lock = threading.Lock()
        self.latest_qr_info = [None, None, None]  # 用list可变对象
        self.qr_info_set = [False]  # 标记是否已识别到有效二维码
        
        # 设置二维码订阅
        self.qr_sub = rospy.Subscriber("/qr_code_info", String, self.qr_code_callback)
        
        rospy.loginfo("=== 无人机视觉导航系统启动 ===")
        rospy.loginfo(f"Python版本: {sys.version}")
        rospy.loginfo("无人机控制对象初始化完成")

    def qr_code_callback(self, msg):
        try:
            parts = msg.data.strip().split(',')
            if len(parts) == 3:
                with self.qr_info_lock:
                    if not self.qr_info_set[0]:
                        self.latest_qr_info[0] = parts[0].strip()
                        self.latest_qr_info[1] = parts[1].strip()
                        self.latest_qr_info[2] = parts[2].strip()
                        self.qr_info_set[0] = True
        except Exception as e:
            rospy.logerr(f"解析二维码信息失败: {e}")

    def unlock(self):
        """解锁无人机"""
        rospy.loginfo("状态1: 解锁无人机")
        if not getattr(self.drone, 'unlock_thread_running', False):
            self.drone.unlock(use_thread=True)
            self.flag = 2

    def takeoff(self, height=1.0):
        """起飞到指定高度"""
        rospy.loginfo("状态2: 起飞到指定高度")
        self.drone.send_position_x_y_z_t(1.85, 0, height, 5, use_thread=True)
        self.flag = 3

    def rise_to_height(self, height=1.3):
        """上升到指定高度"""
        rospy.loginfo("状态3: 原地上升到1.3m")
        self.drone.send_position_x_y_z_t(1.85, 0, height, 2, use_thread=True)
        self.flag = 4

    def navigate_to_point(self, x, y, z=1.3, yaw=0, tolerance=0.1, frame=1):
        """导航到指定点"""
        rospy.loginfo(f"状态4: publish_nav_goal到({x},{y},{z})")
        self.drone.publish_nav_goal_x_y_z_yaw_tol_frame(x, y, z, yaw, tolerance, frame, use_thread=True)
        self.flag = 5

    def move_to_position(self, x, y, z=1.3, duration=3.2):
        """移动到指定位置"""
        rospy.loginfo(f"状态5: send_position到({x},{y},{z})")
        self.drone.send_position_x_y_z_t(x, y, z, duration, use_thread=True)
        self.flag = 6

    def handle_special_target(self, target):
        """
        处理特殊目标的动作序列：追踪中心 -> 下降3秒 -> 上升3秒 -> 继续航线
        如果追踪超时或发生任何错误，直接恢复航线
        
        Args:
            target (str): 目标类别名称
        """
        try:
            self.is_processing_target = True
            rospy.loginfo(f"开始处理目标: {target}")
            
            # 中断当前航线
            self.drone.stop_all_threads()
            rospy.loginfo("已中断当前航线")
            
            # 1. 追踪到中心
            rospy.loginfo("追踪到目标中心...")
            self.drone.track_velocity_z_centertol_ttol(z=1.3, MAX_VEL=0.3, center_tolerance=20, timeout=3, use_thread=True)
            while not self.drone.control_complete() and not rospy.is_shutdown():
                pass
                
            # 检查是否追踪超时
            if hasattr(self.drone, 'track_timeout_flag') and self.drone.track_timeout_flag:
                rospy.logwarn("追踪超时，直接恢复航线")
                rospy.loginfo(f"恢复到原导航点: x={self.current_nav_x}, y={self.current_nav_y}")
                self.drone.publish_nav_goal_x_y_z_yaw_tol_frame(self.current_nav_x, self.current_nav_y, 1.3, 0, 0.1, 1, use_thread=True)
                return

            # 2. 下降3秒
            rospy.loginfo("下降3秒...")
            self.drone.send_velocity_vx_vy_z_t(0, 0, 0.3, 3, use_thread=True)
            while not self.drone.control_complete() and not rospy.is_shutdown():
                pass
            
            self.drone.control_servo_id_angle_t(1, 100, 0.2)

            # 3. 上升3秒
            rospy.loginfo("上升3秒...")
            self.drone.send_velocity_vx_vy_z_t(0, 0, 1.3, 3, use_thread=True)
            while not self.drone.control_complete() and not rospy.is_shutdown():
                pass
            
            # 4. 继续原航线
            if self.current_nav_x and self.current_nav_y:
                rospy.loginfo("恢复原航线...")
                # 先悬停1秒确保稳定
                self.drone.hover_delay_t(1, use_thread=True)
                while not self.drone.control_complete() and not rospy.is_shutdown():
                    pass
                # 继续导航到原目标点
                rospy.loginfo(f"继续导航到原目标点: x={self.current_nav_x}, y={self.current_nav_y}")
                self.drone.publish_nav_goal_x_y_z_yaw_tol_frame(self.current_nav_x, self.current_nav_y, 1.3, 0, 0.1, 1, use_thread=True)
            
            # 标记目标已处理
            self.processed_targets.add(target)
            rospy.loginfo(f"目标 {target} 处理完成")
            
        except Exception as e:
            rospy.logerr(f"处理目标时发生错误: {e}")
            # 发生错误时也尝试恢复原航线
            if self.current_nav_x and self.current_nav_y:
                try:
                    rospy.loginfo("尝试恢复原航线...")
                    self.drone.publish_nav_goal_x_y_z_yaw_tol_frame(self.current_nav_x, self.current_nav_y, 1.3, 0, 0.1, 1, use_thread=True)
                    while not self.drone.control_complete() and not rospy.is_shutdown():
                        pass
                except Exception as e2:
                    rospy.logerr(f"恢复航线失败: {e2}")
        finally:
            self.is_processing_target = False

    def handle_landing_target(self, target_class):
        """
        根据目标类别选择降落点
        Args:
            target_class (str): 目标类别名称
        """
        if target_class == "left":
            return 0, -1.5
        else:  # right
            return 0, 1.5

    def land(self):
        """降落"""
        rospy.loginfo("状态12: 降落")
        self.drone.land_lock_vz_t(-0.4, 5, use_thread=True)
        self.flag = 13

    def execute_circle_movement(self):
        """执行圆周运动"""
        if self.flag == 5 and self.drone.control_complete():
            rospy.loginfo("状态5: send_position到(3.2,1.1,1.3)")
            self.drone.send_position_x_y_z_t(3.2, 1.1, 1.3, 3.2, use_thread=True)
            self.flag = 6

        elif self.flag == 6 and self.drone.control_complete():
            rospy.loginfo("状态6: send_position到(3.2,-1.1,1.3)")
            self.drone.send_position_x_y_z_t(3.2, -1.1, 1.3, 3.2, use_thread=True)
            self.flag = 7

        elif self.flag == 7 and self.drone.control_complete():
            rospy.loginfo("状态7: send_position到(3.5,-1.1,1.3)")
            self.drone.send_position_x_y_z_t(3.5, -1.1, 1.3, 3.2, use_thread=True)
            self.flag = 8

        elif self.flag == 8 and self.drone.control_complete():
            rospy.loginfo("状态8: send_position到(3.5,1.1,1.3)")
            self.drone.send_position_x_y_z_t(3.5, 1.1, 1.3, 3.2, use_thread=True)
            self.circle_count += 1
            if self.circle_count < 2:
                self.flag = 5  # 再来一圈
            else:
                self.flag = 9

    def execute_final_movement(self):
        """执行最终移动"""
        if self.flag == 9 and self.drone.control_complete():
            rospy.loginfo("状态9: send_position到(3.5,-1.88,1.3)")
            self.drone.send_position_x_y_z_t(3.5, -1.88, 1.3, 4, use_thread=True)
            self.flag = 10

        elif self.flag == 10 and self.drone.control_complete():
            rospy.loginfo("状态10: publish_nav_goal到(3.5,1.88,1.3)")
            self.drone.publish_nav_goal_x_y_z_yaw_tol_frame(3.5, 1.88, 1.3, 0, 0.1, 1, use_thread=True)
            self.flag = 11

    def select_landing_point(self):
        """选择降落点"""
        if self.flag == 11 and self.drone.control_complete():
            rospy.loginfo("状态11: 获取目标类别并选择降落点")
            # 优先用二维码识别的降落点
            landing_x, landing_y = None, None
            with self.qr_info_lock:
                qr_class1, qr_class2, qr_landing = self.latest_qr_info
            if qr_landing == "left":
                landing_x, landing_y = 0, -1.5
            elif qr_landing == "right":
                landing_x, landing_y = 0, 1.5
            else:
                # 兼容原有yolo类别
                target_class = self.drone.get_current_yolo_class()
                if target_class == "left":
                    landing_x, landing_y = 0, -1.5
                elif target_class == "right":
                    landing_x, landing_y = 0, 1.5
            if landing_x is not None:
                self.drone.send_position_x_y_z_t(landing_x, landing_y, 1.3, 5, use_thread=True)
                self.flag = 12
            else:
                rospy.logwarn("未检测到有效的降落点标记，等待检测...")

    def run_mission(self):
        """执行主任务循环"""
        while not rospy.is_shutdown():
            try:
                # 状态机：处理不同阶段的导航任务
                if self.flag == 1:  # 解锁状态
                    self.unlock()

                elif self.flag == 2 and self.drone.control_complete():
                    self.takeoff(1.0)

                elif self.flag == 3 and self.drone.control_complete():
                    self.rise_to_height(1.3)

                elif self.flag == 4 and self.drone.control_complete():
                    self.navigate_to_point(3.5, 1.1, 1.3)

                elif self.flag in [5, 6, 7, 8]:
                    self.execute_circle_movement()

                elif self.flag in [9, 10]:
                    self.execute_final_movement()

                elif self.flag == 11:
                    self.select_landing_point()

                elif self.flag == 12 and self.drone.control_complete():
                    self.land()

                elif self.flag == 13:
                    rospy.loginfo("状态13: 所有动作完成")
                    self.drone.stop_all_threads()  # 确保所有线程都停止
                    break

            except Exception as e:
                rospy.logerr(f"发生错误: {e}")
                self.drone.stop_all_threads()  # 在发生错误时停止所有线程
                break

def main():
    """
    主函数
    在这里设置航线任务
    """
    # 创建无人机控制对象
    drone = Action_t("sim")
    controller = DroneController(drone)
    
    try:
        # 执行主任务
        controller.run_mission()
        
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"程序执行出错: {e}")
        controller.drone.land_auto(use_thread=True)
    finally:
        rospy.loginfo("程序结束")

if __name__ == '__main__':
    main()
