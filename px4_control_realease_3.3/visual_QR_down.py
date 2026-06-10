#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2024.5.3
说明：专注于二维码识别并发布到/ai_detect_info和/qr_code_info话题
（调用朝下看的摄像头
'''

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import time
import os
import tkinter as tk
from tkinter import StringVar
from std_msgs.msg import String as RosString
from pyzbar.pyzbar import decode  # 导入pyzbar库进行二维码识别

class VisualFollower:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("opencv")
        rospy.loginfo("OpenCV VisualFollow node started")

        # 订阅图像话题/iris_0/camera_1/camera/image_down
        self.image_sub = rospy.Subscriber("/iris_0/camera_1/camera/image_down", Image, self.image_callback)
        # ROS发布器
        self.ai_pub = rospy.Publisher("/ai_detect_info", RosString, queue_size=10)
        self.qr_pub = rospy.Publisher("/qr_code_info", RosString, queue_size=10)
        
        # 初始化坐标
        self.qr_x = 0.0
        self.qr_y = 0.0
        self.qr_x_f = 0.0  # 低通滤波后的x
        self.qr_y_f = 0.0  # 低通滤波后的y
        self.center_alpha = 0.3  # 低通滤波系数

        self.lock = threading.Lock()
        self.last_saved_time = time.time()

        # 创建保存图片的目录
        self.image_save_path = "/home/p/yy330_ws/yy330_sim_map_new/launch_sim/px4_control_realease_3.1/__pycache__"
        if not os.path.exists(self.image_save_path):
            os.makedirs(self.image_save_path)

        # 固定的文件名
        self.image_filename = os.path.join(self.image_save_path, "landmark.png")

        # 不再需要OpenCV的二维码识别器，使用pyzbar库
        self.last_qr_info = ("", "", "")  # 类别1,类别2,降落点
        # Tkinter界面
        self.display_frame = None
        self.display_info = ("", "", "")
        self.tk_thread = threading.Thread(target=self.init_tk, daemon=True)
        self.tk_thread.start()
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

        # 最新画面和信息
        self.latest_frame = None
        self.latest_info = ("", "", "")
        # 添加帧同步控制变量
        self.last_tk_update_time = 0
        self.new_frame_flag = False
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0
        # 启动Tk定时刷新线程
        self.tk_update_thread = threading.Thread(target=self.tk_update_loop, daemon=True)
        self.tk_update_thread.start()

    def init_tk(self):
        self.root = tk.Tk()
        self.root.title("二维码与类别信息显示")
        self.root.geometry("240x300")
        # 创建主框架
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)
        # 信息区
        info_frame = tk.Frame(main_frame, width=200, height=300, bg='white')
        info_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        # 信息标签
        self.var1 = StringVar()
        self.var2 = StringVar()
        self.var3 = StringVar()
        # 类别1
        tk.Label(info_frame, text="类别1:", font=('Arial', 12)).pack(pady=10)
        tk.Label(info_frame, textvariable=self.var1, width=20, bg='white', font=('Arial', 12)).pack(pady=5)
        # 类别2
        tk.Label(info_frame, text="类别2:", font=('Arial', 12)).pack(pady=10)
        tk.Label(info_frame, textvariable=self.var2, width=20, bg='white', font=('Arial', 12)).pack(pady=5)
        # 降落点
        tk.Label(info_frame, text="降落点:", font=('Arial', 12)).pack(pady=10)
        tk.Label(info_frame, textvariable=self.var3, width=20, bg='white', font=('Arial', 12)).pack(pady=5)
        # 设置窗口关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        """处理窗口关闭事件"""
        self.root.quit()
        self.root.destroy()
        rospy.signal_shutdown("GUI closed")

    def update_tk(self, frame, info):
        """只更新Tkinter界面信息，不显示画面"""
        try:
            with self.lock:
                self.var1.set(info[0])
                self.var2.set(info[1])
                self.var3.set(info[2])
        except Exception as e:
            rospy.logerr(f"更新信息时出错: {e}")

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_frame(frame)
            # 标记有新帧
            with self.lock:
                self.new_frame_flag = True
                self.frame_count += 1
                # 计算FPS
                now = time.time()
                if now - self.last_fps_time >= 1.0:
                    self.fps = self.frame_count
                    self.frame_count = 0
                    self.last_fps_time = now
                    rospy.loginfo(f"Current FPS: {self.fps}")
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")

    def process_frame(self, frame):
        # 使用pyzbar库进行二维码识别
        decoded_objects = decode(frame)
        qr_info = ("", "", "")
        qr_detected = False
        
        if decoded_objects:
            # 获取第一个识别到的二维码
            qr_code = decoded_objects[0]
            qr_data = qr_code.data.decode('utf-8')
            
            # 期望格式: 类别1,类别2,left/right
            parts = qr_data.strip().split(',')
            if len(parts) == 3:
                qr_info = (parts[0].strip(), parts[1].strip(), parts[2].strip())
                self.last_qr_info = qr_info
                
                # 获取二维码边界框
                rect = qr_code.rect
                # 画出二维码边框
                (x, y, w, h) = rect
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # 计算二维码中心点坐标
                qr_x = x + w // 2
                qr_y = y + h // 2
                qr_detected = True
                
                # 在图像上标记二维码中心点
                cv2.circle(frame, (qr_x, qr_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"QR: ({qr_x}, {qr_y})", (qr_x + 10, qr_y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # 更新坐标并应用低通滤波
                self.qr_x = float(qr_x)
                self.qr_y = float(qr_y)
                self.qr_x_f = self.center_alpha * self.qr_x + (1 - self.center_alpha) * self.qr_x_f
                self.qr_y_f = self.center_alpha * self.qr_y + (1 - self.center_alpha) * self.qr_y_f
                
                # 发布到/qr_code_info话题
                self.qr_pub.publish(RosString(f"{qr_info[0]},{qr_info[1]},{qr_info[2]}"))
                rospy.loginfo(f"二维码识别并发布到/qr_code_info: {qr_info[0]},{qr_info[1]},{qr_info[2]}")
                
                # 发布QR码坐标到/ai_detect_info话题
                self.publish_detection(int(self.qr_x_f), int(self.qr_y_f), "QR")
                rospy.loginfo(f"QR码目标坐标: x={qr_x}, y={qr_y}, 滤波后: x_f={int(self.qr_x_f)}, y_f={int(self.qr_y_f)}")
            else:
                # 如果二维码格式不正确，发布未找到消息
                self.publish_detection(-1, -1, "not_found")
        else:
            # 如果没有检测到二维码，发布未找到消息
            qr_info = self.last_qr_info
            self.publish_detection(-1, -1, "not_found")
        
        # 只用处理后画面刷新Tk
        with self.lock:
            self.latest_frame = frame.copy()
            self.latest_info = qr_info
        # OpenCV窗口显示画面
        try:
            cv2.imshow("Detect", frame)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"OpenCV窗口显示出错: {e}")

    def adjust_image(self, image):
        # 调整图像以优化pyzbar库的二维码识别
        # 转为灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 应用自适应阈值处理，增强二维码边缘
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        # 调整对比度和亮度
        alpha = 1.2  # 对比度控制 (1.0-3.0)
        beta = 20   # 亮度控制 (-100到100)
        adjusted = cv2.convertScaleAbs(thresh, alpha=alpha, beta=beta)
        
        return adjusted

    def save_image(self, image):
        if os.path.exists(self.image_filename):
            os.remove(self.image_filename)
        adjusted_image = self.adjust_image(image)
        cv2.imwrite(self.image_filename, adjusted_image)
        rospy.loginfo("Saved adjusted cropped image to {}".format(self.image_filename))

    def start(self):
        rospy.spin()

    def display_loop(self):
        """显示循环（已不再使用）"""
        pass

    def tk_update_loop(self):
        """Tkinter更新循环"""
        while not rospy.is_shutdown():
            try:
                now = time.time()
                with self.lock:
                    frame = self.latest_frame.copy() if self.latest_frame is not None else None
                    info = self.latest_info
                    new_frame = self.new_frame_flag
                    if new_frame:
                        self.new_frame_flag = False
                
                # 只有有新帧且间隔大于1/30秒才刷新
                if frame is not None and new_frame and (now - self.last_tk_update_time) >= 1/30.0:
                    self.update_tk(frame, info)
                    self.last_tk_update_time = now
                
                # 小睡眠，防止CPU占用过高
                time.sleep(0.001)
                    
            except Exception as e:
                rospy.logerr(f"tk更新循环出错: {e}")

    def publish_detection(self, x, y, class_name):
        """
        发布检测结果到/ai_detect_info话题，按照ros_initialization.py中的格式
        
        参数:
        - x: 目标的X坐标
        - y: 目标的Y坐标
        - class_name: 目标的类别名称
        """
        try:
            if x == -1 and y == -1:
                # 未检测到目标时发送not_found消息
                msg_data = "not_found: x:-1 y:-1;"
            else:
                # 按照约定格式发布消息：'QR: x:320 y:240;' 或 'rect: x:320 y:240;'
                msg_data = f"{class_name}: x:{x} y:{y};"
            
            self.ai_pub.publish(RosString(msg_data))
            
        except Exception as e:
            rospy.logerr(f"发布检测结果时出错: {e}")

if __name__ == '__main__':
    vf = VisualFollower()
    vf.start()