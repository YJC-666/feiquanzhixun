#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
作者：杨锦成
电话：18873833517
时间：2024.5.3
说明：专注于二维码识别并发布到/ai_detect_info和/qr_code_info话题
（调用朝前看的摄像头）
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

class QRCodeTracker:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("qrcode_tracker")
        rospy.loginfo("QR Code Tracker node started")

        # 订阅图像话题
        self.image_sub = rospy.Subscriber("/iris_0/realsense/depth_camera/color/image_raw", Image, self.image_callback)
        # ROS发布器
        self.ai_pub = rospy.Publisher("/ai_detect_info", RosString, queue_size=10)
        self.qr_pub = rospy.Publisher("/qr_code_info", RosString, queue_size=10)
        
        # 初始化坐标
        self.yolo_x = 0.0
        self.yolo_y = 0.0
        self.yolo_x_f = 0.0  # 低通滤波后的x
        self.yolo_y_f = 0.0  # 低通滤波后的y
        self.center_alpha = 0.3  # 低通滤波系数

        self.dilation_iterations = 1
        self.erosion_iterations = 3

        self.lock = threading.Lock()
        self.last_saved_time = time.time()

        # 创建保存图片的目录
        self.image_save_path = "/home/p/yy330_ws/src/px4_control_realease_1.3"
        if not os.path.exists(self.image_save_path):
            os.makedirs(self.image_save_path)

        # 固定的文件名
        self.image_filename = os.path.join(self.image_save_path, "qrcode.png")

        # 二维码识别器
        self.qr_detector = cv2.QRCodeDetector()
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
        self.root.title("二维码追踪信息显示")
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
        # 只做二维码识别
        qr_data, bbox, _ = self.qr_detector.detectAndDecode(frame)
        qr_info = ("", "", "")
        if qr_data:
            # 期望格式: 类别1,类别2,left/right
            parts = qr_data.strip().split(',')
            if len(parts) == 3:
                qr_info = (parts[0].strip(), parts[1].strip(), parts[2].strip())
                self.last_qr_info = qr_info
                # 画出二维码边框
                if bbox is not None and len(bbox) > 0:
                    pts = np.int32(bbox).reshape(-1, 2)
                    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                    # 计算二维码中心点坐标
                    center_x = np.mean(pts[:, 0])
                    center_y = np.mean(pts[:, 1])
                    self.yolo_x = float(center_x)
                    self.yolo_y = float(center_y)
                    self.yolo_x_f = self.center_alpha * self.yolo_x + (1 - self.center_alpha) * self.yolo_x_f
                    self.yolo_y_f = self.center_alpha * self.yolo_y + (1 - self.center_alpha) * self.yolo_y_f
                    
                    # 在图像上标记二维码中心点
                    cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                    
                    # 发布到ai_detect_info用于视觉追踪
                    msg = f"qrcode: x:{int(self.yolo_x_f)} y:{int(self.yolo_y_f)}"
                    self.ai_pub.publish(RosString(msg))
                    
                # 发布到/qr_code_info (此行保留，因为/qr_code_info话题是给二维码信息用的)
                self.qr_pub.publish(RosString(f"{qr_info[0]},{qr_info[1]},{qr_info[2]}"))
                rospy.loginfo(f"二维码识别并发布到/qr_code_info: {qr_info[0]},{qr_info[1]},{qr_info[2]}")
                rospy.loginfo(f"二维码中心坐标: x={self.yolo_x}, y={self.yolo_y}, 滤波后: x_f={self.yolo_x_f}, y_f={self.yolo_y_f}")
        else:
            # 如果没有检测到二维码，发送not_found消息
            msg = "not_found: x:-1 y:-1"
            self.ai_pub.publish(RosString(msg))
            qr_info = self.last_qr_info
            
        # 只用处理后画面刷新Tk
        with self.lock:
            self.latest_frame = frame.copy()
            self.latest_info = qr_info
            
        # 发布处理后画面到/show_image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # self.show_image_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"发布/show_image失败: {e}")
            
        # OpenCV窗口显示画面
        try:
            cv2.imshow("QR Code Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"OpenCV窗口显示出错: {e}")

    def adjust_image(self, image):
        # 调整曝光度和对比度
        alpha = 1.793  # 对比度控制 (1.0-3.0)
        beta = 38   # 亮度控制 (-100到100)
        adjusted = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

        # 转换为HSV并调整饱和度
        hsv = cv2.cvtColor(adjusted, cv2.COLOR_BGR2HSV)
        hsv[..., 1] = cv2.add(hsv[..., 1], 1)  # 增加饱和度
        adjusted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        
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

    def publish_detection(self, x, y):
        """发布QR码检测结果到/ai_detect_info话题"""
        try:
            msg = RosString()
            if x == -1 and y == -1:
                # 未检测到QR码时发送not_found消息
                msg.data = "not_found: x:-1 y:-1;"
            else:
                # 按照约定格式发布消息：'qrcode: x:320 y:240;'
                msg.data = f"qrcode: x:{x} y:{y};"
            self.ai_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"发布QR码检测结果时出错: {e}")

if __name__ == '__main__':
    tracker = QRCodeTracker()
    tracker.start()

