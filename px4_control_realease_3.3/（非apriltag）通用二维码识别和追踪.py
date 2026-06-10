#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
作者: 杨锦成
微信: 18873833517
时间: 2024.8.26
文件名: video_q.py
说明: mavros_mask 0b010111000011 //选择需要的功能掩码，需要的给0, 不需要的给1, 从右往左对应PX PY PZ VX VY VZ AFX AFY AFZ FORCE YAW YAW_RATE
功能:
    1. 订阅相机图像主题 `/camera/color/image_raw`（来自 D435 相机）。
    2. 检测并解码二维码。
    3. 使用光流法进行二维码追踪。
    4. 使用卡尔曼滤波器预测和校正二维码中心位置。
    5. 发布二维码信息到 `Q_code` 主题。
    6. 发布处理后的图像到 `qcode_image` 主题。
'''

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading

class QRCodeTracker:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('qr_code_tracker', anonymous=True)

        # 订阅相机图像主题
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # 初始化CV桥，用于ROS图像消息与OpenCV图像之间的转换
        self.bridge = CvBridge()

        # 初始化二维码检测器
        self.qr_detector = cv2.QRCodeDetector()

        # 定义用于光流的参数
        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # 初始化卡尔曼滤波器
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                 [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                [0, 1, 0, 1],
                                                [0, 0, 1, 0],
                                                [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

        # 定义锁，用于线程同步
        self.lock = threading.Lock()

        # 初始化变量
        self.tracking = False              # 是否在追踪状态
        self.prev_gray = None              # 前一帧的灰度图
        self.p0 = None                     # 初始特征点
        self.data = ""                     # 当前二维码数据
        self.center = (0, 0)               # 二维码中心坐标

        # 保存上一帧的二维码信息
        self.last_qr_data = ""
        self.frames_since_last_detection = 0
        self.MAX_MISSING_FRAMES = 5        # 最大允许缺失的帧数

        # 创建发布者
        self.qr_pub = rospy.Publisher('Q_code', String, queue_size=10)
        self.image_pub = rospy.Publisher('qcode_image', Image, queue_size=10)  # 发布处理后的图像

        rospy.loginfo("二维码追踪节点初始化完成。")

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge转换错误: {0}".format(e))
            return

        # 对图像进行高斯滤波，减少噪音
        blur = cv2.GaussianBlur(cv_image, (5, 5), 0)

        # 转换为灰度图像
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        # 处理当前帧（二维码检测和光流追踪）
        self.process_frame(gray, cv_image)

        # 发布处理后的图像到 `qcode_image` 主题
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(processed_image_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge发布图像错误: {0}".format(e))

    def qr_detection(self, gray_frame, frame):
        # 尝试检测和解码二维码
        data_tmp, points_tmp, _ = self.qr_detector.detectAndDecode(gray_frame)
        if points_tmp is not None and data_tmp:
            with self.lock:
                self.data = data_tmp
                points = points_tmp[0]
                self.last_qr_data = data_tmp  # 保存最新的二维码信息
                self.frames_since_last_detection = 0  # 重置计数

                # 绘制绿色多边形框出二维码
                pts = points.astype(int).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                # 计算中心坐标
                center_x = int(np.mean(points[:, 0]))
                center_y = int(np.mean(points[:, 1]))
                center_measured = np.array([[np.float32(center_x)], [np.float32(center_y)]])
                
                # 卡尔曼滤波器校正
                self.kalman.correct(center_measured)
                
                # 卡尔曼滤波器预测
                prediction = self.kalman.predict()
                self.center = (int(prediction[0]), int(prediction[1]))

                # 打印中心坐标
                rospy.loginfo("二维码中心坐标: {}".format(self.center))

                # 在图像上显示识别的信息
                cv2.putText(frame, f"Data: {self.data}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Center: {self.center}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 发布二维码信息，格式为 '类名: x: y:'
                class_name = "QR"  # 假设二维码的类别为 "QR"
                qr_message = f"{class_name}: x:{self.center[0]} y:{self.center[1]}"
                self.qr_pub.publish(qr_message)

                # 为光流追踪做准备
                self.prev_gray = gray_frame.copy()
                self.p0 = points.reshape(-1, 1, 2)
                self.tracking = True
        else:
            # 未检测到二维码
            with self.lock:
                if self.tracking:
                    self.frames_since_last_detection += 1

    def optical_flow_tracking(self, gray_frame, frame):
        with self.lock:
            if self.tracking and self.prev_gray is not None and self.p0 is not None:
                # 使用光流追踪特征点
                p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray_frame, self.p0, None, **self.lk_params)

                if p1 is not None:
                    # 选择好的追踪点
                    good_new = p1[st == 1]
                    good_old = self.p0[st == 1]

                    if len(good_new) >= 4:
                        # 绘制追踪点
                        pts = good_new.astype(int).reshape((-1, 1, 2))
                        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                        # 计算中心坐标
                        center_x = int(np.mean(good_new[:, 0]))
                        center_y = int(np.mean(good_new[:, 1]))
                        center_measured = np.array([[np.float32(center_x)], [np.float32(center_y)]])

                        # 卡尔曼滤波器校正
                        self.kalman.correct(center_measured)

                        # 卡尔曼滤波器预测
                        prediction = self.kalman.predict()
                        self.center = (int(prediction[0]), int(prediction[1]))

                        # 打印中心坐标
                        rospy.loginfo("追踪中二维码中心坐标: {}".format(self.center))

                        # 在图像上显示保存的二维码信息
                        if self.last_qr_data:
                            cv2.putText(frame, f"Data: {self.last_qr_data}", (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.putText(frame, f"Center: {self.center}", (10, 60),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            
                            # 发布保存的二维码信息，格式为 '类名: x: y:'
                            class_name = "QR"
                            qr_message = f"{class_name}: x:{self.center[0]} y:{self.center[1]}"
                            self.qr_pub.publish(qr_message)

                        # 更新前一帧和追踪点
                        self.prev_gray = gray_frame.copy()
                        self.p0 = good_new.reshape(-1, 1, 2)
                    else:
                        # 追踪失败
                        self.tracking = False
                        self.p0 = None
                        self.last_qr_data = ""
                        rospy.loginfo("追踪失败，清除保存的二维码信息。")
                else:
                    # 追踪失败
                    self.tracking = False
                    self.p0 = None
                    self.last_qr_data = ""
                    rospy.loginfo("追踪失败，清除保存的二维码信息。")

                # 如果超过最大缺失帧数，清除保存的二维码信息
                if self.frames_since_last_detection > self.MAX_MISSING_FRAMES:
                    self.last_qr_data = ""
                    self.tracking = False
                    self.p0 = None
                    rospy.loginfo("超过最大缺失帧数，清除保存的二维码信息。")

            # 如果仅有追踪但未检测到二维码且有保存的二维码信息且未超过最大缺失帧数
            if self.frames_since_last_detection > 0 and self.last_qr_data and self.frames_since_last_detection <= self.MAX_MISSING_FRAMES:
                # 在图像上显示之前保存的二维码信息
                cv2.putText(frame, f"Data: {self.last_qr_data}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Center: {self.center}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 发布保存的二维码信息，格式为 '类名: x: y:'
                class_name = "QR"
                qr_message = f"{class_name}: x:{self.center[0]} y:{self.center[1]}"
                self.qr_pub.publish(qr_message)

    def process_frame(self, gray, frame):
        # 创建两个线程，一个用于二维码检测，另一个用于光流追踪
        threads = []
        t1 = threading.Thread(target=self.qr_detection, args=(gray, frame))
        t2 = threading.Thread(target=self.optical_flow_tracking, args=(gray, frame))

        threads.append(t1)
        threads.append(t2)

        # 启动线程
        for t in threads:
            t.start()

        # 等待所有线程完成
        for t in threads:
            t.join()

    def run(self):
        rospy.loginfo("启动二维码追踪节点...")
        rospy.spin()
        # 节点关闭时销毁所有OpenCV窗口（尽管此处已移除显示窗口）
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tracker = QRCodeTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass

