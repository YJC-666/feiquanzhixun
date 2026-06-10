#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HSVTuner:
    def __init__(self, use_ros=True):
        self.use_ros = use_ros
        self.bridge = CvBridge()

        # 如果是 ROS 模式，则订阅摄像头话题
        if self.use_ros:
            rospy.init_node("hsv_tuner", anonymous=True)
            self.image_sub = rospy.Subscriber("/iris_0/camera_1/camera/image_raw_down", Image, self.image_callback)
            self.frame = None
        else:
            self.cap = cv2.VideoCapture(0)  # 本地摄像头

        # 创建 OpenCV 窗口和滑动条
        self.create_trackbars()

    def create_trackbars(self):
        """ 创建 HSV 滑动条 """
        cv2.namedWindow("HSV Tuner")
        cv2.createTrackbar("H_min", "HSV Tuner", 0, 179, lambda x: None)
        cv2.createTrackbar("H_max", "HSV Tuner", 179, 179, lambda x: None)
        cv2.createTrackbar("S_min", "HSV Tuner", 0, 255, lambda x: None)
        cv2.createTrackbar("S_max", "HSV Tuner", 255, 255, lambda x: None)
        cv2.createTrackbar("V_min", "HSV Tuner", 0, 255, lambda x: None)
        cv2.createTrackbar("V_max", "HSV Tuner", 255, 255, lambda x: None)

    def image_callback(self, msg):
        """ 订阅 ROS 话题并转换为 OpenCV 图像 """
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge 错误: %s", str(e))

    def process_frame(self, frame):
        """ 处理图像：HSV 转换 + 阈值处理 """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 读取滑动条的值
        h_min = cv2.getTrackbarPos("H_min", "HSV Tuner")
        h_max = cv2.getTrackbarPos("H_max", "HSV Tuner")
        s_min = cv2.getTrackbarPos("S_min", "HSV Tuner")
        s_max = cv2.getTrackbarPos("S_max", "HSV Tuner")
        v_min = cv2.getTrackbarPos("V_min", "HSV Tuner")
        v_max = cv2.getTrackbarPos("V_max", "HSV Tuner")

        # 进行阈值分割
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # 结果合成
        result = cv2.bitwise_and(frame, frame, mask=mask)

        return mask, result

    def run(self):
        """ 运行主循环 """
        while True:
            if self.use_ros:
                if self.frame is None:
                    rospy.loginfo("等待 ROS 图像...")
                    continue
                frame = self.frame.copy()
            else:
                ret, frame = self.cap.read()
                if not ret:
                    print("无法获取摄像头图像")
                    break

            mask, result = self.process_frame(frame)

            # 显示结果
            cv2.imshow("Original", frame)
            cv2.imshow("Mask", mask)
            cv2.imshow("Filtered", result)

            # 按 "q" 退出
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()
        if not self.use_ros:
            self.cap.release()

if __name__ == "__main__":
    use_ros = True  # 设置为 False 则使用本地摄像头
    tuner = HSVTuner(use_ros)
    tuner.run()
