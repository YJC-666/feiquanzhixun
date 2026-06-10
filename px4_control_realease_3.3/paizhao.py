import cv2
import time
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

class ROSImageCapture:
    def __init__(self, 
                 save_path="/home/p/yy330_ws/yy330_sim_map_new/launch_sim/px4_control_realease_3.1/da_xiang",
                 base_filename="orin_da_xiang_",
                 image_topic="/iris_0/camera_1/camera/image_down"):
        self.save_path = save_path
        self.base_filename = base_filename
        self.image_topic = image_topic
        self.bridge = CvBridge()
        self.current_frame = None
        self.img_counter = 0
        self.running = True
        
        # 创建保存目录
        if not os.path.exists(save_path):
            os.makedirs(save_path)
            print(f"创建保存目录: {save_path}")
            
        # 初始化ROS节点
        rospy.init_node('image_capture_node', anonymous=True)
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        print(f"订阅图像话题: {self.image_topic}")
        print("按下 'g' 键进行拍照，按 'q' 键退出...")
        
    def image_callback(self, msg):
        """ROS图像话题回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换错误: {e}")
            
    def capture_images_with_manual_trigger(self):
        """手动触发图像捕获"""
        while self.running and not rospy.is_shutdown():
            if self.current_frame is None:
                rospy.logwarn("等待图像数据...")
                rospy.sleep(0.1)
                continue
                
            frame = self.current_frame.copy()

            # 在图像上添加提示信息
            cv2.putText(
                frame,
                "ROS Image Topic",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )
            
            # 显示当前帧数
            cv2.putText(
                frame,
                f"Images: {self.img_counter}",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

            # 显示实时画面
            cv2.imshow("ROS Camera Preview", frame)

            # 检测按键
            key = cv2.waitKey(33) & 0xFF
            if key == ord('q'):  # 按 'q' 键退出
                print("退出程序")
                self.running = False
                break
            elif key == ord('g'):  # 按 'g' 键拍照
                # 保存原始图像（不包含字体消息）
                filename = f"{self.base_filename}{self.img_counter}.jpg"
                filepath = os.path.join(self.save_path, filename)
                cv2.imwrite(filepath, self.current_frame)
                print(f"保存图片: {filepath}")
                self.img_counter += 1

        # 关闭所有窗口
        cv2.destroyAllWindows()
        print("拍照结束")
        
    def start_capture(self):
        """启动图像捕获"""
        try:
            self.capture_images_with_manual_trigger()
        except KeyboardInterrupt:
            print("\n用户中断程序")
        finally:
            self.running = False
            rospy.signal_shutdown("图像捕获结束")


def capture_images_with_manual_trigger(
    save_path="/home/p/yy330_ws/yy330_sim_map_new/launch_sim/px4_control_realease_3.1/lao_hu",
    base_filename="next_chuyu23",
    image_topic="/iris_0/camera_1/camera/image_down"
):
    """兼容性函数，保持原有接口"""
    capture = ROSImageCapture(save_path, base_filename, image_topic)
    capture.start_capture()

if __name__ == "__main__":
    # 用户可以在这里修改保存路径、文件名前缀和图像话题
    save_directory = "/home/p/yy330_ws/yy330_sim_map_new/launch_sim/px4_control_realease_3.1/da_xiang"      # 修改为你想要的保存路径
    filename_prefix = "orin_da_xiang_"        # 修改为你想要的文件名前缀
    topic_name = "/iris_0/camera_1/camera/image_down"  # ROS图像话题名称

    capture_images_with_manual_trigger(
        save_path=save_directory,
        base_filename=filename_prefix,
        image_topic=topic_name
    )

