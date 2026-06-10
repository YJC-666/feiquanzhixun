#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
import sys
from ultralytics import YOLO
from time import time
import tf.transformations
import math
import json

from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes

class Yolo_Dect:
    def __init__(self):
        # jiazai canshu
        self.weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov8/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = float(rospy.get_param('~conf', '0.5'))
        self.visualize = rospy.get_param('~visualize', 'True')

        # xiangji neican shezhi (genju yonghu tigong de canshu)
        self.camera_matrix = np.array([[369.502083, 0.0, 640.0],
                                      [0.0, 369.502083, 360.0],
                                      [0.0, 0.0, 1.0]])
        self.fx = 369.502083  # jiaoju x
        self.fy = 369.502083  # jiaoju y
        self.cx = 640.0       # zhudian x
        self.cy = 360.0       # zhudian y
        
        # shiji shiyong de fenbianlv 1280x720
        self.cx_actual = 640.0  # 1280/2
        self.cy_actual = 360.0  # 720/2
        
        # wurenjiGaodu he jiance quyu shezhi
        self.drone_height = 1.22  # wurenji juli dimian gaodu (mi)
        self.roi_size_real = 0.5  # jiance quyu bianzhang (mi): 50cm*50cm
        
        # jisuan jiance quyu zai xiangsu zuobiao xi zhong de fanwei
        self.calculate_roi_pixels()

        # shebei xuanze
        self.device = 'cpu' if rospy.get_param('/use_cpu', 'false') else 'cuda'

        # moxing jiazai
        try:
            self.model = YOLO(self.weight_path, task='detect')
            if self.weight_path.endswith('.pt'):  # 仅PyTorch模型需要特殊处理
                self.model.fuse()
                self.model.to(self.device)
            rospy.loginfo(f"chenggong jiazai moxing: {self.weight_path}")
        except Exception as e:
            rospy.logerr(f"moxing jiazai shibai: {str(e)}")
            sys.exit(1)

        self.model.conf = conf
        self.color_image = Image()
        self.getImageStatus = False
        self.classes_colors = {}

        # ROS tongxin shezhi
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                        queue_size=1, buff_size=52428800)
        self.position_pub = rospy.Publisher(pub_topic, BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher('/yolov8/detection_image', Image, queue_size=1)
        self.xy_pub = rospy.Publisher('/yolov8/pub_image_xy', BoundingBox, queue_size=1)
        # tianjia fabushiqi, yongyu fabu fuhe rosinit.py yolo_callback jiekou de xiaoxi
        self.ai_detect_pub = rospy.Publisher('/ai_detect_info', String, queue_size=1)
        # tianjia fabushiqi, yongyu tongji ROI quyu nei de zhonglei he shuliang
        self.roi_stats_pub = rospy.Publisher('/roi_detection_stats', String, queue_size=1)
        # tianjia fabushiqi, yongyu fabu quanqiu mubiao siyuanshu lichengji xinxi
        self.global_target_odom_pub = rospy.Publisher('/global_target_odometry', Odometry, queue_size=1)
        # tianjia fabushiqi, yongyu fabu mubiao quyu ID xinxi
        self.target_region_pub = rospy.Publisher('/target_region_info', String, queue_size=1)
        
        # dingyue wurenji lichengji xinxi
        self.drone_odom_sub = rospy.Subscriber('/iris_0/mavros/local_position/odom', Odometry, self.odom_callback, queue_size=1)
        
        # wurenji dangqian lichengji xinxi
        self.current_drone_odom = None
        self.drone_height_from_odom = 0.0
        
        # quyu panduan bianliang
        self.target_region_threshold = 0.15  # 15cm yueshu yu run_wildlife_survey.py
        self.detected_regions = {}  # yong yu ji lu jiance dao de quyu
        
        rospy.loginfo(f"YOLOv8 ROS jiedian yi chushihua, jiance quyu: {self.roi_x1}-{self.roi_x2}, {self.roi_y1}-{self.roi_y2}")
        rospy.loginfo("dengdai tuxiang shuru...")
        while not self.getImageStatus and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    def odom_callback(self, odom_msg):
        """
        jieshou wurenji lichengji xinxi huidiaohansu
        """
        try:
            self.current_drone_odom = odom_msg
            # huoqu wurenji z zhou gaodu
            self.drone_height_from_odom = odom_msg.pose.pose.position.z
            # gengxin jiance quyu daxiao (genju shishi gaodu)
            if abs(self.drone_height_from_odom - self.drone_height) > 0.1:  # gaodu bianhua chaoguo 10cm shi gengxin
                self.drone_height = max(0.5, self.drone_height_from_odom)  # zuidi gaodu 0.5mi
                self.calculate_roi_pixels()
        except Exception as e:
            rospy.logerr(f"lichengji shuju chuli cuowu: {str(e)}")
    
    def calculate_roi_pixels(self):
        """
        genju xiangji neican, wurenji gaodu he jiance quyu daxiao jisuan xiangsu zuobiao fanwei
        """
        # jisuan dimian shang 50cm duiying de xiangsu daxiao
        # shiyong xiangsi sanjiaoxing yuanli: xiangsu daxiao = (jiaoju * shiji daxiao) / juli
        pixel_size_x = (self.fx * self.roi_size_real) / self.drone_height
        pixel_size_y = (self.fy * self.roi_size_real) / self.drone_height
        
        # jisuan jiance quyu de xiangsu zuobiao fanwei (yi tuxiang zhongxin wei jizhun)
        half_roi_x = int(pixel_size_x / 2)
        half_roi_y = int(pixel_size_y / 2)
        
        self.roi_x1 = max(0, int(self.cx_actual - half_roi_x))
        self.roi_x2 = min(1280, int(self.cx_actual + half_roi_x))
        self.roi_y1 = max(0, int(self.cy_actual - half_roi_y))
        self.roi_y2 = min(720, int(self.cy_actual + half_roi_y))
        
        rospy.loginfo(f"jisuan dedao de jiance quyu xiangsu fanwei: x[{self.roi_x1}-{self.roi_x2}], y[{self.roi_y1}-{self.roi_y2}]")
        rospy.loginfo(f"jiance quyu xiangsu daxiao: {self.roi_x2-self.roi_x1}x{self.roi_y2-self.roi_y1}")

    def image_callback(self, image):
        try:
            self.boundingBoxes = BoundingBoxes()
            self.boundingBoxes.header = image.header
            self.boundingBoxes.image_header = image.header
            self.getImageStatus = True
            
            # ROS tuxiang zhuan OpenCV geshi
            self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.width, -1)
            # baochi BGR geshi, yinwei OpenCV he ROS dou shiyong BGR geshi
            # self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

            # dui zhengge tuxiang jinxing tuili (zidong chuli ONNX/PyTorch geshi)
            results = self.model.predict(
                self.color_image,
                show=False,
                conf=0.3,
                device=self.device if self.weight_path.endswith('.pt') else None
            )
            
            self.dectshow(results, image.height, image.width)

        except Exception as e:
            rospy.logerr(f"tuili cuowu: {str(e)}")
            self.getImageStatus = False

    def dectshow(self, results, height, width):
        try:
            # chuangjian wanzheng tuxiang yongyu xianshi
            self.frame = self.color_image.copy()
            
            fps = 1000.0 / results[0].speed['inference']
            cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

            for result in results[0].boxes:
                # huoqu jiancekuang zuobiao
                xmin = np.int64(result.xyxy[0][0].item())
                ymin = np.int64(result.xyxy[0][1].item())
                xmax = np.int64(result.xyxy[0][2].item())
                ymax = np.int64(result.xyxy[0][3].item())
                
                # chuangjian bianjiekuang xiaoxi
                boundingBox = BoundingBox()
                boundingBox.xmin = xmin
                boundingBox.ymin = ymin
                boundingBox.xmax = xmax
                boundingBox.ymax = ymax
                boundingBox.Class = results[0].names[result.cls.item()]
                boundingBox.probability = result.conf.item()
                
                # jisuan quanqiu zuobiao yongyu xianshi
                global_coords = self.calculate_global_coordinates(boundingBox)
                
                # zai yuantu shang huizhi jiancekuang (lvse)
                cv2.rectangle(self.frame, (int(boundingBox.xmin), int(boundingBox.ymin)), 
                             (int(boundingBox.xmax), int(boundingBox.ymax)), (0, 255, 0), 2)
                
                # xianshi leibie he zhixin du
                label_text = f'{boundingBox.Class}: {boundingBox.probability:.2f}'
                cv2.putText(self.frame, label_text, 
                           (int(boundingBox.xmin), int(boundingBox.ymin)-50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                if global_coords is not None:
                    # xianshi shishi jiesuan chulai de xy zuobiao
                    coord_text = f'XY: ({global_coords[0]:.2f}, {global_coords[1]:.2f})'
                    cv2.putText(self.frame, coord_text, 
                               (int(boundingBox.xmin), int(boundingBox.ymin)-30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    
                    # panduan bingji lu mubiao suozai quyu
                    region_id = self.determine_target_region(global_coords)
                    if region_id:
                        # xianshi jiesuan chulai de zuobiao ID
                        region_text = f'ID: {region_id}'
                        cv2.putText(self.frame, region_text, 
                                   (int(boundingBox.xmin), int(boundingBox.ymin)-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                        
                        # ji lu mubiao leibie he quyu ID
                        if region_id not in self.detected_regions:
                            self.detected_regions[region_id] = {}
                        
                        if boundingBox.Class not in self.detected_regions[region_id]:
                            self.detected_regions[region_id][boundingBox.Class] = 0
                        
                        self.detected_regions[region_id][boundingBox.Class] += 1
                    else:
                        # ru guo mei you qu yu ID, xian shi "No ID"
                        cv2.putText(self.frame, "ID: No ID", 
                                   (int(boundingBox.xmin), int(boundingBox.ymin)-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                
                self.boundingBoxes.bounding_boxes.append(boundingBox)
                self.xy_pub.publish(boundingBox)
                
                # jisuan bing fabu quanqiu mubiao lichengji xinxi
                self.calculate_and_publish_global_target_odom(boundingBox)

            self.position_pub.publish(self.boundingBoxes)
            self.publish_image(self.frame, height, width)
            # fabu fuhe rosinit.py yolo_callback jiekou geshi de xiaoxi
            self.publish_ai_detect_info()
            # fabu ROI quyu nei de jiance tongji xinxi
            self.publish_roi_stats()
            # fabu mubiao quyu ID xinxi
            self.publish_target_region_info()

            if self.visualize == 'True' or self.visualize is True:
                cv2.imshow('YOLOv8 Detection', self.frame)
                cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"keshihua cuowu: {str(e)}")

    def publish_image(self, imgdata, height, width):
        try:
            image_temp = Image()
            header = Header(stamp=rospy.Time.now())
            header.frame_id = self.camera_frame
            image_temp.height = height
            image_temp.width = width
            image_temp.encoding = 'bgr8'
            image_temp.data = np.array(imgdata).tobytes()
            image_temp.header = header
            image_temp.step = width * 3
            self.image_pub.publish(image_temp)
        except Exception as e:
            rospy.logerr(f"tuxiang fabu cuowu: {str(e)}")

    def publish_ai_detect_info(self):
        """
        fabu fuhe rosinit.py yolo_callback jiekou yaoqiu de jiance xinxi
        geshi: 'class: x:zuobiaozhi y:zuobiaozhi;'
        """
        try:
            if len(self.boundingBoxes.bounding_boxes) == 0:
                # meiyou jiancedao mubiao shi fasong not_found xiaoxi
                msg_data = "mei_jiance_dao: x:-1 y:-1;"
            else:
                msg_data = ""
                for box in self.boundingBoxes.bounding_boxes:
                    # jisuan bianjiekuang zhongxin zuobiao
                    center_x = int((box.xmin + box.xmax) / 2)
                    center_y = int((box.ymin + box.ymax) / 2)
                    # anzhao yueding geshi goujian xiaoxi: 'class: x:zuobiaozhi y:zuobiaozhi;'
                    msg_data += f"{box.Class}: x:{center_x} y:{center_y}; "
                
                # yichu mowei duoyu de kongge
                msg_data = msg_data.rstrip()
            
            # fabu xiaoxi dao /ai_detect_info huati
            self.ai_detect_pub.publish(String(msg_data))
            
        except Exception as e:
            rospy.logerr(f"fabu AI jiance xinxi cuowu: {str(e)}")
    
    def publish_roi_stats(self):
        """
        fabu ROI quyu nei jiancedao de zhonglei tongji xinxi
        geshi: 'class1:shuliang1ge class2:shuliang2ge ...'
        """
        try:
            if len(self.boundingBoxes.bounding_boxes) == 0:
                # meiyou jiancedao mubiao shi fasong kong tongji
                stats_msg = "wu_jiance_mubiao"
            else:
                # tongji ge leibie de shuliang
                class_counts = {}
                for box in self.boundingBoxes.bounding_boxes:
                    class_name = box.Class
                    if class_name in class_counts:
                        class_counts[class_name] += 1
                    else:
                        class_counts[class_name] = 1
                
                # goujian tongji xiaoxi
                stats_parts = []
                for class_name, count in class_counts.items():
                    stats_parts.append(f"{class_name}:{count}")
                
                stats_msg = " ".join(stats_parts)
            
            # fabu tongji xiaoxi dao /roi_detection_stats huati
            self.roi_stats_pub.publish(String(stats_msg))
            
        except Exception as e:
            rospy.logerr(f"fabu ROI tongji xinxi cuowu: {str(e)}")
    

    def calculate_global_coordinates(self, boundingBox):
        """jisuan mubiao de quanqiu zuobiao"""
        if self.current_drone_odom is None:
            return None
        
        # jisuan mubiao zai tuxiang zhong de zhongxin dian
        center_x = (boundingBox.xmin + boundingBox.xmax) / 2.0
        center_y = (boundingBox.ymin + boundingBox.ymax) / 2.0
        
        # shiyong xiangji neicanhe wurenji gaodu jisuan mubiao zai wurenji zuobiaoxia de pianyi
        cx_actual = self.cx
        cy_actual = self.cy
        
        # jisuan mubiao xiangdui yu xiangji zhongxin de pianyi (xiangsu)
        pixel_offset_x = center_x - cx_actual
        pixel_offset_y = center_y - cy_actual
        
        # huoqu wurenji dangqian de zitai (sixiangshu)
        orientation = self.current_drone_odom.pose.pose.orientation
        # zhuanhuan wei yaogunzhongxin - zuo you - shangxia (roll-pitch-yaw)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # shiyong xiangsi sanjiaoxing yuanli jisuan shiji pianyi (mi)
        # bing kaolv wurenji qingxie jiaozheng (roll & pitch)
        
        # jisuan xiangji de z zhou gaodu (kaolv wurenji qingxie)
        # jiaoju fangxiang chengxiang shi fuyou zhengfu, suo yi shi angle_correction = cos(roll)*cos(pitch)
        angle_correction = math.cos(roll) * math.cos(pitch)
        corrected_height = self.drone_height / angle_correction if angle_correction > 0.1 else self.drone_height
        
        # jiaozheng xiangsu pianyi (kaolv pitch he roll)
        # pitch yingxiang y zhou (qianhou)
        # roll yingxiang x zhou (zuo you)
        corrected_pixel_offset_y = pixel_offset_y - math.tan(pitch) * self.fy
        corrected_pixel_offset_x = pixel_offset_x - math.tan(roll) * self.fx
        
        # shiyong jiaozheng hou de gaodu he xiangsu pianyi jisuan shiji pianyi
        # zhu: xiangji zuobiaxi zhong, x xiangqian (chengxiang fangxiang), y xiangyou
        # dan wurenji zuobiaxi zhong, x chaoqian, y chaozuo
        real_offset_forward = -corrected_pixel_offset_y * corrected_height / self.fy
        real_offset_left = -corrected_pixel_offset_x * corrected_height / self.fx
        
        # huoqu wurenji dangqian weizhi he zitai
        drone_x = self.current_drone_odom.pose.pose.position.x
        drone_y = self.current_drone_odom.pose.pose.position.y
        
        # jiang mubiao pianyi zhuanhuan dao quanqiu zuobiaxi (x chaoqian, y chaozuo)
        global_offset_x = real_offset_forward * math.cos(yaw) - real_offset_left * math.sin(yaw)
        global_offset_y = real_offset_forward * math.sin(yaw) + real_offset_left * math.cos(yaw)
        
        # jisuan mubiao de quanqiu weizhi
        # gen ju yonghu fanku, xu yao xiuzheng zuobiaxi pianyi
        # mubiao x=1m, y=1.5m shi, ying gai xianshi x=0.5m, y=0.5m
        # suo yi xu yao jian qu 0.5m de pianyi
        target_global_x = drone_x + global_offset_x - 0.5
        target_global_y = drone_y + global_offset_y - 1
        
        return (target_global_x, target_global_y)
    
    def calculate_and_publish_global_target_odom(self, boundingBox):
        """jisuan bing fabu quanqiu mubiao lichengji xinxi"""
        if self.current_drone_odom is None:
            rospy.logwarn("No odometry data received yet")
            return
        
        # jisuan mubiao zai tuxiang zhong de zhongxin dian
        center_x = (boundingBox.xmin + boundingBox.xmax) / 2.0
        center_y = (boundingBox.ymin + boundingBox.ymax) / 2.0
        
        # shiyong xiangji neicanhe wurenji gaodu jisuan mubiao zai wurenji zuobiaoxia de pianyi
        # xiangji neicanzhong de zhongxin dian (cx, cy) shi xiangdui yu yuantu de
        cx_actual = self.cx
        cy_actual = self.cy
        
        # jisuan mubiao xiangdui yu xiangji zhongxin de pianyi (xiangsu)
        pixel_offset_x = center_x - cx_actual
        pixel_offset_y = center_y - cy_actual
        
        # huoqu wurenji dangqian de zitai (sixiangshu)
        orientation = self.current_drone_odom.pose.pose.orientation
        # zhuanhuan wei yaogunzhongxin - zuo you - shangxia (roll-pitch-yaw)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # shiyong xiangsi sanjiaoxing yuanli jisuan shiji pianyi (mi)
        # bing kaolv wurenji qingxie jiaozheng (roll & pitch)
        
        # jisuan xiangji de z zhou gaodu (kaolv wurenji qingxie)
        # jiaoju fangxiang chengxiang shi fuyou zhengfu, suo yi shi angle_correction = cos(roll)*cos(pitch)
        angle_correction = math.cos(roll) * math.cos(pitch)
        corrected_height = self.drone_height / angle_correction if angle_correction > 0.1 else self.drone_height
        
        # jiaozheng xiangsu pianyi (kaolv pitch he roll)
        # pitch yingxiang y zhou (qianhou)
        # roll yingxiang x zhou (zuo you)
        corrected_pixel_offset_y = pixel_offset_y - math.tan(pitch) * self.fy
        corrected_pixel_offset_x = pixel_offset_x - math.tan(roll) * self.fx
        
        # shiyong jiaozheng hou de gaodu he xiangsu pianyi jisuan shiji pianyi
        # zhu: xiangji zuobiaxi zhong, x xiangqian (chengxiang fangxiang), y xiangyou
        # dan wurenji zuobiaxi zhong, x chaoqian, y chaozuo
        real_offset_forward = -corrected_pixel_offset_y * corrected_height / self.fy  # xiangji -y -> wurenji x (qianhou)
        real_offset_left = -corrected_pixel_offset_x * corrected_height / self.fx     # xiangji -x -> wurenji y (zuoyou)
        
        # huoqu wurenji dangqian weizhi he zitai
        drone_x = self.current_drone_odom.pose.pose.position.x
        drone_y = self.current_drone_odom.pose.pose.position.y
        drone_z = self.current_drone_odom.pose.pose.position.z
        
        # jiang mubiao pianyi zhuanhuan dao quanqiu zuobiaxi (x chaoqian, y chaozuo)
        # kaolv wurenji de yaw jiao jinxing zuobiaxi xuanzhuan
        global_offset_x = real_offset_forward * math.cos(yaw) - real_offset_left * math.sin(yaw)
        global_offset_y = real_offset_forward * math.sin(yaw) + real_offset_left * math.cos(yaw)
        
        # jisuan mubiao de quanqiu weizhi
        target_global_x = drone_x + global_offset_x
        target_global_y = drone_y + global_offset_y
        target_global_z = 0.0  # jiashe mubiao zai dimian shang
        
        # jisuan mubiao xiangdui yu wurenji de fangxiang jiao
        target_yaw = math.atan2(global_offset_y, global_offset_x)
        target_quaternion = tf.transformations.quaternion_from_euler(0, 0, target_yaw)
        
        # chuangjian bing fabu Odometry xiaoxi
        target_odom = Odometry()
        target_odom.header.stamp = rospy.Time.now()
        target_odom.header.frame_id = "map"  # quanqiu zuobiaxi
        target_odom.child_frame_id = "target"
        
        # shezhimubiao weizhi
        target_odom.pose.pose.position.x = target_global_x
        target_odom.pose.pose.position.y = target_global_y
        target_odom.pose.pose.position.z = target_global_z
        
        # shezhi mubiao zitai (sishu shu)
        target_odom.pose.pose.orientation.x = target_quaternion[0]
        target_odom.pose.pose.orientation.y = target_quaternion[1]
        target_odom.pose.pose.orientation.z = target_quaternion[2]
        target_odom.pose.pose.orientation.w = target_quaternion[3]
        
        # shezhi xiefang cha juzhen (jiashe mubiao jingzhi)
        target_odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                      0, 0.1, 0, 0, 0, 0,
                                      0, 0, 0.1, 0, 0, 0,
                                      0, 0, 0, 0.1, 0, 0,
                                      0, 0, 0, 0, 0.1, 0,
                                      0, 0, 0, 0, 0, 0.1]
        
        # shezhi sudu wei ling (jiashe mubiao jingzhi)
        target_odom.twist.twist.linear.x = 0.0
        target_odom.twist.twist.linear.y = 0.0
        target_odom.twist.twist.linear.z = 0.0
        target_odom.twist.twist.angular.x = 0.0
        target_odom.twist.twist.angular.y = 0.0
        target_odom.twist.twist.angular.z = 0.0
        
        # shezhi sudu xiefang cha juzhen
        target_odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                       0, 0.1, 0, 0, 0, 0,
                                       0, 0, 0.1, 0, 0, 0,
                                       0, 0, 0, 0, 0.1, 0,
                                       0, 0, 0, 0, 0.1, 0,
                                       0, 0, 0, 0, 0, 0.1]
        
        # fabu mubiao lichengji xinxi
        self.global_target_odom_pub.publish(target_odom)

    def determine_target_region(self, global_coords):
        """panduan mubiao suozai de quyu ID, can zhao B1-B7 A1-A9 fenge fanshi"""
        try:
            if global_coords is None or self.current_drone_odom is None:
                return None
            
            target_x, target_y = global_coords
            
            # da yin diao shi xin xi
            rospy.loginfo(f"Mubiao quanju zuobiao: x={target_x:.2f}, y={target_y:.2f}")
            
            # zuobiao xi ding yi: (0,0) wei A9 B1 zhong xin
            # X zhou: zheng fangxiang wei B1->B7 (dong fangxiang)
            # Y zhou: zheng fangxiang wei A9->A1 (bei fangxiang)
            # mei ge quyu 0.5m x 0.5m
            
            # jisuan xiang dui yu A9 B1 zhong xin de pian yi
            # B hang suo yin (0-6 dui ying B1-B7)
            if target_x >= 0:
                row = min(6, int((target_x + 0.25) / 0.5))  # jia 0.25 shi wei le zheng que ding wei dao fangge zhong xin
            else:
                row = 0  # fu X zhi ren wei zai B1
            
            # A lie suo yin (0-8 dui ying A9-A1)
            if target_y >= 0:
                col = min(8, int((target_y + 0.25) / 0.5))  # jia 0.25 shi wei le zheng que ding wei dao fangge zhong xin
            else:
                col = 0  # fu Y zhi ren wei zai A9
            
            # jisuan dang qian fangge de zhong xin zuo biao
            # A9 B1 zhong xin wei (0,0), qi ta fangge xiang ying pian yi
            center_x = row * 0.5  # B1=0, B2=0.5, B3=1.0, etc.
            center_y = col * 0.5  # A9=0, A8=0.5, A7=1.0, etc.
            
            # jisuan ju li
            distance = math.sqrt((target_x - center_x)**2 + (target_y - center_y)**2)
            
            # she zhi pan ding yue zhi
            self.target_region_threshold = 0.25  # 25cm nei ren wei zai gai quyu
            
            # sheng cheng quyu ID
            # B hang: B1=1, B7=7 (row+1)
            # A lie: A9=9, A1=1 (9-col)
            region_id = f"B{row+1} A{9-col}"
            
            # da yin diao shi xin xi
            rospy.loginfo(f"Jisuan quyu: {region_id}, zhongxin: ({center_x:.2f}, {center_y:.2f}), juli: {distance:.2f}m, yuzhi: {self.target_region_threshold}m")
            
            # ru guo ju li xiao yu yue zhi, ren wei zai zhe ge quyu nei
            if distance <= self.target_region_threshold:
                rospy.loginfo(f"Mubiao zai quyu: {region_id} nei")
                return region_id
            else:
                rospy.loginfo(f"Mubiao juli quyu: {region_id} tai yuan: {distance:.2f}m > {self.target_region_threshold}m")
            
            return None
        except Exception as e:
            rospy.logerr(f"Panduan quyu cuowu: {str(e)}")
            return None

    def publish_target_region_info(self):
        """fabu mubiao quyu ID xinxi"""
        try:
            if not self.detected_regions:
                return
            
            # gou jian shu ju
            region_data = {}
            for region_id, class_counts in self.detected_regions.items():
                region_data[region_id] = class_counts
            
            # zhuan huan wei JSON ge shi
            msg_data = json.dumps(region_data)
            
            # fabu xin xi
            self.target_region_pub.publish(String(msg_data))
            
            # qing chu ji lu, zhi fa bu yi ci
            self.detected_regions = {}
            
        except Exception as e:
            rospy.logerr(f"fabu quyu xinxi cuowu: {str(e)}")

    def __del__(self):
        cv2.destroyAllWindows()

def main():
    rospy.init_node('yolov8_ros', anonymous=True)
    try:
        yolo_dect = Yolo_Dect()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
