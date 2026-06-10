#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
航点接收器 - 从ROS topic接收航点数据并保存为YAML文件
"""

import rospy
import json
import yaml
import os
from std_msgs.msg import String
from datetime import datetime

class WaypointReceiver:
    def __init__(self):
        """初始化航点接收器"""
        # 初始化ROS节点
        rospy.init_node('waypoint_receiver', anonymous=True)
        
        # 设置保存路径
        self.save_path = os.path.expanduser("/home/p/yy330_ws/yy330_sim_map_new/launch_sim/px4_control_realease_3.1/mission_log")
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            print(f"创建保存目录: {self.save_path}")
        
        # 订阅航点数据topic
        self.subscriber = rospy.Subscriber('/wildlife_survey/waypoints', String, self.waypoint_callback)
        
        print("航点接收器已启动")
        print(f"监听topic: /wildlife_survey/waypoints")
        print(f"保存路径: {self.save_path}")

        # 显示已有的航点文件
        self.show_existing_files()

        print("等待接收航点数据...")
    
    def waypoint_callback(self, msg):
        """处理接收到的航点数据"""
        try:
            print(f"\n收到航点数据，大小: {len(msg.data)} 字符")
            
            # 将JSON字符串转换回Python字典
            waypoints_data = json.loads(msg.data)
            
            # 验证数据格式
            if not self.validate_waypoint_data(waypoints_data):
                print("错误：接收到的数据格式无效")
                return
            
            # 生成文件名（递增后缀）
            filename = self.generate_incremental_filename("waypoints")
            filepath = os.path.join(self.save_path, filename)
            
            # 保存为YAML文件
            self.save_to_yaml(waypoints_data, filepath)
            
            # 显示接收信息
            self.display_received_info(waypoints_data, filepath)
            
        except json.JSONDecodeError as e:
            print(f"JSON解析错误: {e}")
        except Exception as e:
            print(f"处理航点数据时出错: {e}")

    def generate_incremental_filename(self, base_name):
        """生成递增的文件名"""
        # 基础文件名格式：waypoints_1.yaml, waypoints_2.yaml, ...
        counter = 1

        while True:
            filename = f"{base_name}_{counter}.yaml"
            filepath = os.path.join(self.save_path, filename)

            # 检查文件是否存在
            if not os.path.exists(filepath):
                print(f"生成文件名: {filename} (序号: {counter})")
                return filename

            counter += 1

            # 防止无限循环（虽然不太可能）
            if counter > 9999:
                print("警告：文件序号超过9999，使用时间戳")
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                return f"{base_name}_{timestamp}.yaml"

    def show_existing_files(self):
        """显示已有的航点文件"""
        try:
            # 查找所有waypoints_*.yaml文件
            waypoint_files = []
            for filename in os.listdir(self.save_path):
                if filename.startswith('waypoints_') and filename.endswith('.yaml'):
                    waypoint_files.append(filename)

            if waypoint_files:
                # 按文件名排序
                waypoint_files.sort()
                print(f"\n已有航点文件 ({len(waypoint_files)}个):")
                for i, filename in enumerate(waypoint_files, 1):
                    filepath = os.path.join(self.save_path, filename)
                    # 获取文件大小
                    try:
                        file_size = os.path.getsize(filepath)
                        size_kb = file_size / 1024
                        print(f"  {i}. {filename} ({size_kb:.1f}KB)")
                    except:
                        print(f"  {i}. {filename}")

                # 预测下一个文件名
                next_number = self.get_next_file_number()
                print(f"\n下一个文件将保存为: waypoints_{next_number}.yaml")
            else:
                print("\n目录中暂无航点文件")
                print("下一个文件将保存为: waypoints_1.yaml")

        except Exception as e:
            print(f"检查已有文件时出错: {e}")

    def get_next_file_number(self):
        """获取下一个文件序号"""
        counter = 1
        while True:
            filename = f"waypoints_{counter}.yaml"
            filepath = os.path.join(self.save_path, filename)
            if not os.path.exists(filepath):
                return counter
            counter += 1
            if counter > 9999:  # 防止无限循环
                return counter

    def validate_waypoint_data(self, data):
        """验证航点数据格式"""
        try:
            # 检查必要的字段
            if 'waypoints' not in data or 'metadata' not in data:
                return False
            
            # 检查航点列表
            waypoints = data['waypoints']
            if not isinstance(waypoints, list) or len(waypoints) == 0:
                return False
            
            # 检查第一个航点的格式
            first_waypoint = waypoints[0]
            required_fields = ['id', 'coordinate', 'global_position', 'action']
            for field in required_fields:
                if field not in first_waypoint:
                    return False
            
            return True
            
        except Exception:
            return False
    
    def save_to_yaml(self, data, filepath):
        """保存数据为YAML文件"""
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
            print(f"航点数据已保存到: {filepath}")
            
        except Exception as e:
            print(f"保存YAML文件失败: {e}")
            raise
    
    def display_received_info(self, data, filepath):
        """显示接收到的数据信息"""
        try:
            metadata = data.get('metadata', {})
            waypoints = data.get('waypoints', [])

            # 提取文件序号
            filename = os.path.basename(filepath)
            if '_' in filename and filename.endswith('.yaml'):
                try:
                    file_number = filename.split('_')[-1].replace('.yaml', '')
                    print(f"\n=== 接收到的航点信息 (第{file_number}次) ===")
                except:
                    print(f"\n=== 接收到的航点信息 ===")
            else:
                print(f"\n=== 接收到的航点信息 ===")

            print(f"文件保存位置: {filepath}")
            print(f"文件名: {filename}")
            print(f"总航点数量: {len(waypoints)}")
            
            # 显示网格信息
            if 'total_cells' in metadata:
                print(f"网格方格数: {metadata['total_cells']}")
            
            if 'cell_size' in metadata:
                cell_size = metadata['cell_size']
                print(f"方格大小: {cell_size.get('width_cm', 'N/A')}cm × {cell_size.get('height_cm', 'N/A')}cm")
            
            # 显示高度信息
            if 'height_settings' in metadata:
                height_settings = metadata['height_settings']
                print(f"起降高度: {height_settings.get('takeoff_landing_height_m', 'N/A')}米")
                print(f"巡查高度: {height_settings.get('survey_height_m', 'N/A')}米")
            
            # 显示坐标系信息
            if 'coordinate_system' in metadata:
                coord_sys = metadata['coordinate_system']
                print(f"坐标系原点: {coord_sys.get('origin', 'N/A')}")
                print(f"坐标单位: {coord_sys.get('units', 'N/A')}")
            
            # 统计航点类型
            action_counts = {}
            for waypoint in waypoints:
                action = waypoint.get('action', 'unknown')
                action_counts[action] = action_counts.get(action, 0) + 1
            
            print("航点类型统计:")
            for action, count in action_counts.items():
                print(f"  {action}: {count}个")
            
            # 显示起点和终点
            if waypoints:
                start_point = waypoints[0]
                end_point = waypoints[-1]
                print(f"起点: {start_point.get('coordinate', 'N/A')} - {start_point.get('action', 'N/A')}")
                print(f"终点: {end_point.get('coordinate', 'N/A')} - {end_point.get('action', 'N/A')}")
            
            print("=== 信息显示完成 ===")

            # 显示目录中的文件统计
            self.show_directory_summary()

        except Exception as e:
            print(f"显示信息时出错: {e}")

    def show_directory_summary(self):
        """显示目录文件统计"""
        try:
            waypoint_files = []
            total_size = 0

            for filename in os.listdir(self.save_path):
                if filename.startswith('waypoints_') and filename.endswith('.yaml'):
                    filepath = os.path.join(self.save_path, filename)
                    try:
                        file_size = os.path.getsize(filepath)
                        waypoint_files.append((filename, file_size))
                        total_size += file_size
                    except:
                        waypoint_files.append((filename, 0))

            if waypoint_files:
                print(f"\n📁 目录统计:")
                print(f"   总文件数: {len(waypoint_files)}个")
                print(f"   总大小: {total_size/1024:.1f}KB")
                print(f"   最新文件: {max(waypoint_files, key=lambda x: x[0])[0]}")

            print("")  # 空行分隔

        except Exception as e:
            print(f"显示目录统计时出错: {e}")
    
    def run(self):
        """运行接收器"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("\n接收器已停止")
        except Exception as e:
            print(f"运行时错误: {e}")

def main():
    """主函数"""
    try:
        # 创建并运行航点接收器
        receiver = WaypointReceiver()
        receiver.run()
        
    except rospy.ROSInterruptException:
        print("ROS中断")
    except Exception as e:
        print(f"程序错误: {e}")

if __name__ == '__main__':
    main()
