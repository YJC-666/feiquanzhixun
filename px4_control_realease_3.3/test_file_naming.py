#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
文件命名机制测试脚本
用于验证航点接收器的递增文件命名功能
"""

import os
import yaml
from datetime import datetime

def create_test_files(save_path, count=5):
    """创建测试文件"""
    print(f"在 {save_path} 中创建 {count} 个测试文件...")
    
    # 确保目录存在
    os.makedirs(save_path, exist_ok=True)
    
    # 创建测试数据
    test_data = {
        'waypoints': [
            {
                'id': 1,
                'coordinate': 'B1 A9',
                'global_position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'action': 'takeoff'
            }
        ],
        'metadata': {
            'total_points': 1,
            'test_file': True,
            'created_at': datetime.now().isoformat()
        }
    }
    
    # 创建文件
    for i in range(1, count + 1):
        filename = f"waypoints_{i}.yaml"
        filepath = os.path.join(save_path, filename)
        
        # 添加文件特定信息
        test_data['metadata']['file_number'] = i
        test_data['metadata']['created_at'] = datetime.now().isoformat()
        
        with open(filepath, 'w', encoding='utf-8') as f:
            yaml.dump(test_data, f, default_flow_style=False, allow_unicode=True)
        
        print(f"  创建: {filename}")

def test_incremental_naming(save_path):
    """测试递增命名功能"""
    print(f"\n测试递增命名功能...")
    
    def generate_incremental_filename(base_name):
        """生成递增的文件名（复制自waypoint_receiver.py）"""
        counter = 1
        
        while True:
            filename = f"{base_name}_{counter}.yaml"
            filepath = os.path.join(save_path, filename)
            
            if not os.path.exists(filepath):
                return filename, counter
            
            counter += 1
            
            if counter > 9999:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                return f"{base_name}_{timestamp}.yaml", counter
    
    # 测试命名
    for i in range(3):
        filename, number = generate_incremental_filename("waypoints")
        print(f"  测试 {i+1}: 下一个文件名 = {filename} (序号: {number})")
        
        # 创建文件以测试下一次
        filepath = os.path.join(save_path, filename)
        with open(filepath, 'w') as f:
            f.write(f"# 测试文件 {number}\n")

def show_directory_contents(save_path):
    """显示目录内容"""
    print(f"\n目录内容 ({save_path}):")
    
    try:
        files = []
        for filename in os.listdir(save_path):
            if filename.startswith('waypoints_') and filename.endswith('.yaml'):
                filepath = os.path.join(save_path, filename)
                size = os.path.getsize(filepath)
                files.append((filename, size))
        
        if files:
            files.sort()
            print(f"  找到 {len(files)} 个航点文件:")
            total_size = 0
            for filename, size in files:
                print(f"    {filename} ({size} bytes)")
                total_size += size
            print(f"  总大小: {total_size} bytes")
        else:
            print("  没有找到航点文件")
            
    except Exception as e:
        print(f"  错误: {e}")

def cleanup_test_files(save_path):
    """清理测试文件"""
    print(f"\n清理测试文件...")
    
    try:
        count = 0
        for filename in os.listdir(save_path):
            if filename.startswith('waypoints_') and filename.endswith('.yaml'):
                filepath = os.path.join(save_path, filename)
                
                # 检查是否是测试文件
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        content = f.read()
                        if 'test_file' in content or '# 测试文件' in content:
                            os.remove(filepath)
                            print(f"  删除: {filename}")
                            count += 1
                except:
                    pass
        
        print(f"  共删除 {count} 个测试文件")
        
    except Exception as e:
        print(f"  清理时出错: {e}")

def main():
    """主函数"""
    print("=== 航点文件命名机制测试 ===")
    
    # 测试路径
    test_path = os.path.expanduser("~/catkin_ws/test_waypoints")
    
    print(f"测试路径: {test_path}")
    
    # 显示初始状态
    show_directory_contents(test_path)
    
    # 创建测试文件
    create_test_files(test_path, 3)
    
    # 显示创建后的状态
    show_directory_contents(test_path)
    
    # 测试递增命名
    test_incremental_naming(test_path)
    
    # 显示最终状态
    show_directory_contents(test_path)
    
    # 询问是否清理
    try:
        response = input("\n是否清理测试文件? (y/N): ").strip().lower()
        if response in ['y', 'yes']:
            cleanup_test_files(test_path)
            show_directory_contents(test_path)
        else:
            print("保留测试文件")
    except KeyboardInterrupt:
        print("\n测试中断")
    
    print("\n=== 测试完成 ===")

if __name__ == '__main__':
    main()
