#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import Int32
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import yaml
import math

class WildlifeSurveyStation(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 初始化ROS节点
        rospy.init_node('wildlife_survey_station', anonymous=True)
        
        # 创建发布者
        self.command_pub = rospy.Publisher('/mission_command', Int32, queue_size=10)
        
        # 网格参数 - 坐标格式：B行 A列（如B1 A9）
        # 行：B1-B7（7行，B1在底部，B7在顶部）
        # 列：A1-A9（9列，A1在左侧，A9在右侧）
        self.grid_rows = 7  # 行数 (B1-B7，共7行)
        self.grid_cols = 9  # 列数 (A1-A9，共9列)
        
        # 红点位置 (起降点) - 设置在B1 A9位置
        self.red_point = (8, 6)  # B1 A9位置 (A9列B1行，即右下角)
        
        # 禁区列表
        self.forbidden_zones = []
        
        # 航点列表
        self.waypoints = []
        
        # 返回路径起始索引（用于区分巡查路径和返回路径）
        self.return_path_start_index = -1
        
        # 任务状态
        self.mission_active = False
        
        # 定时器用于发布命令
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_command)
        self.timer.start(200)  # 5Hz
        
        self.init_ui()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("野生动物巡查系统地面站")
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 创建地图部件
        self.map_widget = self.create_map_widget()
        main_layout.addWidget(self.map_widget, 2)
        
        # 创建控制面板
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, 1)
        
    def create_control_panel(self):
        """创建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 状态显示
        status_group = QGroupBox("系统状态")
        status_layout = QVBoxLayout(status_group)
        
        self.status_label = QLabel("状态: 待机")
        self.status_label.setStyleSheet("QLabel { font-size: 14px; font-weight: bold; }")
        
        self.waypoint_count_label = QLabel("路径长度: 0 格")
        
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.waypoint_count_label)
        
        layout.addWidget(status_group)
        
        # 禁区设置
        forbidden_group = QGroupBox("禁区设置")
        forbidden_layout = QVBoxLayout(forbidden_group)
        
        forbidden_layout.addWidget(QLabel("点击地图选择3个禁区:"))
        
        self.forbidden_status_label = QLabel("已选择: 0/3")
        forbidden_layout.addWidget(self.forbidden_status_label)
        
        clear_forbidden_btn = QPushButton("清除禁区")
        clear_forbidden_btn.clicked.connect(self.clear_forbidden_zones)
        forbidden_layout.addWidget(clear_forbidden_btn)
        
        layout.addWidget(forbidden_group)
        
        # 路径规划
        planning_group = QGroupBox("路径规划")
        planning_layout = QVBoxLayout(planning_group)
        
        plan_btn = QPushButton("规划最短闭环路径")
        plan_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        plan_btn.clicked.connect(self.plan_path)
        planning_layout.addWidget(plan_btn)

        # 添加算法说明
        info_label = QLabel("注：严格相邻移动，总路程最短，允许重复")
        info_label.setStyleSheet("QLabel { color: #666; font-size: 10px; }")
        planning_layout.addWidget(info_label)
        
        save_btn = QPushButton("保存航线")
        save_btn.clicked.connect(self.save_waypoints)
        planning_layout.addWidget(save_btn)
        
        layout.addWidget(planning_group)
        
        # 任务控制
        mission_group = QGroupBox("任务控制")
        mission_layout = QVBoxLayout(mission_group)
        
        self.start_btn = QPushButton("开始")
        self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 15px; }")
        self.start_btn.clicked.connect(self.toggle_mission)
        mission_layout.addWidget(self.start_btn)
        
        self.reset_btn = QPushButton("重置")
        self.reset_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 10px; }")
        self.reset_btn.clicked.connect(self.reset_mission)
        mission_layout.addWidget(self.reset_btn)
        
        layout.addWidget(mission_group)
        
        # 添加弹性空间
        layout.addStretch()
        
        return panel
        
    def create_map_widget(self):
        """创建地图显示部件"""
        return MapWidget(self)
        
    def coord_to_position(self, coord_str):
        """将坐标字符串转换为网格位置"""
        # 支持格式：B行 A列 (如 "B1 A9")
        parts = coord_str.strip().split()
        if len(parts) != 2:
            return None

        try:
            # 解析行部分 (B1-B7)
            row_part = parts[0].upper()
            if not row_part.startswith('B'):
                return None
            row_num = int(row_part[1:])
            if not (1 <= row_num <= 7):
                return None
            row = 7 - row_num  # B1对应索引6，B7对应索引0

            # 解析列部分 (A1-A9)
            col_part = parts[1].upper()
            if not col_part.startswith('A'):
                return None
            col_num = int(col_part[1:])
            if not (1 <= col_num <= 9):
                return None
            col = col_num - 1  # A1对应索引0，A9对应索引8

            return (col, row)

        except ValueError:
            return None
        
    def position_to_coord(self, col, row):
        """将网格位置转换为坐标字符串"""
        if 0 <= col < self.grid_cols and 0 <= row < self.grid_rows:
            # 行：B1-B7（索引6-0对应B1-B7）
            row_str = f"B{7 - row}"
            # 列：A1-A9（索引0-8对应A1-A9）
            col_str = f"A{col + 1}"
            return f"{row_str} {col_str}"  # 格式：B行 A列
        return None
        
    def grid_to_world(self, col, row):
        """将网格坐标转换为世界坐标（米）"""
        # 假设每个网格为10x10米，返回网格中心点坐标
        # 每个方块的中心点坐标为 (col*10+5, row*10+5)
        x = col * 10.0 + 5.0
        y = row * 10.0 + 5.0
        # 高度固定为1.2米
        z = 1.2
        return (x, y, z)
        
    def clear_forbidden_zones(self):
        """清除所有禁区"""
        self.forbidden_zones.clear()
        self.update_forbidden_status()
        self.map_widget.update()
        
    def update_forbidden_status(self):
        """更新禁区状态显示"""
        count = len(self.forbidden_zones)
        self.forbidden_status_label.setText(f"已选择: {count}/3")
        
    def add_forbidden_zone(self, col, row):
        """添加禁区"""
        if len(self.forbidden_zones) >= 3:
            QMessageBox.warning(self, "警告", "最多只能设置3个禁区")
            return False
        
        # 检查是否已经是禁区
        if (col, row) in self.forbidden_zones:
            return False
        
        # 检查是否是红点位置
        if (col, row) == self.red_point:
            QMessageBox.warning(self, "警告", "不能在起降点设置禁区")
            return False
        
        # 添加禁区
        self.forbidden_zones.append((col, row))
        self.update_forbidden_status()
        return True
        
    def find_detour_to_landing_point_with_repeats(self, start, target):
        """寻找从起点到目标点的绕行路径，允许重复航点"""
        # 首先尝试原有的绕行策略
        detour_path = self.find_detour_to_landing_point(start, target)
        if detour_path:
            return detour_path
        
        # 如果原有策略失败，允许通过已访问的航点进行绕行
        for waypoint in self.waypoints:
            if waypoint != start and waypoint not in self.forbidden_zones:
                # 检查从起点到已访问航点，再到目标点的路径是否安全
                if (not self.path_crosses_forbidden_zone(start, waypoint) and
                    not self.path_crosses_forbidden_zone(waypoint, target)):
                    return [waypoint, target]
        
        # 如果仍然失败，尝试通过多个已访问航点的组合
        for i, waypoint1 in enumerate(self.waypoints):
            if waypoint1 != start and waypoint1 not in self.forbidden_zones:
                for j, waypoint2 in enumerate(self.waypoints):
                    if (waypoint2 != start and waypoint2 != waypoint1 and 
                        waypoint2 not in self.forbidden_zones):
                        # 检查三段路径是否都安全
                        if (not self.path_crosses_forbidden_zone(start, waypoint1) and
                            not self.path_crosses_forbidden_zone(waypoint1, waypoint2) and
                            not self.path_crosses_forbidden_zone(waypoint2, target)):
                            return [waypoint1, waypoint2, target]
        
        return None
            
        if (col, row) == self.red_point:
            QMessageBox.warning(self, "警告", "不能将红点设为禁区")
            return False
            
        if (col, row) in self.forbidden_zones:
            QMessageBox.warning(self, "警告", "该位置已是禁区")
            return False
            
        self.forbidden_zones.append((col, row))
        self.update_forbidden_status()
        self.map_widget.update()
        return True
        
    def plan_path(self):
        """规划最短闭环路径 - 从起降点开始，遍历所有非禁区方块，最后回到起降点"""
        if len(self.forbidden_zones) < 3:
            QMessageBox.warning(self, "警告", "请先选择3个禁区！")
            return

        # 清空之前的航点和返回路径索引
        self.waypoints = []
        self.return_path_start_index = -1

        # 生成所有需要访问的点（除了禁区）
        all_points = []
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                point = (col, row)
                if point not in self.forbidden_zones:
                    all_points.append(point)

        # 确保起降点在可访问点列表中
        if self.red_point not in all_points:
            QMessageBox.warning(self, "警告", "起降点位置被禁区阻挡，无法规划路径！")
            return

        # 从起降点开始
        self.waypoints.append(self.red_point)

        # 从待访问点中移除起始点
        remaining_points = [p for p in all_points if p != self.red_point]

        # 使用最短路径算法遍历所有剩余点
        if not self.plan_shortest_coverage_path(remaining_points):
            QMessageBox.warning(self, "警告", "无法规划完整的遍历路径！")
            return

        # 添加简单的返回路径（方块到方块）
        if not self.add_simple_return_path():
            QMessageBox.warning(self, "警告", "无法规划返回路径！")
            return

        # 更新显示
        total_distance = self.calculate_total_distance()
        self.waypoint_count_label.setText(f"路径长度: {len(self.waypoints)} 格，总距离: {total_distance} 格")
        self.map_widget.update()

        # 验证是否遍历了所有非禁区点并形成闭环
        visited_points = set(self.waypoints)
        expected_points = set(all_points)
        is_closed_loop = (len(self.waypoints) >= 2 and
                         self.waypoints[0] == self.waypoints[-1] == self.red_point)

        if visited_points == expected_points and is_closed_loop:
            repeated_visits = self.count_repeated_visits()
            turn_count = self.calculate_turn_count()
            repeat_info = f"无重复访问" if repeated_visits == 0 else f"重复访问{repeated_visits}个方块"

            QMessageBox.information(self, "成功",
                f"最短闭环路径规划完成！\n"
                f"• 航点数量：{len(self.waypoints)}个\n"
                f"• 遍历方块：{len(all_points)}个（全覆盖）\n"
                f"• 总距离：{total_distance}格\n"
                f"• 转弯次数：{turn_count}次\n"
                f"• 重复情况：{repeat_info}\n"
                f"• 已形成闭环")
        else:
            missing_points = expected_points - visited_points
            if missing_points:
                QMessageBox.warning(self, "警告", f"路径规划不完整，遗漏了{len(missing_points)}个方块！")
            elif not is_closed_loop:
                QMessageBox.warning(self, "警告", "路径未形成闭环！")
        
    def find_optimal_start_point(self, all_points):
        """寻找最优起始点，强制从B1 A9位置开始（与降落点相同）"""
        # 强制起始点为B1 A9位置，与降落点相同 (A9列B1行，对应坐标(8, 6))
        start_point = self.red_point  # 使用与降落点相同的位置 B1 A9

        # 检查B1 A9位置是否在可访问点列表中
        if start_point in all_points:
            return start_point
        else:
            # 如果B1 A9位置不可访问（比如是禁区），返回None
            return None

    def plan_shortest_coverage_path(self, remaining_points):
        """规划总路程最短的路径，严格相邻移动，允许重复遍历"""
        if not remaining_points:
            return True

        unvisited = set(remaining_points)

        while unvisited:
            current_pos = self.waypoints[-1]

            # 找到到最近未访问点的最短路径（允许重复遍历）
            nearest_target, shortest_path = self.find_shortest_path_to_any_unvisited(current_pos, unvisited)

            if shortest_path:
                # 添加最短路径上的所有点（包括可能的重复点）
                for point in shortest_path:
                    self.waypoints.append(point)
                    if point in unvisited:
                        unvisited.remove(point)
            else:
                # 如果找不到路径，说明有问题
                break

        return len(unvisited) == 0

    def find_shortest_path_to_any_unvisited(self, start, unvisited):
        """找到到任何未访问点的最短路径，返回(目标点, 路径)"""
        best_target = None
        best_path = None
        min_distance = float('inf')

        # 对所有未访问点计算最短路径
        for target in unvisited:
            path = self.find_shortest_adjacent_path(start, target)
            if path and len(path) < min_distance:
                min_distance = len(path)
                best_target = target
                best_path = path

        return best_target, best_path

    def find_shortest_adjacent_path(self, start, end):
        """使用BFS寻找两点间的最短路径，严格相邻移动"""
        from collections import deque

        if start == end:
            return [end]

        queue = deque([(start, [start])])
        visited = {start}

        while queue:
            current, path = queue.popleft()

            # 检查四个相邻方向
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                next_col = current[0] + dx
                next_row = current[1] + dy
                next_point = (next_col, next_row)

                # 检查是否为有效的相邻方块
                if (0 <= next_col < self.grid_cols and
                    0 <= next_row < self.grid_rows and
                    next_point not in visited and
                    next_point not in self.forbidden_zones):

                    new_path = path + [next_point]

                    if next_point == end:
                        return new_path[1:]  # 不包括起始点

                    visited.add(next_point)
                    queue.append((next_point, new_path))

        return None  # 找不到路径

    def find_optimal_next_point(self, current_pos, unvisited):
        """找到最优的下一个点，考虑总路程最短和减少转弯"""
        # 获取当前移动方向
        current_direction = self.get_current_direction()

        # 寻找相邻的未访问点
        adjacent_candidates = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 上、右、下、左

        for i, (dx, dy) in enumerate(directions):
            next_col = current_pos[0] + dx
            next_row = current_pos[1] + dy
            next_point = (next_col, next_row)

            if (next_point in unvisited and
                self.is_safe_adjacent_move(current_pos, next_point)):

                # 计算选择这个点的总体代价
                total_cost = self.calculate_total_cost(next_point, unvisited, current_direction, i)
                adjacent_candidates.append((total_cost, next_point))

        if adjacent_candidates:
            # 选择总代价最小的点
            adjacent_candidates.sort(key=lambda x: x[0])
            return adjacent_candidates[0][1]

        return None

    def get_current_direction(self):
        """获取当前移动方向"""
        if len(self.waypoints) < 2:
            return None

        prev_point = self.waypoints[-2]
        current_point = self.waypoints[-1]

        dx = current_point[0] - prev_point[0]
        dy = current_point[1] - prev_point[1]

        # 返回方向索引：0=上, 1=右, 2=下, 3=左
        if dy == 1:
            return 0  # 上
        elif dx == 1:
            return 1  # 右
        elif dy == -1:
            return 2  # 下
        elif dx == -1:
            return 3  # 左

        return None

    def calculate_total_cost(self, candidate_point, unvisited, current_direction, move_direction):
        """计算选择某个点的总体代价（距离 + 转弯惩罚 + 剩余路径估算）"""
        # 基础移动代价
        base_cost = 1

        # 转弯惩罚
        turn_penalty = 0
        if current_direction is not None and current_direction != move_direction:
            turn_penalty = 0.5

        # 估算剩余路径长度（到最远点的距离）
        remaining_unvisited = unvisited - {candidate_point}
        if remaining_unvisited:
            max_distance_to_remaining = max(
                self.manhattan_distance(candidate_point, point)
                for point in remaining_unvisited
            )
            # 估算回到起点的距离
            distance_to_start = self.manhattan_distance(candidate_point, self.red_point)
            remaining_cost = max_distance_to_remaining + distance_to_start
        else:
            # 如果这是最后一个点，只考虑回到起点的距离
            remaining_cost = self.manhattan_distance(candidate_point, self.red_point)

        return base_cost + turn_penalty + remaining_cost * 0.1  # 给未来路径较小权重

    def find_shortest_path_to_unvisited(self, start, unvisited):
        """寻找到任何未访问点的最短路径"""
        # 按距离排序未访问点
        sorted_targets = sorted(unvisited,
                               key=lambda p: self.manhattan_distance(start, p))

        for target in sorted_targets:
            path = self.find_safe_path_between_points(start, target)
            if path:
                return path

        return None

    def find_best_next_point_no_repeat(self, current_pos, unvisited):
        """找到最佳的下一个点，优先考虑不重复访问"""
        # 首先尝试找到相邻的未访问点（最优选择）
        adjacent_points = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            next_col = current_pos[0] + dx
            next_row = current_pos[1] + dy
            next_point = (next_col, next_row)

            if (0 <= next_col < self.grid_cols and
                0 <= next_row < self.grid_rows and
                next_point in unvisited and
                next_point not in self.forbidden_zones):
                adjacent_points.append(next_point)

        if adjacent_points:
            # 如果有相邻的未访问点，选择距离起点最远的（避免过早回到起点附近）
            return max(adjacent_points,
                      key=lambda p: self.manhattan_distance(p, self.red_point))

        # 如果没有相邻点，寻找最近的未访问点
        return self.find_nearest_safe_point(current_pos, unvisited)

    def find_shortest_path_no_repeat(self, start, unvisited):
        """寻找到未访问点的最短路径，尽量避免重复访问已访问的方块"""
        visited_waypoints = set(self.waypoints)  # 已访问的方块

        # 按距离排序未访问点
        sorted_targets = sorted(unvisited,
                               key=lambda p: self.manhattan_distance(start, p))

        for target in sorted_targets:
            path = self.find_path_avoiding_visited(start, target, visited_waypoints)
            if path:
                return path

        return None

    def find_path_avoiding_visited(self, start, end, visited_waypoints):
        """寻找路径，尽量避开已访问的方块"""
        from collections import deque
        import heapq

        # A*算法，对已访问的方块增加惩罚
        open_set = [(0, start, [])]
        closed_set = set()

        while open_set:
            f_score, current, path = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)

            if current == end:
                return path + [end]

            # 检查四个方向的相邻点
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                next_col = current[0] + dx
                next_row = current[1] + dy
                next_point = (next_col, next_row)

                if (0 <= next_col < self.grid_cols and
                    0 <= next_row < self.grid_rows and
                    next_point not in closed_set and
                    next_point not in self.forbidden_zones):

                    g_score = len(path) + 1
                    h_score = self.manhattan_distance(next_point, end)

                    # 对已访问的方块增加惩罚（除了目标点）
                    penalty = 0
                    if next_point in visited_waypoints and next_point != end:
                        penalty = 100  # 重复访问惩罚

                    f_score = g_score + h_score + penalty

                    new_path = path + [next_point] if next_point != end else path
                    heapq.heappush(open_set, (f_score, next_point, new_path))

        return None

    def find_nearest_safe_point(self, current_pos, unvisited):
        """找到距离当前位置最近且可安全到达的点（严格检查，不从顶角穿过）"""
        nearest_point = None
        min_distance = float('inf')

        for point in unvisited:
            if self.is_safe_adjacent_move(current_pos, point):
                distance = self.manhattan_distance(current_pos, point)
                if distance < min_distance:
                    min_distance = distance
                    nearest_point = point

        return nearest_point

    def find_path_to_any_unvisited(self, start, unvisited):
        """寻找到任何未访问点的路径"""
        # 按距离排序未访问点
        sorted_targets = sorted(unvisited,
                               key=lambda p: self.manhattan_distance(start, p))

        for target in sorted_targets:
            path = self.find_safe_path_between_points(start, target)
            if path:
                return path

        return None

    def find_safe_path_between_points(self, start, end):
        """使用A*算法寻找两点间的安全路径（严格相邻移动，最短路程+最少转弯）"""
        from collections import deque
        import heapq

        # A*算法，状态包含位置和移动方向
        # 状态：(f_score, current, path, last_direction)
        open_set = [(0, start, [], None)]
        closed_set = set()

        while open_set:
            f_score, current, path, last_direction = heapq.heappop(open_set)

            # 状态键包含位置和方向，避免重复访问相同状态
            state_key = (current, last_direction)
            if state_key in closed_set:
                continue

            closed_set.add(state_key)

            if current == end:
                return path + [end]

            # 检查四个方向的严格相邻点
            directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 上、右、下、左

            for i, (dx, dy) in enumerate(directions):
                next_col = current[0] + dx
                next_row = current[1] + dy
                next_point = (next_col, next_row)

                # 严格的相邻移动检查
                if self.is_safe_adjacent_move(current, next_point):
                    next_state_key = (next_point, i)
                    if next_state_key not in closed_set:

                        g_score = len(path) + 1  # 实际距离
                        h_score = self.manhattan_distance(next_point, end)  # 启发式距离

                        # 转弯惩罚：如果方向改变，增加惩罚
                        turn_penalty = 0
                        if last_direction is not None and last_direction != i:
                            turn_penalty = 0.3  # 转弯惩罚权重（较小，优先考虑距离）

                        f_score = g_score + h_score + turn_penalty

                        new_path = path + [next_point] if next_point != end else path
                        heapq.heappush(open_set, (f_score, next_point, new_path, i))

        return None

    def is_safe_direct_path(self, start, end):
        """检查两点间的直接路径是否安全（方块到方块，不穿过禁区）"""
        # 只允许相邻方块间的移动（上下左右）
        dx = abs(end[0] - start[0])
        dy = abs(end[1] - start[1])

        # 只允许相邻方块移动
        if (dx == 1 and dy == 0) or (dx == 0 and dy == 1):
            return end not in self.forbidden_zones

        return False

    def is_safe_adjacent_move(self, start, end):
        """严格检查相邻方块移动是否安全（不从顶角穿过，不穿过禁区）"""
        # 检查是否在网格范围内
        if not (0 <= end[0] < self.grid_cols and 0 <= end[1] < self.grid_rows):
            return False

        # 检查目标点是否是禁区
        if end in self.forbidden_zones:
            return False

        # 检查是否是相邻移动（只允许上下左右）
        dx = end[0] - start[0]
        dy = end[1] - start[1]

        # 只允许相邻方块移动，不允许对角线
        if not ((dx == 1 and dy == 0) or (dx == -1 and dy == 0) or
                (dx == 0 and dy == 1) or (dx == 0 and dy == -1)):
            return False

        # 额外检查：确保移动路径不会"擦过"禁区的顶角
        # 这是最关键的检查，防止从顶角穿过
        return self.check_no_corner_crossing(start, end)

    def check_no_corner_crossing(self, start, end):
        """检查移动是否会从禁区顶角穿过"""
        start_col, start_row = start
        end_col, end_row = end

        # 对于相邻方块移动，检查是否会"擦过"禁区
        # 虽然是相邻移动，但仍需确保路径完全安全

        # 检查移动方向
        dx = end_col - start_col
        dy = end_row - start_row

        # 对于水平移动
        if dy == 0:
            # 检查移动路径上方和下方是否有禁区可能造成顶角穿越
            for check_dy in [-1, 1]:
                check_row = start_row + check_dy
                if 0 <= check_row < self.grid_rows:
                    # 检查起点和终点在垂直方向的相邻位置
                    if ((start_col, check_row) in self.forbidden_zones and
                        (end_col, check_row) in self.forbidden_zones):
                        # 如果起点和终点的上方（或下方）都是禁区，
                        # 这种移动可能会从禁区顶角穿过
                        return False

        # 对于垂直移动
        if dx == 0:
            # 检查移动路径左侧和右侧是否有禁区可能造成顶角穿越
            for check_dx in [-1, 1]:
                check_col = start_col + check_dx
                if 0 <= check_col < self.grid_cols:
                    # 检查起点和终点在水平方向的相邻位置
                    if ((check_col, start_row) in self.forbidden_zones and
                        (check_col, end_row) in self.forbidden_zones):
                        # 如果起点和终点的左侧（或右侧）都是禁区，
                        # 这种移动可能会从禁区顶角穿过
                        return False

        return True

    def manhattan_distance(self, p1, p2):
        """计算曼哈顿距离"""
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def add_simple_return_path(self):
        """添加简单的返回路径（严格相邻移动，最短路径）"""
        if not self.waypoints:
            return False

        # 记录返回路径的起始索引
        self.return_path_start_index = len(self.waypoints) - 1

        last_point = self.waypoints[-1]

        # 如果最后一个点已经是起降点，直接返回成功
        if last_point == self.red_point:
            return True

        # 使用严格相邻移动寻找最短返回路径
        return_path = self.find_shortest_adjacent_path(last_point, self.red_point)
        if return_path:
            for point in return_path:
                self.waypoints.append(point)
            return True

        return False

    def calculate_total_distance(self):
        """计算总路程（曼哈顿距离）"""
        if len(self.waypoints) < 2:
            return 0

        total_distance = 0
        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]
            distance = self.manhattan_distance(p1, p2)
            total_distance += distance

        return total_distance

    def count_repeated_visits(self):
        """统计重复访问的方块数量（不包括起降点的正常重复）"""
        if len(self.waypoints) < 2:
            return 0

        visit_count = {}
        for point in self.waypoints:
            visit_count[point] = visit_count.get(point, 0) + 1

        repeated_count = 0
        for point, count in visit_count.items():
            if point == self.red_point:
                # 起降点正常应该访问2次（起点和终点），超过2次才算重复
                if count > 2:
                    repeated_count += count - 2
            else:
                # 其他点超过1次就算重复
                if count > 1:
                    repeated_count += count - 1

        return repeated_count

    def calculate_turn_count(self):
        """计算路径中的转弯次数"""
        if len(self.waypoints) < 3:
            return 0

        turn_count = 0
        prev_direction = None

        for i in range(len(self.waypoints) - 1):
            current_point = self.waypoints[i]
            next_point = self.waypoints[i + 1]

            dx = next_point[0] - current_point[0]
            dy = next_point[1] - current_point[1]

            # 确定当前移动方向
            if dy == 1:
                current_direction = 0  # 上
            elif dx == 1:
                current_direction = 1  # 右
            elif dy == -1:
                current_direction = 2  # 下
            elif dx == -1:
                current_direction = 3  # 左
            else:
                continue  # 无效移动，跳过

            # 检查是否转弯
            if prev_direction is not None and prev_direction != current_direction:
                turn_count += 1

            prev_direction = current_direction

        return turn_count

    def plan_complete_coverage(self, remaining_points):
        """完全覆盖规划，确保访问所有剩余点，只允许90度转弯"""
        unvisited = set(remaining_points)
        
        while unvisited:
            # 从当前位置找到最佳的下一个点（只允许90度转弯）
            current_pos = self.waypoints[-1] if self.waypoints else self.red_point
            current_col, current_row = current_pos
            
            # 获取当前移动方向（如果有的话）
            current_direction = None
            if len(self.waypoints) >= 2:
                prev_pos = self.waypoints[-2]
                prev_col, prev_row = prev_pos
                
                if prev_col == current_col:  # 垂直移动
                    current_direction = 'vertical'
                elif prev_row == current_row:  # 水平移动
                    current_direction = 'horizontal'
            
            # 寻找可行的下一个点
            best_next = None
            best_score = float('inf')  # 较小的分数更好
            
            # 检查四个方向的相邻点
            directions = [
                (0, 1),   # 上
                (1, 0),   # 右
                (0, -1),  # 下
                (-1, 0)   # 左
            ]
            
            for dx, dy in directions:
                next_col = current_col + dx
                next_row = current_row + dy
                next_point = (next_col, next_row)
                
                # 检查点是否有效且未访问
                if (0 <= next_col < self.grid_cols and 
                    0 <= next_row < self.grid_rows and
                    next_point in unvisited and
                    next_point not in self.forbidden_zones and
                    next_point != self.red_point):
                    
                    # 检查路径是否安全
                    if not self.path_crosses_forbidden_zone(current_pos, next_point):
                        # 计算分数（优先考虑保持当前方向，其次考虑未访问点数量）
                        score = 0
                        
                        # 检查是否需要转弯
                        next_direction = 'horizontal' if dy == 0 else 'vertical'
                        if current_direction and current_direction != next_direction:
                            score += 100  # 转弯惩罚
                        
                        # 计算该方向上连续可访问的点数量（越多越好）
                        continuous_points = 0
                        check_col, check_row = next_col, next_row
                        while True:
                            check_col += dx
                            check_row += dy
                            check_point = (check_col, check_row)
                            
                            if (0 <= check_col < self.grid_cols and 
                                0 <= check_row < self.grid_rows and
                                check_point in unvisited and
                                check_point not in self.forbidden_zones and
                                check_point != self.red_point and
                                not self.path_crosses_forbidden_zone(next_point, check_point)):
                                continuous_points += 1
                            else:
                                break
                        
                        # 分数计算：转弯惩罚 - 连续点奖励
                        score -= continuous_points * 10
                        
                        if score < best_score:
                            best_score = score
                            best_next = next_point
            
            if best_next:
                self.waypoints.append(best_next)
                unvisited.remove(best_next)
            else:
                # 如果没有直接可达的点，尝试通过中间点到达
                if not self.find_path_through_intermediate(unvisited):
                    # 如果仍然无法安全到达任何剩余点，停止规划
                    break
                    
    def find_path_through_intermediate(self, unvisited):
        """通过中间点寻找到达未访问点的路径，确保完全避开禁飞区"""
        current_pos = self.waypoints[-1] if self.waypoints else self.red_point
        
        # 按距离排序未访问点，优先处理较近的点
        sorted_targets = sorted(unvisited, key=lambda p: abs(p[0] - current_pos[0]) + abs(p[1] - current_pos[1]))
        
        for target in sorted_targets:
            # 尝试找到安全的中间点路径
            best_path = self.find_safe_intermediate_path(current_pos, target, unvisited)
            if best_path:
                # 添加找到的安全路径
                for point in best_path:
                    if point in unvisited:
                        self.waypoints.append(point)
                        unvisited.discard(point)
                    elif point != current_pos:  # 避免重复添加当前位置
                        self.waypoints.append(point)
                return True
        return False
        
    def find_safe_intermediate_path(self, start, target, unvisited):
        """寻找从起点到目标点的安全中间路径"""
        # 尝试不同的中间点策略
        strategies = [
            self.try_direct_intermediate,
            self.try_corner_intermediate,
            self.try_edge_intermediate
        ]
        
        for strategy in strategies:
            path = strategy(start, target, unvisited)
            if path:
                return path
        return None
        
    def try_direct_intermediate(self, start, target, unvisited):
        """尝试直接中间点路径"""
        # 尝试所有可能的中间点
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                intermediate = (col, row)
                if (intermediate not in self.forbidden_zones and 
                    intermediate != self.red_point and
                    intermediate not in self.waypoints):
                    
                    # 检查从起点到中间点，再到目标点的路径是否安全
                    if (not self.path_crosses_forbidden_zone(start, intermediate) and
                        not self.path_crosses_forbidden_zone(intermediate, target)):
                        
                        if intermediate in unvisited:
                            return [intermediate, target]
                        else:
                            return [intermediate, target]
        return None
        
    def try_corner_intermediate(self, start, target, unvisited):
        """尝试通过角落点的路径"""
        start_col, start_row = start
        target_col, target_row = target
        
        # 尝试两个角落点：(start_col, target_row) 和 (target_col, start_row)
        corners = [(start_col, target_row), (target_col, start_row)]
        
        for corner in corners:
            if (corner not in self.forbidden_zones and 
                corner != self.red_point and
                corner not in self.waypoints):
                
                # 检查通过角落点的路径是否安全
                if (not self.path_crosses_forbidden_zone(start, corner) and
                    not self.path_crosses_forbidden_zone(corner, target)):
                    
                    if corner in unvisited:
                        return [corner, target]
                    else:
                        return [corner, target]
        return None
        
    def try_edge_intermediate(self, start, target, unvisited):
        """尝试通过边缘点的路径"""
        # 尝试沿着网格边缘寻找安全路径
        edge_points = []
        
        # 添加边缘点
        for col in range(self.grid_cols):
            edge_points.extend([(col, 0), (col, self.grid_rows-1)])
        for row in range(self.grid_rows):
            edge_points.extend([(0, row), (self.grid_cols-1, row)])
        
        # 移除重复点和禁区点
        edge_points = list(set(edge_points))
        edge_points = [p for p in edge_points if p not in self.forbidden_zones and p != self.red_point]
        
        for edge_point in edge_points:
            if edge_point not in self.waypoints:
                # 检查通过边缘点的路径是否安全
                if (not self.path_crosses_forbidden_zone(start, edge_point) and
                    not self.path_crosses_forbidden_zone(edge_point, target)):
                    
                    if edge_point in unvisited:
                        return [edge_point, target]
                    else:
                        return [edge_point, target]
        return None
                    
    def is_safe_path_to_point(self, target_point):
        """检查到目标点的路径是否安全（不经过禁区）"""
        if not self.waypoints:
            return True
            
        last_point = self.waypoints[-1]
        return not self.path_crosses_forbidden_zone(last_point, target_point)
        
    def path_crosses_forbidden_zone(self, start, end):
        """检查路径是否穿过禁区（包括禁区的边缘和顶点）"""
        start_col, start_row = start
        end_col, end_row = end

        # 如果起点或终点在禁区内，直接返回True
        if start in self.forbidden_zones or end in self.forbidden_zones:
            return True

        # 直接检查路径线段是否与任何禁区方块相交
        for forbidden_col, forbidden_row in self.forbidden_zones:
            # 检查路径是否与禁区方块相交
            if self.line_intersects_square(start_col, start_row, end_col, end_row,
                                         forbidden_col, forbidden_row):
                return True

        return False

    def line_intersects_square(self, x1, y1, x2, y2, square_col, square_row):
        """检查线段是否与方块相交（包括边缘和顶点）"""
        # 方块的边界（每个方块占据从(col, row)到(col+1, row+1)的区域）
        square_left = square_col
        square_right = square_col + 1
        square_top = square_row
        square_bottom = square_row + 1

        # 使用线段与矩形相交的算法
        # 检查线段是否与矩形的任何边相交

        # 如果线段的两个端点都在矩形的同一侧，则不相交
        if ((x1 < square_left and x2 < square_left) or
            (x1 > square_right and x2 > square_right) or
            (y1 < square_top and y2 < square_top) or
            (y1 > square_bottom and y2 > square_bottom)):
            return False

        # 如果线段的任一端点在矩形内，则相交
        if (square_left <= x1 <= square_right and square_top <= y1 <= square_bottom):
            return True
        if (square_left <= x2 <= square_right and square_top <= y2 <= square_bottom):
            return True

        # 检查线段是否与矩形的边相交
        # 使用参数方程检查相交
        dx = x2 - x1
        dy = y2 - y1

        if dx != 0:
            # 检查与左边和右边的相交
            t_left = (square_left - x1) / dx
            t_right = (square_right - x1) / dx

            for t in [t_left, t_right]:
                if 0 <= t <= 1:
                    y_intersect = y1 + t * dy
                    if square_top <= y_intersect <= square_bottom:
                        return True

        if dy != 0:
            # 检查与上边和下边的相交
            t_top = (square_top - y1) / dy
            t_bottom = (square_bottom - y1) / dy

            for t in [t_top, t_bottom]:
                if 0 <= t <= 1:
                    x_intersect = x1 + t * dx
                    if square_left <= x_intersect <= square_right:
                        return True

        return False
        
    def add_safe_landing_path(self):
        """添加安全的45度降落路径，允许返回时走重复航点"""
        if not self.waypoints:
            return False

        # 记录返回路径的起始索引
        self.return_path_start_index = len(self.waypoints) - 1

        last_point = self.waypoints[-1]
        red_col, red_row = self.red_point

        # 检查当前最后一个点是否已经是降落点
        if last_point == self.red_point:
            # 如果最后一个点已经是降落点，直接返回成功（起降点相同的情况）
            return True

        # 检查当前最后一个点是否已经在红点的45度对角线上
        last_col, last_row = last_point

        # 如果已经在45度对角线上且路径安全，直接降落
        if (abs(last_col - red_col) == abs(last_row - red_row) and
            not self.path_crosses_forbidden_zone(last_point, self.red_point)):
            self.waypoints.append(self.red_point)
            return True
        
        # 寻找能形成45度角且不经过禁区的降落点
        landing_candidates = []
        
        # 从红点向四个对角线方向寻找合适的降落点
        directions = [(-1, 1), (1, 1), (-1, -1), (1, -1)]  # 左上、右上、左下、右下
        
        for dx, dy in directions:
            for distance in range(1, max(self.grid_cols, self.grid_rows)):
                approach_col = red_col + dx * distance
                approach_row = red_row + dy * distance
                
                if (0 <= approach_col < self.grid_cols and 
                    0 <= approach_row < self.grid_rows and
                    (approach_col, approach_row) not in self.forbidden_zones):
                    
                    # 检查从降落点到红点的45度路径是否安全
                    if not self.path_crosses_forbidden_zone((approach_col, approach_row), self.red_point):
                        landing_candidates.append((approach_col, approach_row, distance))
        
        if landing_candidates:
            # 选择距离最近的合适降落点
            landing_candidates.sort(key=lambda x: x[2])
            approach_point = (landing_candidates[0][0], landing_candidates[0][1])
            
            # 检查从当前位置到降落点的直接路径是否安全
            if not self.path_crosses_forbidden_zone(last_point, approach_point):
                # 直接路径安全，添加降落点
                if approach_point != last_point:
                    self.waypoints.append(approach_point)
                self.waypoints.append(self.red_point)
                return True
            else:
                # 直接路径不安全，允许通过重复航点绕行到降落点
                detour_path = self.find_detour_to_landing_point_with_repeats(last_point, approach_point)
                if detour_path:
                    # 添加绕行路径（允许重复航点）
                    for point in detour_path:
                        if point != last_point:  # 避免重复添加当前位置
                            self.waypoints.append(point)
                    # 最后降落到红点
                    self.waypoints.append(self.red_point)
                    return True
        
        # 如果找不到安全的45度降落路径，尝试直接绕行到红点
        if not self.path_crosses_forbidden_zone(last_point, self.red_point):
            self.waypoints.append(self.red_point)
            return True
        else:
            # 需要绕行到红点，允许重复航点
            detour_path = self.find_detour_to_landing_point_with_repeats(last_point, self.red_point)
            if detour_path:
                for point in detour_path:
                    if point != last_point:
                        self.waypoints.append(point)
                return True
            
        return False
        
    def find_detour_to_landing_point(self, start, target):
        """寻找从起点到目标点的绕行路径，避开禁飞区"""
        # 尝试不同的绕行策略
        detour_strategies = [
            self.try_corner_detour,
            self.try_edge_detour,
            self.try_wide_detour
        ]
        
        for strategy in detour_strategies:
            path = strategy(start, target)
            if path:
                return path
        return None
        
    def try_corner_detour(self, start, target):
        """尝试通过角落点绕行"""
        start_col, start_row = start
        target_col, target_row = target
        
        # 尝试两个角落点：(start_col, target_row) 和 (target_col, start_row)
        corners = [(start_col, target_row), (target_col, start_row)]
        
        for corner in corners:
            if (0 <= corner[0] < self.grid_cols and 
                0 <= corner[1] < self.grid_rows and
                corner not in self.forbidden_zones and
                corner != self.red_point):
                
                # 检查通过角落点的路径是否安全
                if (not self.path_crosses_forbidden_zone(start, corner) and
                    not self.path_crosses_forbidden_zone(corner, target)):
                    return [corner, target]
        return None
        
    def try_edge_detour(self, start, target):
        """尝试通过边缘点绕行"""
        # 尝试沿着网格边缘寻找安全路径
        edge_points = []
        
        # 添加边缘点
        for col in range(self.grid_cols):
            edge_points.extend([(col, 0), (col, self.grid_rows-1)])
        for row in range(self.grid_rows):
            edge_points.extend([(0, row), (self.grid_cols-1, row)])
        
        # 移除重复点和禁区点
        edge_points = list(set(edge_points))
        edge_points = [p for p in edge_points if p not in self.forbidden_zones and p != self.red_point]
        
        # 按距离排序
        edge_points.sort(key=lambda p: abs(p[0] - start[0]) + abs(p[1] - start[1]))
        
        for edge_point in edge_points:
            # 检查通过边缘点的路径是否安全
            if (not self.path_crosses_forbidden_zone(start, edge_point) and
                not self.path_crosses_forbidden_zone(edge_point, target)):
                return [edge_point, target]
        return None
        
    def try_wide_detour(self, start, target):
        """尝试更大范围的绕行路径"""
        # 在更大范围内寻找中间点
        for distance in range(2, max(self.grid_cols, self.grid_rows)):
            for dx in range(-distance, distance + 1):
                for dy in range(-distance, distance + 1):
                    if abs(dx) + abs(dy) == distance:  # 曼哈顿距离为distance的点
                        intermediate_col = start[0] + dx
                        intermediate_row = start[1] + dy
                        intermediate = (intermediate_col, intermediate_row)
                        
                        if (0 <= intermediate_col < self.grid_cols and 
                            0 <= intermediate_row < self.grid_rows and
                            intermediate not in self.forbidden_zones and
                            intermediate != self.red_point):
                            
                            # 检查通过中间点的路径是否安全
                            if (not self.path_crosses_forbidden_zone(start, intermediate) and
                                not self.path_crosses_forbidden_zone(intermediate, target)):
                                return [intermediate, target]
        return None
         
    def save_waypoints(self):
        """保存航线到YAML文件"""
        if not self.waypoints:
            QMessageBox.warning(self, "警告", "请先规划路径！")
            return
            
        filename, _ = QFileDialog.getSaveFileName(
            self, "保存航线", "waypoints.yaml", "YAML files (*.yaml)")
            
        if filename:
            try:
                waypoints_data = {
                    'waypoints': [],
                    'metadata': {
                        'total_points': len(self.waypoints),
                        'grid_size': f"{self.grid_cols}x{self.grid_rows}",
                        'forbidden_zones': [self.position_to_coord(col, row) for col, row in self.forbidden_zones],
                        'red_point': self.position_to_coord(*self.red_point)
                    }
                }
                
                for i, (col, row) in enumerate(self.waypoints):
                    coord = self.position_to_coord(col, row)
                    world_x, world_y, world_z = self.grid_to_world(col, row)
                    
                    waypoint = {
                        'id': i + 1,
                        'coordinate': coord,
                        'grid_position': {'col': col, 'row': row},
                        'world_position': {'x': world_x, 'y': world_y, 'z': world_z},
                        'action': 'survey' if (col, row) != self.red_point else 'land'
                    }
                    waypoints_data['waypoints'].append(waypoint)
                    
                with open(filename, 'w', encoding='utf-8') as f:
                    yaml.dump(waypoints_data, f, default_flow_style=False, allow_unicode=True)
                    
                QMessageBox.information(self, "成功", f"航线已保存到 {filename}")
                
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败: {str(e)}")
                
    def toggle_mission(self):
        """切换任务状态"""
        self.mission_active = not self.mission_active
        
        if self.mission_active:
            self.start_btn.setText("停止")
            self.start_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 15px; }")
            self.status_label.setText("状态: 任务进行中")
        else:
            self.start_btn.setText("开始")
            self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 15px; }")
            self.status_label.setText("状态: 待机")
            
    def reset_mission(self):
        """重置任务"""
        self.mission_active = False
        self.start_btn.setText("开始")
        self.start_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; padding: 15px; }")
        self.status_label.setText("状态: 待机")
        
        # 清空航点
        self.waypoints.clear()
        self.return_path_start_index = -1
        self.waypoint_count_label.setText("路径长度: 0 格")
        
        # 清空禁区
        self.forbidden_zones.clear()
        self.update_forbidden_status()
        
        # 更新显示
        self.map_widget.update()
        
        QMessageBox.information(self, "信息", "任务已重置")
        
    def publish_command(self):
        """发布命令到ROS话题"""
        command = Int32()
        command.data = 1 if self.mission_active else 0
        self.command_pub.publish(command)

class MapWidget(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.setMinimumSize(600, 400)
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 计算绘制区域
        margin = 50
        draw_width = self.width() - 2 * margin
        draw_height = self.height() - 2 * margin
        
        # 计算网格大小
        cell_width = draw_width / self.parent.grid_cols
        cell_height = draw_height / self.parent.grid_rows
        
        # 绘制网格
        painter.setPen(QPen(Qt.black, 1))
        for i in range(self.parent.grid_cols + 1):
            x = margin + i * cell_width
            painter.drawLine(x, margin, x, margin + draw_height)
            
        for i in range(self.parent.grid_rows + 1):
            y = margin + i * cell_height
            painter.drawLine(margin, y, margin + draw_width, y)
            
        # 绘制坐标标签
        painter.setPen(QPen(Qt.black, 1))
        font = QFont()
        font.setPointSize(10)
        painter.setFont(font)
        
        # 列标签 (A1-A9) - 在底部水平显示
        for i in range(self.parent.grid_cols):
            x = margin + (i + 0.5) * cell_width - 8
            y = margin + draw_height + 20
            painter.drawText(x, y, f"A{i + 1}")
            
        # 行标签 (B7-B1) - 在左侧垂直显示，从上到下为B7到B1
        for i in range(self.parent.grid_rows):
            x = margin - 25
            y = margin + (i + 0.5) * cell_height + 5
            painter.drawText(x, y, f"B{7 - i}")
            
        # 绘制禁区
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.setPen(QPen(Qt.darkRed, 2))
        for col, row in self.parent.forbidden_zones:
            x = margin + col * cell_width
            y = margin + row * cell_height  # 直接使用row，因为索引0对应A9（顶部），索引8对应A1（底部）
            painter.drawRect(int(x), int(y), int(cell_width), int(cell_height))
            
        # 绘制红点(起降点)
        painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
        painter.setPen(QPen(Qt.darkRed, 3))
        red_col, red_row = self.parent.red_point
        x = margin + (red_col + 0.5) * cell_width
        y = margin + (red_row + 0.5) * cell_height  # 直接使用red_row
        painter.drawEllipse(int(x-10), int(y-10), 20, 20)
        
        # 绘制航线
        if len(self.parent.waypoints) > 1:
            for i in range(len(self.parent.waypoints) - 1):
                col1, row1 = self.parent.waypoints[i]
                col2, row2 = self.parent.waypoints[i + 1]
                
                x1 = margin + (col1 + 0.5) * cell_width
                y1 = margin + (row1 + 0.5) * cell_height  # 直接使用row1
                x2 = margin + (col2 + 0.5) * cell_width
                y2 = margin + (row2 + 0.5) * cell_height  # 直接使用row2
                
                # 判断是巡查路径还是返回路径
                if (self.parent.return_path_start_index >= 0 and 
                    i >= self.parent.return_path_start_index):
                    # 返回路径用紫色
                    painter.setPen(QPen(QColor(128, 0, 128), 3))  # 紫色
                else:
                    # 巡查路径用绿色
                    painter.setPen(QPen(Qt.green, 3))
                
                painter.drawLine(x1, y1, x2, y2)
                
                # 为巡查路径绘制箭头
                if (self.parent.return_path_start_index < 0 or 
                    i < self.parent.return_path_start_index):
                    self.draw_arrow(painter, x1, y1, x2, y2)
                
        # 绘制航点
        painter.setBrush(QBrush(Qt.green, Qt.SolidPattern))
        painter.setPen(QPen(Qt.darkGreen, 2))
        for i, (col, row) in enumerate(self.parent.waypoints):
            x = margin + (col + 0.5) * cell_width
            y = margin + (row + 0.5) * cell_height  # 直接使用row
            painter.drawEllipse(x-5, y-5, 10, 10)
            
            # 绘制航点编号
            painter.setPen(QPen(Qt.black, 1))
            painter.drawText(x+8, y+5, str(i+1))
            painter.setPen(QPen(Qt.darkGreen, 2))
    
    def draw_arrow(self, painter, x1, y1, x2, y2):
        """在线段上绘制箭头"""
        # 计算线段的方向向量
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        
        if length == 0:
            return
            
        # 单位方向向量
        ux = dx / length
        uy = dy / length
        
        # 箭头参数
        arrow_length = 15
        arrow_angle = math.pi / 6  # 30度
        
        # 箭头位置（线段中点）
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        
        # 计算箭头的两个端点
        cos_angle = math.cos(arrow_angle)
        sin_angle = math.sin(arrow_angle)
        
        # 箭头左端点
        left_x = mid_x - arrow_length * (ux * cos_angle + uy * sin_angle)
        left_y = mid_y - arrow_length * (uy * cos_angle - ux * sin_angle)
        
        # 箭头右端点
        right_x = mid_x - arrow_length * (ux * cos_angle - uy * sin_angle)
        right_y = mid_y - arrow_length * (uy * cos_angle + ux * sin_angle)
        
        # 绘制箭头
        painter.drawLine(mid_x, mid_y, left_x, left_y)
        painter.drawLine(mid_x, mid_y, right_x, right_y)
            
    def mousePressEvent(self, event):
        """处理鼠标点击事件"""
        if event.button() == Qt.LeftButton:
            # 计算点击位置对应的网格坐标
            margin = 50
            draw_width = self.width() - 2 * margin
            draw_height = self.height() - 2 * margin
            
            cell_width = draw_width / self.parent.grid_cols
            cell_height = draw_height / self.parent.grid_rows
            
            # 转换鼠标坐标到网格坐标
            x = event.x() - margin
            y = event.y() - margin
            
            if 0 <= x <= draw_width and 0 <= y <= draw_height:
                col = int(x / cell_width)
                row = int(y / cell_height)  # 直接使用y坐标，因为A9在顶部（索引0），A1在底部（索引8）
                
                # 确保坐标在有效范围内
                if 0 <= col < self.parent.grid_cols and 0 <= row < self.parent.grid_rows:
                    # 添加禁区
                    self.parent.add_forbidden_zone(col, row)
                    self.update()  # 重新绘制地图
             
if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    try:
        window = WildlifeSurveyStation()
        window.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass