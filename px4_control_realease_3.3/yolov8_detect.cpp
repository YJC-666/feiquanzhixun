#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <memory>
#include <mutex>
#include <atomic>
#include <sstream>
#include <signal.h>
#include <csignal>
#include <map>
#include <unordered_map>
#include <cmath>

#include "yolov8.h"
#include "postprocess.h"
#include "yolov8_rknn_detect/YoloDetection.h"
#include "yolov8_rknn_detect/YoloDetections.h"
#include "yolov8_rknn_detect/YoloIDCombination.h"

// 前向声明函数
void publishIdCombination(const std::vector<DetectionResult>& detections, 
                        const std::vector<std::string>& class_labels,
                        ros::Publisher& publisher);

// 低通滤波器类
class LowPassFilter {
public:
    LowPassFilter(float alpha = 0.7f) : alpha_(alpha), initialized_(false) {}
    
    void setAlpha(float alpha) {
        alpha_ = std::max(0.0f, std::min(1.0f, alpha));
    }
    
    float getAlpha() const { return alpha_; }
    
    void reset() {
        initialized_ = false;
    }
    
    float filter(float value) {
        if (!initialized_) {
            filtered_value_ = value;
            initialized_ = true;
        } else {
            filtered_value_ = alpha_ * value + (1.0f - alpha_) * filtered_value_;
        }
        return filtered_value_;
    }
    
    // 添加const版本的获取当前值方法
    float getCurrentValue() const {
        return initialized_ ? filtered_value_ : 0.0f;
    }
    
    bool isInitialized() const { return initialized_; }

private:
    float alpha_;              // 滤波系数 (0-1)，越大响应越快
    mutable float filtered_value_;     // 当前滤波值，使用mutable以支持const方法
    mutable bool initialized_;         // 是否已初始化，使用mutable以支持const方法
};

// 目标追踪滤波器
class TargetTracker {
public:
    struct TrackedObject {
        int class_id;
        LowPassFilter x_filter;
        LowPassFilter y_filter;
        LowPassFilter width_filter;
        LowPassFilter height_filter;
        LowPassFilter confidence_filter;
        ros::Time last_update;
        int lost_frames;
        
        TrackedObject(float alpha, int cls_id = -1) 
            : class_id(cls_id), x_filter(alpha), y_filter(alpha), 
              width_filter(alpha), height_filter(alpha), confidence_filter(alpha),
              lost_frames(0) {}
    };
    
    TargetTracker(float alpha = 0.7f, float max_distance = 100.0f, int max_lost_frames = 10)
        : alpha_(alpha), max_distance_(max_distance), max_lost_frames_(max_lost_frames) {}
    
    void setParameters(float alpha, float max_distance, int max_lost_frames) {
        alpha_ = alpha;
        max_distance_ = max_distance;
        max_lost_frames_ = max_lost_frames;
        
        // 更新所有已存在目标的滤波器参数
        for (auto& pair : tracked_objects_) {
            pair.second.x_filter.setAlpha(alpha);
            pair.second.y_filter.setAlpha(alpha);
            pair.second.width_filter.setAlpha(alpha);
            pair.second.height_filter.setAlpha(alpha);
            pair.second.confidence_filter.setAlpha(alpha);
        }
    }
    
    std::vector<DetectionResult> updateAndFilter(const std::vector<DetectionResult>& detections) {
        ros::Time current_time = ros::Time::now();
        std::vector<DetectionResult> filtered_results;
        std::set<int> matched_ids;
        
        // 为每个检测结果找到最匹配的追踪目标
        for (const auto& detection : detections) {
            float center_x = detection.box.x + detection.box.width / 2.0f;
            float center_y = detection.box.y + detection.box.height / 2.0f;
            
            int best_id = -1;
            float min_distance = max_distance_;
            
            // 寻找最近的已追踪目标
            for (const auto& pair : tracked_objects_) {
                int id = pair.first;
                const auto& obj = pair.second;
                
                if (obj.class_id != detection.class_id || matched_ids.count(id)) {
                    continue;
                }
                
                if (obj.x_filter.isInitialized() && obj.y_filter.isInitialized()) {
                    float current_x = obj.x_filter.getCurrentValue();
                    float current_y = obj.y_filter.getCurrentValue();
                    float dx = center_x - current_x;
                    float dy = center_y - current_y;
                    float distance = std::sqrt(dx*dx + dy*dy);
                    
                    if (distance < min_distance) {
                        min_distance = distance;
                        best_id = id;
                    }
                }
            }
            
            TrackedObject* obj_ptr = nullptr;
            
            // 如果没找到匹配的目标，创建新的追踪对象
            if (best_id == -1) {
                best_id = next_id_++;
                auto result = tracked_objects_.emplace(best_id, TrackedObject(alpha_, detection.class_id));
                obj_ptr = &(result.first->second);
                ROS_DEBUG("创建新的追踪目标 ID: %d, 类别: %d", best_id, detection.class_id);
            } else {
                obj_ptr = &tracked_objects_.at(best_id);
            }
            
            matched_ids.insert(best_id);
            
            // 更新追踪对象
            obj_ptr->last_update = current_time;
            obj_ptr->lost_frames = 0;
            
            DetectionResult filtered_result;
            filtered_result.class_id = detection.class_id;
            
            // 应用低通滤波
            float filtered_x = obj_ptr->x_filter.filter(center_x);
            float filtered_y = obj_ptr->y_filter.filter(center_y);
            float filtered_width = obj_ptr->width_filter.filter(detection.box.width);
            float filtered_height = obj_ptr->height_filter.filter(detection.box.height);
            float filtered_confidence = obj_ptr->confidence_filter.filter(detection.prob);
            
            filtered_result.box.x = static_cast<int>(filtered_x - filtered_width / 2.0f);
            filtered_result.box.y = static_cast<int>(filtered_y - filtered_height / 2.0f);
            filtered_result.box.width = static_cast<int>(filtered_width);
            filtered_result.box.height = static_cast<int>(filtered_height);
            filtered_result.prob = filtered_confidence;
            
            filtered_results.push_back(filtered_result);
        }
        
        // 清理丢失的目标
        auto it = tracked_objects_.begin();
        while (it != tracked_objects_.end()) {
            if (!matched_ids.count(it->first)) {
                it->second.lost_frames++;
                if (it->second.lost_frames > max_lost_frames_) {
                    ROS_DEBUG("删除丢失的追踪目标 ID: %d", it->first);
                    it = tracked_objects_.erase(it);
                    continue;
                }
            }
            ++it;
        }
        
        ROS_DEBUG_THROTTLE(1.0, "当前追踪目标数量: %zu", tracked_objects_.size());
        return filtered_results;
    }
    
    void reset() {
        tracked_objects_.clear();
        next_id_ = 0;
        ROS_INFO("重置所有追踪目标");
    }
    
    size_t getTrackedObjectCount() const {
        return tracked_objects_.size();
    }

private:
    float alpha_;                                    // 滤波系数
    float max_distance_;                            // 最大匹配距离
    int max_lost_frames_;                           // 最大丢失帧数
    std::map<int, TrackedObject> tracked_objects_;  // 追踪对象映射
    int next_id_ = 0;                               // 下一个目标ID
};

// 全局变量
std::shared_ptr<Yolov8> yolo;
ros::Publisher detection_pub;
ros::Publisher robot_detection_pub;
ros::Publisher id_combination_pub;  // 添加ID组合发布者
ros::Publisher target_region_pub;   // 添加目标区域ID发布者
image_transport::Publisher image_pub;
float conf_threshold;
float nms_threshold;
bool only_best_detection;
std::vector<std::string> labels;
std::mutex yolo_mutex;
std::atomic<bool> shutdown_requested(false);
std::atomic<int> processed_frames(0);

// 高精度相机内参数据 - 支持动态校准
// 根据用户提供的标定数据更新内参
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
    402.5927201658822, 0.0, 328.4746212860611,
    0.0, 402.4039776127119, 252.28843849710458,
    0.0, 0.0, 1.0);

// 相机内参的分解参数，方便计算
double fx = 402.5927201658822;  // 焦距x
double fy = 402.4039776127119;  // 焦距y
double cx = 328.4746212860611;  // 主点x
double cy = 252.28843849710458;  // 主点y

// 高精度畸变系数 - 支持鱼眼和径向畸变校正
cv::Mat dist_coeffs = (cv::Mat_<double>(1, 8) << 
    27.512044551822996, -58.01859148281487, -0.0004558759195965709, -0.0015786230537342438, 
    23.14988772306903, 27.25062828920284, -57.926071562394746, 23.80020160847425);

// 高精度计算相关参数
struct HighPrecisionParams {
    bool enable_multi_frame_fusion = true;     // 启用多帧融合
    int fusion_window_size = 5;                // 融合窗口大小
    double outlier_threshold = 0.1;            // 异常值阈值(米)
    double temporal_weight_decay = 0.8;        // 时间权重衰减因子
    bool enable_adaptive_filtering = true;     // 启用自适应滤波
    double min_confidence_threshold = 0.5;     // 最小置信度阈值
} high_precision_params;

// 多帧坐标融合缓存
struct CoordinateFrame {
    std::pair<double, double> coordinates;
    ros::Time timestamp;
    double confidence;
    double height;
    tf::Quaternion orientation;
};
std::deque<CoordinateFrame> coordinate_history;
std::mutex coordinate_history_mutex;
    
// 无人机相关参数
nav_msgs::Odometry current_drone_odom;  // 无人机当前里程计信息
std::mutex drone_odom_mutex;            // 无人机里程计互斥锁
double drone_height = 0.0;              // 无人机距地面高度 (米) - 动态从里程计获取
double roi_size_real = 0.5;             // 检测区域边长 (米): 50cm*50cm
double target_region_threshold = 0.23;  // 目标区域判定阈值 (米)，与1.py保持一致
bool has_drone_odom = false;            // 是否接收到无人机里程计信息
ros::Subscriber drone_odom_sub;         // 无人机里程计订阅者
std::map<std::string, std::map<std::string, int>> detected_regions; // 记录检测到的区域信息

// 地图设置 - 与地面站一致
const int grid_rows = 7;  // 行数 (B1-B7，共7行)
const int grid_cols = 9;  // 列数 (A1-A9，共9列)
const double cell_size = 0.5;  // 每个方格边长50cm = 0.5米
// 坐标系原点: A9 B1 中心为 (0,0)

// 坐标系偏移量（地面站设置无人机初始位置为原点）
double offset_x = 0.0;
double offset_y = 0.0;
double offset_z = 0.0;

// 低通滤波器相关参数
bool enable_tracking_filter;
float filter_alpha;
float filter_max_distance;
int filter_max_lost_frames;
std::unique_ptr<TargetTracker> target_tracker;

// 信号处理函数
void sigintHandler(int sig) {
    ROS_INFO("收到关闭信号，正在安全退出...");
    shutdown_requested.store(true);
    ros::shutdown();
}

// 从文件加载标签
bool loadLabels(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("无法打开标签文件: %s", filename.c_str());
        return false;
    }
    
    labels.clear();
    std::string line;
    while (std::getline(file, line)) {
        // 移除行尾的空白字符
        line.erase(line.find_last_not_of(" \n\r\t") + 1);
        if (!line.empty()) {
            labels.push_back(line);
        }
    }
    
    if (labels.empty()) {
        ROS_ERROR("标签文件为空");
        return false;
    }
    
    ROS_INFO("加载了 %zu 个标签", labels.size());
    for (size_t i = 0; i < labels.size(); ++i) {
        ROS_DEBUG("标签[%zu]: %s", i, labels[i].c_str());
    }
    return true;
}

// 无人机里程计回调函数 - 完全对齐Python版本的实现
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    try {
        std::lock_guard<std::mutex> lock(drone_odom_mutex);
        
        // 首次获取无人机位置时初始化坐标系统
        if (!has_drone_odom) {
            // 记录无人机初始位置
            ROS_INFO("记录无人机初始位置: (%.2f, %.2f, %.2f)", 
                msg->pose.pose.position.x, 
                msg->pose.pose.position.y, 
                msg->pose.pose.position.z);
            
            // 初始化坐标系，使无人机初始位置对应于红点位置(B1 A9)
            // 参考cv_pro.py中的坐标系初始化方法
            // 对齐mission_dongwu.py中的initialize_coordinate_system方法
            offset_x = -msg->pose.pose.position.x;
            offset_y = -msg->pose.pose.position.y;
            offset_z = -msg->pose.pose.position.z;
            
            ROS_INFO("坐标系偏移量: X=%.2f, Y=%.2f, Z=%.2f", 
                offset_x, offset_y, offset_z);
                
            has_drone_odom = true;
        }
        
        // 保存当前里程计数据
        current_drone_odom = *msg;
        
        // 动态更新无人机高度 - 与Python版本一致
        double new_height = msg->pose.pose.position.z;
        
        // 首次获取高度或高度变化超过10cm时更新
        if (drone_height == 0.0 || fabs(new_height - drone_height) > 0.1) {
            drone_height = std::max(0.5, new_height);  // 最低高度0.5米，防止异常值
            ROS_INFO("无人机高度动态更新为: %.2f 米", drone_height);
        }
        
        // 获取姿态信息用于调试
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        ROS_DEBUG_THROTTLE(1.0, "无人机位置: (%.2f, %.2f, %.2f), 姿态(弧度): roll=%.2f, pitch=%.2f, yaw=%.2f",
                         msg->pose.pose.position.x, 
                         msg->pose.pose.position.y, 
                         msg->pose.pose.position.z,
                         roll, pitch, yaw);
    } catch (const std::exception& e) {
        ROS_ERROR("处理无人机里程计信息时发生错误: %s", e.what());
    }
}

// 多帧坐标融合函数 - 提高定位精度和稳定性
std::pair<double, double> fuseMultiFrameCoordinates(const std::pair<double, double>& current_coords, 
                                                   double confidence, double height, 
                                                   const tf::Quaternion& orientation) {
    std::lock_guard<std::mutex> lock(coordinate_history_mutex);
    
    if (!high_precision_params.enable_multi_frame_fusion) {
        return current_coords;
    }
    
    ros::Time current_time = ros::Time::now();
    
    // 添加当前帧到历史记录
    CoordinateFrame current_frame;
    current_frame.coordinates = current_coords;
    current_frame.timestamp = current_time;
    current_frame.confidence = confidence;
    current_frame.height = height;
    current_frame.orientation = orientation;
    
    coordinate_history.push_back(current_frame);
    
    // 保持窗口大小
    while (coordinate_history.size() > static_cast<size_t>(high_precision_params.fusion_window_size)) {
        coordinate_history.pop_front();
    }
    
    // 如果历史数据不足，直接返回当前坐标
    if (coordinate_history.size() < 2) {
        return current_coords;
    }
    
    // 计算加权平均坐标
    double total_weight = 0.0;
    double weighted_x = 0.0;
    double weighted_y = 0.0;
    
    for (size_t i = 0; i < coordinate_history.size(); ++i) {
        const auto& frame = coordinate_history[i];
        
        // 计算时间权重（越新的帧权重越大）
        double time_diff = (current_time - frame.timestamp).toSec();
        double temporal_weight = pow(high_precision_params.temporal_weight_decay, time_diff);
        
        // 计算置信度权重
        double confidence_weight = std::max(0.1, frame.confidence);
        
        // 计算高度一致性权重（高度变化小的权重大）
        double height_diff = fabs(frame.height - height);
        double height_weight = exp(-height_diff * 2.0);  // 高度差越小权重越大
        
        // 综合权重
        double total_frame_weight = temporal_weight * confidence_weight * height_weight;
        
        // 异常值检测
        double distance = sqrt(pow(frame.coordinates.first - current_coords.first, 2) + 
                              pow(frame.coordinates.second - current_coords.second, 2));
        
        if (distance > high_precision_params.outlier_threshold) {
            total_frame_weight *= 0.1;  // 降低异常值权重
            ROS_DEBUG("检测到异常坐标，距离: %.3f米，降低权重", distance);
        }
        
        weighted_x += frame.coordinates.first * total_frame_weight;
        weighted_y += frame.coordinates.second * total_frame_weight;
        total_weight += total_frame_weight;
    }
    
    if (total_weight > 0.0) {
        double fused_x = weighted_x / total_weight;
        double fused_y = weighted_y / total_weight;
        
        ROS_DEBUG("多帧融合结果 - 原始: (%.3f, %.3f), 融合: (%.3f, %.3f), 帧数: %zu", 
                 current_coords.first, current_coords.second, fused_x, fused_y, coordinate_history.size());
        
        return std::make_pair(fused_x, fused_y);
    }
    
    return current_coords;
}

// 高精度参数初始化函数
void initializeHighPrecisionParams() {
    // 从ROS参数服务器读取配置（如果存在）
    ros::NodeHandle nh("~");
    
    nh.param("high_precision/enable_multi_frame_fusion", 
             high_precision_params.enable_multi_frame_fusion, true);
    nh.param("high_precision/fusion_window_size", 
             high_precision_params.fusion_window_size, 5);
    nh.param("high_precision/outlier_threshold", 
             high_precision_params.outlier_threshold, 0.1);
    nh.param("high_precision/temporal_weight_decay", 
             high_precision_params.temporal_weight_decay, 0.8);
    nh.param("high_precision/enable_adaptive_filtering", 
             high_precision_params.enable_adaptive_filtering, true);
    nh.param("high_precision/min_confidence_threshold", 
             high_precision_params.min_confidence_threshold, 0.5);
    
    ROS_INFO("高精度参数初始化完成:");
    ROS_INFO("  - 多帧融合: %s", high_precision_params.enable_multi_frame_fusion ? "启用" : "禁用");
    ROS_INFO("  - 融合窗口大小: %d", high_precision_params.fusion_window_size);
    ROS_INFO("  - 异常值阈值: %.3f米", high_precision_params.outlier_threshold);
    ROS_INFO("  - 时间衰减因子: %.2f", high_precision_params.temporal_weight_decay);
    ROS_INFO("  - 自适应滤波: %s", high_precision_params.enable_adaptive_filtering ? "启用" : "禁用");
    ROS_INFO("  - 最小置信度阈值: %.2f", high_precision_params.min_confidence_threshold);
    
    // 清空历史坐标缓存
    std::lock_guard<std::mutex> lock(coordinate_history_mutex);
    coordinate_history.clear();
    
    ROS_INFO("高精度目标定位系统已就绪");
}

// 自适应滤波函数 - 根据环境动态调整参数
void adaptiveParameterTuning(double height, const tf::Quaternion& orientation, double confidence) {
    if (!high_precision_params.enable_adaptive_filtering) {
        return;
    }
    
    // 根据高度调整融合窗口大小
    if (height > 2.0) {
        high_precision_params.fusion_window_size = 7;  // 高度高时增加融合帧数
    } else if (height < 1.0) {
        high_precision_params.fusion_window_size = 3;  // 高度低时减少融合帧数
    } else {
        high_precision_params.fusion_window_size = 5;  // 默认值
    }
    
    // 根据姿态稳定性调整异常值阈值
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    double attitude_stability = 1.0 / (1.0 + fabs(roll) + fabs(pitch));
    
    if (attitude_stability > 0.8) {
        high_precision_params.outlier_threshold = 0.05;  // 姿态稳定时降低阈值
    } else if (attitude_stability < 0.5) {
        high_precision_params.outlier_threshold = 0.2;   // 姿态不稳定时提高阈值
    } else {
        high_precision_params.outlier_threshold = 0.1;   // 默认值
    }
    
    // 根据置信度调整时间衰减因子
    if (confidence > 0.8) {
        high_precision_params.temporal_weight_decay = 0.9;  // 高置信度时保持更多历史信息
    } else if (confidence < 0.6) {
        high_precision_params.temporal_weight_decay = 0.7;  // 低置信度时快速遗忘
    } else {
        high_precision_params.temporal_weight_decay = 0.8;  // 默认值
    }
    
    ROS_DEBUG_THROTTLE(5.0, "自适应参数调整 - 窗口大小: %d, 异常值阈值: %.3f, 衰减因子: %.2f", 
                      high_precision_params.fusion_window_size, 
                      high_precision_params.outlier_threshold,
                      high_precision_params.temporal_weight_decay);
}

// 高精度目标全局坐标计算 - 使用动态xyz、四元数和内参优化
std::pair<double, double> calculateGlobalCoordinates(const DetectionResult& detection) {
    std::lock_guard<std::mutex> lock(drone_odom_mutex);
    
    // 如果未收到无人机里程计信息，返回无效坐标
    if (!has_drone_odom) {
        ROS_WARN_THROTTLE(5.0, "尚未收到无人机里程计信息，无法计算全局坐标");
        return std::make_pair(0.0, 0.0);
    }
    
    try {
        // 计算目标在图像中的中心点
        float center_x = detection.box.x + detection.box.width / 2.0f;
        float center_y = detection.box.y + detection.box.height / 2.0f;
        
        ROS_DEBUG("目标中心点像素坐标: (%.1f, %.1f)", center_x, center_y);
        
        // === 步骤1: 畸变校正 ===
        // 将像素坐标转换为归一化坐标
        cv::Point2f pixel_point(center_x, center_y);
        std::vector<cv::Point2f> distorted_points = {pixel_point};
        std::vector<cv::Point2f> undistorted_points;
        
        // 使用OpenCV进行畸变校正
        cv::undistortPoints(distorted_points, undistorted_points, camera_matrix, dist_coeffs, cv::Mat(), camera_matrix);
        
        // 获取校正后的像素坐标
        double corrected_center_x = undistorted_points[0].x;
        double corrected_center_y = undistorted_points[0].y;
        
        ROS_DEBUG("畸变校正后像素坐标: (%.2f, %.2f)", corrected_center_x, corrected_center_y);
        
        // === 步骤2: 动态获取无人机状态 ===
        // 获取无人机当前位置（动态xyz）
        double drone_x = current_drone_odom.pose.pose.position.x;
        double drone_y = current_drone_odom.pose.pose.position.y;
        double drone_z = current_drone_odom.pose.pose.position.z;
        
        // 获取无人机当前姿态（动态四元数）
        auto orientation = current_drone_odom.pose.pose.orientation;
        tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        
        // 转换为欧拉角和旋转矩阵
        double roll, pitch, yaw;
        tf::Matrix3x3 rotation_matrix(q);
        rotation_matrix.getRPY(roll, pitch, yaw);
        
        ROS_DEBUG("无人机状态 - 位置: (%.3f, %.3f, %.3f), 姿态: roll=%.3f, pitch=%.3f, yaw=%.3f", 
                 drone_x, drone_y, drone_z, roll, pitch, yaw);
        
        // === 步骤3: 精确高度计算 ===
        // 使用实时Z坐标而非固定高度，提高精度
        double actual_height = drone_z;
        
        // 考虑无人机姿态对有效高度的影响
        // 当无人机倾斜时，相机到地面的垂直距离会发生变化
        double cos_roll = cos(roll);
        double cos_pitch = cos(pitch);
        double effective_height = actual_height / (cos_roll * cos_pitch);
        
        // 防止极端角度导致的异常值
        if (cos_roll * cos_pitch < 0.1) {
            effective_height = actual_height;
            ROS_WARN("无人机倾斜角度过大，使用原始高度: %.2f", actual_height);
        }
        
        ROS_DEBUG("高度计算 - 实际高度: %.3f, 有效高度: %.3f", actual_height, effective_height);
        
        // === 步骤4: 相机坐标系到无人机坐标系转换 ===
        // 计算目标相对于相机光心的归一化坐标
        double normalized_x = (corrected_center_x - cx) / fx;
        double normalized_y = (corrected_center_y - cy) / fy;
        
        // 在相机坐标系中的3D坐标（Z=effective_height）
        double camera_x = normalized_x * effective_height;
        double camera_y = normalized_y * effective_height;
        double camera_z = effective_height;
        
        ROS_DEBUG("相机坐标系中的3D坐标: (%.3f, %.3f, %.3f)", camera_x, camera_y, camera_z);
        
        // === 步骤5: 坐标系变换 ===
        // 相机坐标系到无人机坐标系的变换
        // 相机坐标系: X向右, Y向下, Z向前
        // 无人机坐标系: X向前, Y向左, Z向上
        double drone_frame_x = camera_z;   // 相机Z -> 无人机X (向前)
        double drone_frame_y = -camera_x;  // 相机-X -> 无人机Y (向左)
        double drone_frame_z = -camera_y;  // 相机-Y -> 无人机Z (向上)
        
        // === 步骤6: 姿态补偿 ===
        // 使用旋转矩阵进行精确的姿态补偿
        tf::Vector3 offset_vector(drone_frame_x, drone_frame_y, drone_frame_z);
        tf::Vector3 rotated_offset = rotation_matrix * offset_vector;
        
        // === 步骤7: 全局坐标计算 ===
        // 将补偿后的偏移量加到无人机当前位置
        double target_global_x = drone_x + rotated_offset.x();
        double target_global_y = drone_y + rotated_offset.y();
        
        ROS_INFO("高精度坐标计算 - 无人机位置: (%.3f, %.3f, %.3f), 目标全局坐标: (%.3f, %.3f)", 
                drone_x, drone_y, drone_z, target_global_x, target_global_y);
        
        // === 步骤8: 精度验证 ===
        // 计算投影误差以评估精度
        double projection_error = sqrt(pow(corrected_center_x - center_x, 2) + pow(corrected_center_y - center_y, 2));
        if (projection_error > 5.0) {  // 像素误差超过5像素时警告
            ROS_WARN("畸变校正误差较大: %.2f 像素", projection_error);
        }
        
        ROS_DEBUG("精度信息 - 畸变校正误差: %.2f像素, 有效高度比: %.3f", 
                 projection_error, effective_height / actual_height);
        
        // === 步骤9: 自适应参数调整 ===
        // 根据当前环境条件动态调整算法参数
        adaptiveParameterTuning(actual_height, q, detection.prob);
        
        // === 步骤10: 多帧坐标融合 ===
        // 使用历史帧信息提高定位精度和稳定性
        std::pair<double, double> raw_coordinates = std::make_pair(target_global_x, target_global_y);
        std::pair<double, double> fused_coordinates = fuseMultiFrameCoordinates(
            raw_coordinates, detection.prob, actual_height, q);
        
        // === 步骤11: 最终精度评估 ===
        double fusion_improvement = sqrt(pow(fused_coordinates.first - raw_coordinates.first, 2) + 
                                        pow(fused_coordinates.second - raw_coordinates.second, 2));
        
        if (fusion_improvement > 0.01) {  // 融合改进超过1cm时记录
            ROS_DEBUG("多帧融合改进: %.3f米", fusion_improvement);
        }
        
        // 计算总体精度指标
        double total_uncertainty = projection_error * effective_height / (fx + fy) * 0.5;  // 估算的位置不确定度
        
        ROS_INFO("高精度定位完成 - 最终坐标: (%.4f, %.4f), 估算精度: ±%.3f米, 置信度: %.2f", 
                fused_coordinates.first, fused_coordinates.second, total_uncertainty, detection.prob);
        
        // === 步骤12: 质量控制 ===
        // 检查结果的合理性
        if (fabs(fused_coordinates.first) > 10.0 || fabs(fused_coordinates.second) > 10.0) {
            ROS_WARN("检测到异常坐标值，可能存在计算错误: (%.3f, %.3f)", 
                    fused_coordinates.first, fused_coordinates.second);
        }
        
        if (detection.prob < high_precision_params.min_confidence_threshold) {
            ROS_WARN("目标置信度较低: %.2f，定位精度可能受影响", detection.prob);
        }
        
        return fused_coordinates;
        
    } catch (const std::exception& e) {
        ROS_ERROR("高精度坐标计算时发生错误: %s", e.what());
        return std::make_pair(0.0, 0.0);
    }
}

// 确定目标所在的区域ID - 完全对齐Python版本的实现
std::string determineTargetRegion(const std::pair<double, double>& global_coords) {
    try {
        // 如果未收到无人机里程计信息，返回空区域ID
        if (!has_drone_odom) {
            return "";
        }
        
        double target_x = global_coords.first;
        double target_y = global_coords.second;
        
        ROS_INFO("目标全局坐标: x=%.2f, y=%.2f", target_x, target_y);
        
        // 坐标系定义: (0,0) 为 A9 B1 中心，与1.py保持一致
        // X轴: 正方向为 B1->B7 (东方向)
        // Y轴: 正方向为 A9->A1 (北方向)
        // 每个区域 0.5m x 0.5m
        
        // 计算相对于 A9 B1 中心的偏移
        // B行索引 (0-6 对应B1-B7)
        int row;
        if (target_x >= 0) {
            row = std::min(6, static_cast<int>((target_x + 0.25) / 0.5));  // 加0.25是为了正确定位到方格中心
        } else {
            row = 0;  // 负X值认为在B1
        }
        
        // A列索引 (0-8 对应A9-A1)
        int col;
        if (target_y >= 0) {
            col = std::min(8, static_cast<int>((target_y + 0.25) / 0.5));  // 加0.25是为了正确定位到方格中心
        } else {
            col = 0;  // 负Y值认为在A9
        }
        
        // 计算当前方格的中心坐标
        // A9 B1 中心为 (0,0), 其他方格相应偏移
        double center_x = row * 0.5;  // B1=0, B2=0.5, B3=1.0, etc.
        double center_y = col * 0.5;  // A9=0, A8=0.5, A7=1.0, etc.
        
        // 计算距离
        double distance = sqrt(pow(target_x - center_x, 2) + pow(target_y - center_y, 2));
        
        // 设置判定阈值，与1.py保持一致
        double threshold = 0.25;  // 25cm内认为在该区域
        
        // 生成区域ID
        // B行: B1=1, B7=7 (row+1)
        // A列: A9=9, A1=1 (9-col)
        std::stringstream ss;
        ss << "B" << (row + 1) << " A" << (9 - col);
        std::string region_id = ss.str();
        
        ROS_INFO("计算区域: %s, 中心: (%.2f, %.2f), 距离: %.2fm, 阈值: %.2fm", 
                 region_id.c_str(), center_x, center_y, distance, threshold);
        
        // 如果距离小于阈值，认为在这个区域内
        if (distance <= threshold) {
            ROS_INFO("目标在区域: %s 内", region_id.c_str());
            return region_id;
        } else {
            ROS_INFO("目标距离区域: %s 太远: %.2fm > %.2fm", 
                     region_id.c_str(), distance, threshold);
        }
        
        return "";
    } catch (const std::exception& e) {
        ROS_ERROR("判断区域时出错: %s", e.what());
        return "";
    }
}

// 发布目标区域ID信息 - 完全匹配Python版本格式
void publishTargetRegionInfo(ros::Publisher& publisher) {
    try {
        if (detected_regions.empty() || publisher.getNumSubscribers() == 0) {
            return;
        }
        
        // 构建JSON格式字符串 - 与Python版本的publish_target_region_info对齐
        std::stringstream json_ss;
        json_ss << "{\"regions\": [";
        
        bool first_region = true;
        for (const auto& region_pair : detected_regions) {
            if (!first_region) {
                json_ss << ", ";
            }
            first_region = false;
            
            json_ss << "{\"id\": \"" << region_pair.first << "\", \"detections\": {";
            
            bool first_class = true;
            for (const auto& class_pair : region_pair.second) {
                if (!first_class) {
                    json_ss << ", ";
                }
                first_class = false;
                
                json_ss << "\"" << class_pair.first << "\": " << class_pair.second;
            }
            
            json_ss << "}}";
        }
        
        json_ss << "]}";
        
        // 发布消息
        std_msgs::String msg;
        msg.data = json_ss.str();
        publisher.publish(msg);
        
        ROS_INFO("发布目标区域信息: %s", msg.data.c_str());
        
        // 清除记录，只发布一次
        detected_regions.clear();
        
    } catch (const std::exception& e) {
        ROS_ERROR("发布区域信息错误: %s", e.what());
    }
}

// 图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 检查是否正在关闭
    if (shutdown_requested.load()) {
        return;
    }

    if (!yolo) {
        ROS_ERROR_ONCE("YOLO模型未初始化");
        return;
    }

    // 尝试获取互斥锁，如果失败则跳过此帧
    std::unique_lock<std::mutex> lock(yolo_mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
        ROS_WARN_THROTTLE(1.0, "跳过帧：推理正在进行中");
        return;
    }
    
    try {
        // 将ROS图像转换为OpenCV格式
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge异常: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        if (frame.empty()) {
            ROS_ERROR("收到空图像");
            return;
        }

        // 检查图像大小是否合理
        if (frame.cols <= 0 || frame.rows <= 0 || frame.cols > 4096 || frame.rows > 4096) {
            ROS_ERROR("图像尺寸不合理: %dx%d", frame.cols, frame.rows);
            return;
        }

        // 运行YOLO检测
        std::vector<DetectionResult> detections;
        cv::Mat vis_img;
        
        auto start_time = ros::Time::now();
        int ret = yolo->inference(frame, detections, vis_img, conf_threshold, nms_threshold);
        auto end_time = ros::Time::now();
        
        if (ret != 0) {
            ROS_ERROR("YOLOv8推理失败: %d", ret);
            return;
        }
        
        double inference_time = (end_time - start_time).toSec() * 1000.0;
        ROS_DEBUG("推理时间: %.2f ms", inference_time);

        // 应用追踪低通滤波（如果启用）
        if (enable_tracking_filter && target_tracker) {
            detections = target_tracker->updateAndFilter(detections);
            ROS_DEBUG_THROTTLE(5.0, "低通滤波已启用，当前追踪目标数: %zu", target_tracker->getTrackedObjectCount());
        }

        // 如果启用了only_best_detection，只保留置信度最高的检测结果
        DetectionResult* best_detection = nullptr;
        if (!detections.empty()) {
            auto best_it = std::max_element(detections.begin(), detections.end(),
                [](const DetectionResult& a, const DetectionResult& b) {
                    return a.prob < b.prob;
                });
            best_detection = &(*best_it);
            
            if (only_best_detection) {
                DetectionResult best = *best_it;
                detections.clear();
                detections.push_back(best);
            }
        }
        
        // 准备检测结果消息
        yolov8_rknn_detect::YoloDetections detection_msg;
        detection_msg.header = msg->header;
        
        // 在图像上绘制检测结果并发布机器人检测消息
        bool found_valid_detection = false;
        
        // 如果有有效检测，先发布最佳检测结果到/detect_for_robot
        if (best_detection != nullptr) {
            float x_center = best_detection->box.x + best_detection->box.width / 2.0f;
            float y_center = best_detection->box.y + best_detection->box.height / 2.0f;
            
            // 确保坐标在有效范围内
            x_center = std::max(0.0f, std::min(x_center, (float)vis_img.cols - 1));
            y_center = std::max(0.0f, std::min(y_center, (float)vis_img.rows - 1));
            
            std_msgs::String robot_msg;
            std::string class_name = (best_detection->class_id >= 0 && 
                                    best_detection->class_id < static_cast<int>(labels.size())) 
                                   ? labels[best_detection->class_id] 
                                   : "未知";
                                   
            std::stringstream ss;
            ss << class_name << ": x:" << static_cast<int>(x_center)
               << " y:" << static_cast<int>(y_center);
            robot_msg.data = ss.str();
            robot_detection_pub.publish(robot_msg);
            ROS_INFO_THROTTLE(1.0, "发布检测结果: %s", robot_msg.data.c_str());
            found_valid_detection = true;
        }

        // 处理所有检测结果
        for (const auto& det : detections) {
            // 检查检测框是否有效
            if (det.box.width <= 0 || det.box.height <= 0 ||
                det.box.x < 0 || det.box.y < 0 ||
                det.box.x + det.box.width > vis_img.cols ||
                det.box.y + det.box.height > vis_img.rows) {
                ROS_WARN("检测框无效: x=%d, y=%d, w=%d, h=%d", 
                         det.box.x, det.box.y, det.box.width, det.box.height);
                continue;
            }

            yolov8_rknn_detect::YoloDetection yolo_det;
            
            // 确保类别ID在范围内
            int class_id = det.class_id;
            if (class_id >= 0 && class_id < static_cast<int>(labels.size())) {
                yolo_det.class_name = labels[class_id];
            } else {
                yolo_det.class_name = "未知";
                ROS_WARN("类别ID超出范围: %d (max: %zu)", class_id, labels.size() - 1);
                class_id = 0;
            }
            
            yolo_det.class_id = class_id;
            yolo_det.confidence = det.prob;
            
            // 计算并检查中心点和宽高
            float x_center = det.box.x + det.box.width / 2.0f;
            float y_center = det.box.y + det.box.height / 2.0f;
            
            x_center = std::max(0.0f, std::min(x_center, (float)vis_img.cols - 1));
            y_center = std::max(0.0f, std::min(y_center, (float)vis_img.rows - 1));
            
            yolo_det.x_center = x_center;
            yolo_det.y_center = y_center;
            yolo_det.width = det.box.width;
            yolo_det.height = det.box.height;
            
            detection_msg.detections.push_back(yolo_det);
            
            try {
                // 在图像上绘制检测结果
                std::string label = yolo_det.class_name + " " + 
                                std::to_string(static_cast<int>(yolo_det.confidence * 100)) + "%";
                
                int baseline = 0;
                cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                
                // 确保绘制区域在图像范围内
                cv::Point text_org(std::max(det.box.x, 0),
                                std::max(det.box.y - baseline, label_size.height));
                
                // 计算目标全局坐标
                auto global_coords = calculateGlobalCoordinates(det);
                
                // 确定目标所在区域ID
                std::string region_id = determineTargetRegion(global_coords);
                
                // 记录区域ID和目标类别
                if (!region_id.empty()) {
                    if (detected_regions.find(region_id) == detected_regions.end()) {
                        detected_regions[region_id] = std::map<std::string, int>();
                    }
                    
                    if (detected_regions[region_id].find(yolo_det.class_name) == detected_regions[region_id].end()) {
                        detected_regions[region_id][yolo_det.class_name] = 0;
                    }
                    
                    detected_regions[region_id][yolo_det.class_name] += 1;
                }
                
                // 绘制边界框和标签
                cv::rectangle(vis_img, det.box, cv::Scalar(0, 255, 0), 2);
                cv::rectangle(vis_img, 
                            cv::Point(text_org.x, text_org.y - label_size.height),
                            cv::Point(std::min(text_org.x + label_size.width, vis_img.cols - 1), text_org.y),
                            cv::Scalar(0, 255, 0), -1);
                cv::putText(vis_img, label, text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
                
                // 添加实时坐标信息 - 与Python版本一致的格式
                std::stringstream xy_ss;
                xy_ss << "XY: (" << static_cast<int>(x_center) << "," << static_cast<int>(y_center) << ")";
                cv::Point xy_text_org(det.box.x, det.box.y + det.box.height + 15);
                cv::putText(vis_img, xy_ss.str(), xy_text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                
                // 添加全局坐标信息
                std::stringstream global_ss;
                global_ss << "Global: (" << std::fixed << std::setprecision(2) 
                          << global_coords.first << "," << global_coords.second << ")";
                cv::Point global_text_org(det.box.x, det.box.y + det.box.height + 35);
                cv::putText(vis_img, global_ss.str(), global_text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
                
                // 添加区域ID信息 - 与Python版本完全一致的格式
                if (!region_id.empty()) {
                    std::stringstream region_ss;
                    region_ss << "ID: " << region_id;  // 使用"ID:"前缀，与Python版本一致
                    cv::Point region_text_org(det.box.x, det.box.y + det.box.height + 55);
                    cv::putText(vis_img, region_ss.str(), region_text_org,
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
                } else {
                    cv::Point region_text_org(det.box.x, det.box.y + det.box.height + 55);
                    cv::putText(vis_img, "ID: No ID", region_text_org,  // 使用"ID: No ID"，与Python版本一致
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
                }
            } catch (const cv::Exception& e) {
                ROS_WARN("OpenCV绘制异常: %s", e.what());
                continue;
            }
        }
        
        // 发布检测结果消息
        detection_pub.publish(detection_msg);
        
        // 发布ID组合信息 - 增加try-catch以防止崩溃
        try {
            if (!detections.empty() && !labels.empty()) {
                publishIdCombination(detections, labels, id_combination_pub);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("发布ID组合信息时出错: %s", e.what());
        } catch (...) {
            ROS_ERROR("发布ID组合信息时出现未知错误");
        }
        
        // 发布区域ID信息
        try {
            if (!detected_regions.empty()) {
                publishTargetRegionInfo(target_region_pub);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("发布区域信息时出错: %s", e.what());
        } catch (...) {
            ROS_ERROR("发布区域信息时出现未知错误");
        }
        
        // 如果没有检测到目标，发布not_found消息
        if (!found_valid_detection) {
            std_msgs::String robot_msg;
            robot_msg.data = "not_found: x:-1 y:-1";
            robot_detection_pub.publish(robot_msg);
            ROS_INFO_THROTTLE(10.0, "未检测到目标");
        }
        
        // 发布包含检测框的图像
        if (image_pub.getNumSubscribers() > 0) {
            try {
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "bgr8", vis_img).toImageMsg();
                image_pub.publish(img_msg);
            } catch (const cv::Exception& e) {
                ROS_ERROR("图像发布异常: %s", e.what());
            }
        }
        
        // 更新处理帧数
        int frames = processed_frames.fetch_add(1) + 1;
        if (frames % 100 == 0) {
            ROS_INFO("已处理 %d 帧", frames);
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR("处理图像时发生异常: %s", e.what());
    }
    catch (...) {
        ROS_ERROR("处理图像时发生未知异常");
    }
}

// 发布ID组合信息的函数
void publishIdCombination(const std::vector<DetectionResult>& detections, 
                         const std::vector<std::string>& class_labels,
                         ros::Publisher& publisher) {
    // 进行一些基本检查以确保安全
    if (detections.empty() || class_labels.empty()) {
        return;
    }
    
    // 如果没有订阅者，也无需处理
    if (publisher.getNumSubscribers() == 0) {
        return;
    }

    // 统计各类别的数量
    std::unordered_map<std::string, int> class_counts;
    for (const auto& det : detections) {
        // 确保类别ID在有效范围内
        if (det.class_id >= 0 && det.class_id < static_cast<int>(class_labels.size())) {
            std::string class_name = class_labels[det.class_id];
            class_counts[class_name]++;
        } else {
            class_counts["未知"]++;
        }
    }
    
    // 构建消息
    yolov8_rknn_detect::YoloIDCombination id_msg;
    id_msg.header.stamp = ros::Time::now();
    id_msg.header.frame_id = "camera"; // 使用相机坐标系
    
    std::stringstream combined_ss;
    
    // 填充消息内容
    for (const auto& pair : class_counts) {
        id_msg.ids.push_back(pair.first);
        id_msg.counts.push_back(pair.second);
        
        // 添加到合并消息中
        if (!combined_ss.str().empty()) {
            combined_ss << " ";
        }
        combined_ss << pair.first << ":" << pair.second;
    }
    
    id_msg.combined_message = combined_ss.str();
    
    // 发布消息
    publisher.publish(id_msg);
    ROS_INFO_THROTTLE(1.0, "发布ID组合: %s", id_msg.combined_message.c_str());
}

int main(int argc, char** argv) {
    try {
        setlocale(LC_CTYPE, "zh_CN.utf8");
        ros::init(argc, argv, "yolov8_rknn_detect_node", ros::init_options::NoSigintHandler);
        
        // 设置信号处理器
        signal(SIGINT, sigintHandler);
        
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // 获取基本参数
        std::string model_path;
        std::string label_path;
        std::string input_topic;
        std::string output_topic;
        std::string robot_topic;
        std::string image_topic;
        
        private_nh.param<std::string>("model_path", model_path, "model.rknn");
        private_nh.param<std::string>("label_path", label_path, "labels.txt");
        private_nh.param<std::string>("input_topic", input_topic, "/usb_cam/image_raw");
        private_nh.param<std::string>("output_topic", output_topic, "/yolov8_rknn/detection_result");
        private_nh.param<std::string>("robot_topic", robot_topic, "/detect_for_robot");
        private_nh.param<std::string>("image_topic", image_topic, "/yolov8_rknn/detection_image");
        
        // 获取检测参数
        private_nh.param<float>("conf_threshold", conf_threshold, 0.5f);
        private_nh.param<float>("nms_threshold", nms_threshold, 0.45f);
        private_nh.param<bool>("only_best_detection", only_best_detection, false);
        
        // 获取低通滤波器参数
        private_nh.param<bool>("enable_tracking_filter", enable_tracking_filter, false);
        private_nh.param<float>("filter_alpha", filter_alpha, 0.7f);
        private_nh.param<float>("filter_max_distance", filter_max_distance, 100.0f);
        private_nh.param<int>("filter_max_lost_frames", filter_max_lost_frames, 10);
        
        // 获取相机内参参数 (可选，已有默认值)
        try {
            private_nh.param<double>("camera_fx", fx, camera_matrix.at<double>(0, 0));
            private_nh.param<double>("camera_fy", fy, camera_matrix.at<double>(1, 1));
            private_nh.param<double>("camera_cx", cx, camera_matrix.at<double>(0, 2));
            private_nh.param<double>("camera_cy", cy, camera_matrix.at<double>(1, 2));
            
            // 确保参数有效
            if (fx > 0 && fy > 0 && cx > 0 && cy > 0) {
                // 更新相机内参矩阵
                camera_matrix.at<double>(0, 0) = fx;
                camera_matrix.at<double>(1, 1) = fy;
                camera_matrix.at<double>(0, 2) = cx;
                camera_matrix.at<double>(1, 2) = cy;
            } else {
                ROS_WARN("相机内参参数无效，使用默认值");
            }
            
            // 获取区域判断相关参数
            private_nh.param<double>("drone_height", drone_height, 1.22);
            private_nh.param<double>("roi_size_real", roi_size_real, 0.5);
            private_nh.param<double>("target_region_threshold", target_region_threshold, 0.3);
            
            // 获取调试级别
            int debug_level;
            private_nh.param<int>("debug_level", debug_level, 1);
            if (debug_level >= 2) {
                // 设置ROS调试级别为DEBUG
                if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
                    ros::console::notifyLoggerLevelsChanged();
                    ROS_INFO("已启用详细调试输出");
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("处理参数时出错: %s", e.what());
        }
        
        // 验证参数
        if (conf_threshold < 0.0f || conf_threshold > 1.0f) {
            ROS_WARN("置信度阈值超出范围[0,1]，使用默认值0.5");
            conf_threshold = 0.5f;
        }
        
        if (nms_threshold < 0.0f || nms_threshold > 1.0f) {
            ROS_WARN("NMS阈值超出范围[0,1]，使用默认值0.45");
            nms_threshold = 0.45f;
        }
        
        // 验证滤波器参数
        if (filter_alpha < 0.0f || filter_alpha > 1.0f) {
            ROS_WARN("滤波器Alpha参数超出范围[0,1]，使用默认值0.7");
            filter_alpha = 0.7f;
        }
        
        if (filter_max_distance <= 0.0f) {
            ROS_WARN("滤波器最大距离参数无效，使用默认值100.0");
            filter_max_distance = 100.0f;
        }
        
        if (filter_max_lost_frames <= 0) {
            ROS_WARN("滤波器最大丢失帧数参数无效，使用默认值10");
            filter_max_lost_frames = 10;
        }
        
        // 输出参数信息
        ROS_INFO("=== YOLOv8 RKNN 检测节点参数 ===");
        ROS_INFO("模型路径: %s", model_path.c_str());
        ROS_INFO("标签路径: %s", label_path.c_str());
        ROS_INFO("输入话题: %s", input_topic.c_str());
        ROS_INFO("输出话题: %s", output_topic.c_str());
        ROS_INFO("机器人检测话题: %s", robot_topic.c_str());
        ROS_INFO("图像输出话题: %s", image_topic.c_str());
        ROS_INFO("置信度阈值: %.2f", conf_threshold);
        ROS_INFO("NMS阈值: %.2f", nms_threshold);
        ROS_INFO("只保留最佳检测结果: %s", only_best_detection ? "是" : "否");
        
        ROS_INFO("=== 追踪滤波器参数 ===");
        ROS_INFO("启用追踪滤波器: %s", enable_tracking_filter ? "是" : "否");
        if (enable_tracking_filter) {
            ROS_INFO("滤波器Alpha系数: %.2f (%.2f为无滤波, %.2f为强滤波)", 
                     filter_alpha, 1.0f, 0.1f);
            ROS_INFO("最大匹配距离: %.1f 像素", filter_max_distance);
            ROS_INFO("最大丢失帧数: %d 帧", filter_max_lost_frames);
        }
        
        // 输出相机参数信息
        try {
            ROS_INFO("=== 相机内参 ===");
            ROS_INFO("fx: %.6f", camera_matrix.at<double>(0, 0));
            ROS_INFO("fy: %.6f", camera_matrix.at<double>(1, 1));
            ROS_INFO("cx: %.6f", camera_matrix.at<double>(0, 2));
            ROS_INFO("cy: %.6f", camera_matrix.at<double>(1, 2));
            ROS_INFO("相机模型: Brown-Conrady（根据标定结果）");
        } catch (const std::exception& e) {
            ROS_ERROR("输出相机参数时出错: %s", e.what());
        }
        
        // 加载标签
        if (!loadLabels(label_path)) {
            ROS_ERROR("无法加载标签文件，退出");
            return -1;
        }
        
        // 初始化YOLO模型
        yolo = std::make_shared<Yolov8>();
        int ret = yolo->load_model(model_path.c_str());
        if (ret != 0) {
            ROS_ERROR("无法加载RKNN模型，退出");
            return -1;
        }
        
        // 设置类别数量
        yolo->set_class_num(static_cast<int>(labels.size()));
        ROS_INFO("设置模型类别数量: %d", yolo->get_class_num());
        
        // 初始化追踪滤波器
        if (enable_tracking_filter) {
            target_tracker = std::make_unique<TargetTracker>(filter_alpha, filter_max_distance, filter_max_lost_frames);
            ROS_INFO("追踪滤波器初始化成功");
        }
        
        ROS_INFO("RKNN模型加载成功");
        
        // 设置订阅和发布
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber image_sub = it.subscribe(input_topic, 1, imageCallback);
        image_pub = it.advertise(image_topic, 1);
        detection_pub = nh.advertise<yolov8_rknn_detect::YoloDetections>(output_topic, 1);
        robot_detection_pub = nh.advertise<std_msgs::String>(robot_topic, 1);
        id_combination_pub = nh.advertise<yolov8_rknn_detect::YoloIDCombination>("/yolov8_rknn/id_combination", 1);
        target_region_pub = nh.advertise<std_msgs::String>("/yolov8_rknn/target_region_info", 1);
        
        // 订阅无人机里程计信息
        std::string drone_odom_topic;
        private_nh.param<std::string>("drone_odom_topic", drone_odom_topic, "/mavros/local_position/odom");
        drone_odom_sub = nh.subscribe(drone_odom_topic, 1, odomCallback);
        
        ROS_INFO("YOLOv8 RKNN检测节点已启动，等待图像...");
        ROS_INFO("无人机里程计话题: %s", drone_odom_topic.c_str());
        
        // 主循环
        ros::Rate rate(100); // 100Hz
        while (ros::ok() && !shutdown_requested.load()) {
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("正在清理资源...");
        
        // 等待所有回调完成
        std::lock_guard<std::mutex> lock(yolo_mutex);
        
        // 清理
        yolo.reset();
        target_tracker.reset();
        
        ROS_INFO("YOLOv8 RKNN检测节点已安全退出，共处理 %d 帧", processed_frames.load());
        
        return 0;
    }
    catch (const std::exception& e) {
        ROS_ERROR("程序发生异常: %s", e.what());
        return -1;
    }
    catch (...) {
        ROS_ERROR("程序发生未知异常");
        return -1;
    }
}