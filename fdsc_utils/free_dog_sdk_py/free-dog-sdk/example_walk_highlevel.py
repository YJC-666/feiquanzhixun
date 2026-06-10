#include <fdsc_utils/free_dog_sdk_h.hpp>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <simple_follower/position.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 全局变量
geometry_msgs::Twist current_twist;
ros::Publisher nav_goal_pub;

// 回调函数：处理 /cmd_vel 主题的消息
void twist_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    current_twist = *msg;
}

// 发布导航目标的函数
void publishNavGoal(float nav_x, float nav_y)
{
    simple_follower::position nav_goal;
    nav_goal.nav_x = nav_x;
    nav_goal.nav_y = nav_y;
    nav_goal_pub.publish(nav_goal);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "unitree_control");
    ros::NodeHandle nh;

    // 订阅 /cmd_vel 主题
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, twist_cb);

    // 广播导航目标主题
    nav_goal_pub = nh.advertise<simple_follower::position>("nav_goal", 10);

    // 设置为高WiFi默认配置
    std::string settings = "HIGH_WIFI_DEFAULTS";
    FDSC::UnitreeConnection conn(settings); // 创建连接对象
    conn.startRecv(); // 开始接收数据

    // 创建高层命令和状态对象
    FDSC::highCmd hcmd;
    FDSC::highState hstate;

    // 发送一个空命令以初始化连接
    std::vector<uint8_t> cmd_bytes = hcmd.buildCmd(false);
    conn.send(cmd_bytes);

    // 定义机器人模式
    enum DogMode
    {
        STAND_DOWN,
        RECOVERY,
        STAND_UP,
        FORCE_STAND,
        VEL_WALK
    };

    DogMode dog_mode = STAND_DOWN; // 初始模式为站低
    ros::Time state_start_time = ros::Time::now(); // 记录模式开始时间

    // 500Hz 的循环频率
    ros::Rate loop_rate(500);

    while (ros::ok())
    {
        std::vector<std::vector<uint8_t>> dataall = conn.getData(); // 获取接收的数据

        if (dataall.size() != 0)
        {
            std::vector<uint8_t> data = dataall.at(dataall.size() - 1); // 获取最后一个数据包
            hstate.parseData(data); // 解析数据

            // 控制逻辑
            switch (dog_mode)
            {
                case STAND_DOWN:
                    hcmd.robotmode_ = FDSC::ROBOTModeHigh::STAND_DOWN; // 站低模式
                    if (ros::Time::now() - state_start_time > ros::Duration(8.0))
                    {
                        dog_mode = RECOVERY;
                        state_start_time = ros::Time::now();
                    }
                    break;

                case RECOVERY:
                    hcmd.robotmode_ = FDSC::ROBOTModeHigh::RECOVERY; // 恢复模式
                    if (ros::Time::now() - state_start_time > ros::Duration(8.0))
                    {
                        dog_mode = STAND_UP;
                        state_start_time = ros::Time::now();
                    }
                    break;

                case STAND_UP:
                    hcmd.robotmode_ = FDSC::ROBOTModeHigh::STAND_UP; // 站起模式
                    if (ros::Time::now() - state_start_time > ros::Duration(1.0))
                    {
                        dog_mode = FORCE_STAND;
                        state_start_time = ros::Time::now();
                    }
                    break;

                case FORCE_STAND:
                    hcmd.robotmode_ = FDSC::ROBOTModeHigh::FORCE_STAND; // 强制站立模式
                    if (ros::Time::now() - state_start_time > ros::Duration(1.0))
                    {
                        dog_mode = VEL_WALK;
                        state_start_time = ros::Time::now();
                    }
                    break;

                case VEL_WALK:
                    hcmd.robotmode_ = FDSC::ROBOTModeHigh::VEL_WALK; // 行走模式
                    hcmd.gaitType = FDSC::GaitType::TROT; // 小跑步态
                    hcmd.velocity.at(0) = current_twist.linear.x; // 从 /cmd_vel 主题获取线速度
                    hcmd.velocity.at(1) = current_twist.linear.y; // 从 /cmd_vel 主题获取侧向速度
                    hcmd.yawSpeed = current_twist.angular.z; // 从 /cmd_vel 主题获取角速度
                    hcmd.bodyHeight = 0.1;
                    hcmd.footRaiseHeight = 0.13;
                    break;
            }

            // 发送当前模式的命令
            cmd_bytes = hcmd.buildCmd(false); // 构建命令
            conn.send(cmd_bytes); // 发送命令
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0; // 程序结束
}

