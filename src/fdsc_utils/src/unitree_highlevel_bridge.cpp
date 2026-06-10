#include <fdsc_utils/free_dog_sdk_h.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace
{
double clampAbs(double value, double limit)
{
  const double abs_limit = std::abs(limit);
  return std::max(-abs_limit, std::min(abs_limit, value));
}

std::string modeName(FDSC::ROBOTModeHigh mode)
{
  switch (mode) {
    case FDSC::ROBOTModeHigh::IDLE: return "IDLE";
    case FDSC::ROBOTModeHigh::FORCE_STAND: return "FORCE_STAND";
    case FDSC::ROBOTModeHigh::VEL_WALK: return "VEL_WALK";
    case FDSC::ROBOTModeHigh::POS_WALK: return "POS_WALK";
    case FDSC::ROBOTModeHigh::PATH: return "PATH";
    case FDSC::ROBOTModeHigh::STAND_DOWN: return "STAND_DOWN";
    case FDSC::ROBOTModeHigh::STAND_UP: return "STAND_UP";
    case FDSC::ROBOTModeHigh::DAMPING: return "DAMPING";
    case FDSC::ROBOTModeHigh::RECOVERY: return "RECOVERY";
    case FDSC::ROBOTModeHigh::BACKFLIP: return "BACKFLIP";
    case FDSC::ROBOTModeHigh::JUMPYAW: return "JUMPYAW";
    case FDSC::ROBOTModeHigh::STRAIGHTHAND: return "STRAIGHTHAND";
    case FDSC::ROBOTModeHigh::DANCE1: return "DANCE1";
    case FDSC::ROBOTModeHigh::DANCE2: return "DANCE2";
  }
  return "UNKNOWN";
}

std::string normalizeCommand(std::string value)
{
  value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }), value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return value;
}

bool parseModeCommand(const std::string& raw, FDSC::ROBOTModeHigh& mode)
{
  const std::string value = normalizeCommand(raw);
  if (value == "IDLE") {
    mode = FDSC::ROBOTModeHigh::IDLE;
    return true;
  }
  if (value == "FORCE_STAND" || value == "FORCESTAND" || value == "STAND" || value == "STOP") {
    mode = FDSC::ROBOTModeHigh::FORCE_STAND;
    return true;
  }
  if (value == "VEL_WALK" || value == "VELWALK" || value == "WALK") {
    mode = FDSC::ROBOTModeHigh::VEL_WALK;
    return true;
  }
  if (value == "STAND_DOWN" || value == "STANDDOWN" || value == "DOWN") {
    mode = FDSC::ROBOTModeHigh::STAND_DOWN;
    return true;
  }
  if (value == "STAND_UP" || value == "STANDUP" || value == "UP") {
    mode = FDSC::ROBOTModeHigh::STAND_UP;
    return true;
  }
  if (value == "RECOVERY" || value == "RECOVER") {
    mode = FDSC::ROBOTModeHigh::RECOVERY;
    return true;
  }
  return false;
}

bool isStartupCommand(const std::string& raw)
{
  const std::string value = normalizeCommand(raw);
  return value == "AUTO" || value == "STARTUP" || value == "AUTO_STARTUP" || value == "AUTOSTARTUP";
}

FDSC::GaitType gaitFromInt(int value)
{
  switch (value) {
    case 2: return FDSC::GaitType::TROT_RUNNING;
    case 3: return FDSC::GaitType::CLIMB_STAIR;
    case 4: return FDSC::GaitType::TROT_OBSTACLE;
    case 1:
    default:
      return FDSC::GaitType::TROT;
  }
}
}  // namespace

class UnitreeHighlevelBridge
{
public:
  UnitreeHighlevelBridge()
  : private_nh_("~")
  {
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    private_nh_.param<std::string>("mode_command_topic", mode_command_topic_, "/unitree_highlevel/mode_cmd");
    private_nh_.param<std::string>("status_topic", status_topic_, "/unitree_highlevel/status");
    private_nh_.param<std::string>("connection_settings", connection_settings_, "HIGH_WIFI_DEFAULTS");
    private_nh_.param("control_rate", control_rate_, 500.0);
    private_nh_.param("command_timeout", command_timeout_sec_, 0.35);
    private_nh_.param("max_linear_x", max_linear_x_, 0.35);
    private_nh_.param("max_linear_y", max_linear_y_, 0.25);
    private_nh_.param("max_angular_z", max_angular_z_, 0.80);
    private_nh_.param("body_height", body_height_, 0.10);
    private_nh_.param("foot_raise_height", foot_raise_height_, 0.13);
    private_nh_.param("startup_sequence", startup_sequence_, true);
    private_nh_.param("stand_down_duration", stand_down_duration_, 2.0);
    private_nh_.param("recovery_duration", recovery_duration_, 2.0);
    private_nh_.param("stand_up_duration", stand_up_duration_, 1.0);
    private_nh_.param("force_stand_duration", force_stand_duration_, 1.0);
    private_nh_.param("status_rate", status_rate_, 2.0);

    int gait_type = 1;
    private_nh_.param("gait_type", gait_type, 1);
    gait_type_ = gaitFromInt(gait_type);

    conn_.reset(new FDSC::UnitreeConnection(connection_settings_));
    cmd_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &UnitreeHighlevelBridge::onCmdVel, this);
    mode_sub_ = nh_.subscribe(mode_command_topic_, 1, &UnitreeHighlevelBridge::onModeCommand, this);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);

    startup_begin_ = ros::Time::now();
    last_cmd_stamp_ = ros::Time(0);
    next_status_stamp_ = ros::Time(0);

    ROS_INFO(
      "unitree_highlevel_bridge started. cmd_vel=%s mode_cmd=%s status=%s settings=%s max=[%.3f %.3f %.3f]",
      cmd_vel_topic_.c_str(), mode_command_topic_.c_str(), status_topic_.c_str(), connection_settings_.c_str(),
      max_linear_x_, max_linear_y_, max_angular_z_);
  }

  void spin()
  {
    conn_->startRecv();
    conn_->send(command_.buildCmd(false));

    ros::Rate rate(std::max(1.0, control_rate_));
    while (ros::ok()) {
      const ros::Time now = ros::Time::now();
      consumeStatePackets();
      sendCommand(now);
      publishStatus(now);
      ros::spinOnce();
      rate.sleep();
    }

    command_.robotmode_ = FDSC::ROBOTModeHigh::FORCE_STAND;
    command_.velocity[0] = 0.0;
    command_.velocity[1] = 0.0;
    command_.yawSpeed = 0.0;
    conn_->send(command_.buildCmd(false));
  }

private:
  void onCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
  {
    latest_cmd_ = *msg;
    last_cmd_stamp_ = ros::Time::now();
  }

  void onModeCommand(const std_msgs::String::ConstPtr& msg)
  {
    if (isStartupCommand(msg->data)) {
      startup_sequence_ = true;
      manual_mode_override_ = false;
      startup_begin_ = ros::Time::now();
      latest_cmd_ = geometry_msgs::Twist();
      last_cmd_stamp_ = ros::Time(0);
      ROS_INFO("unitree_highlevel_bridge mode command: AUTO_STARTUP");
      return;
    }

    FDSC::ROBOTModeHigh requested_mode;
    if (!parseModeCommand(msg->data, requested_mode)) {
      ROS_WARN("unitree_highlevel_bridge ignored unknown mode command: %s", msg->data.c_str());
      return;
    }

    startup_sequence_ = false;
    manual_mode_override_ = true;
    requested_mode_ = requested_mode;
    if (requested_mode_ != FDSC::ROBOTModeHigh::VEL_WALK) {
      latest_cmd_ = geometry_msgs::Twist();
      last_cmd_stamp_ = ros::Time(0);
    }
    ROS_INFO("unitree_highlevel_bridge mode command: %s", modeName(requested_mode_).c_str());
  }

  void consumeStatePackets()
  {
    std::vector<std::vector<uint8_t>> packets;
    conn_->getData(packets);
    if (packets.empty()) {
      return;
    }

    const std::vector<uint8_t>& packet = packets.back();
    if (packet.size() < 1087) {
      return;
    }

    try {
      state_.parseData(packet);
      has_state_ = true;
    } catch (const std::exception& exc) {
      ROS_WARN_THROTTLE(2.0, "unitree_highlevel_bridge state parse failed: %s", exc.what());
    }
  }

  void sendCommand(const ros::Time& now)
  {
    const FDSC::ROBOTModeHigh mode = selectMode(now);
    command_.robotmode_ = mode;
    command_.bodyHeight = static_cast<float>(body_height_);
    command_.footRaiseHeight = static_cast<float>(foot_raise_height_);
    command_.velocity[0] = 0.0f;
    command_.velocity[1] = 0.0f;
    command_.yawSpeed = 0.0f;

    if (mode == FDSC::ROBOTModeHigh::VEL_WALK) {
      command_.gaitType = gait_type_;
      const bool fresh_cmd = last_cmd_stamp_.isValid() &&
        (now - last_cmd_stamp_).toSec() <= command_timeout_sec_;
      if (fresh_cmd) {
        command_.velocity[0] = static_cast<float>(clampAbs(latest_cmd_.linear.x, max_linear_x_));
        command_.velocity[1] = static_cast<float>(clampAbs(latest_cmd_.linear.y, max_linear_y_));
        command_.yawSpeed = static_cast<float>(clampAbs(latest_cmd_.angular.z, max_angular_z_));
      }
    } else {
      command_.gaitType = FDSC::GaitType::IDLE;
    }

    conn_->send(command_.buildCmd(false));
    current_mode_ = mode;
  }

  FDSC::ROBOTModeHigh selectMode(const ros::Time& now) const
  {
    if (manual_mode_override_) {
      return requested_mode_;
    }

    if (!startup_sequence_) {
      return FDSC::ROBOTModeHigh::VEL_WALK;
    }

    const double t = (now - startup_begin_).toSec();
    double cursor = 0.0;
    cursor += std::max(0.0, stand_down_duration_);
    if (t < cursor) {
      return FDSC::ROBOTModeHigh::STAND_DOWN;
    }
    cursor += std::max(0.0, recovery_duration_);
    if (t < cursor) {
      return FDSC::ROBOTModeHigh::RECOVERY;
    }
    cursor += std::max(0.0, stand_up_duration_);
    if (t < cursor) {
      return FDSC::ROBOTModeHigh::STAND_UP;
    }
    cursor += std::max(0.0, force_stand_duration_);
    if (t < cursor) {
      return FDSC::ROBOTModeHigh::FORCE_STAND;
    }
    return FDSC::ROBOTModeHigh::VEL_WALK;
  }

  void publishStatus(const ros::Time& now)
  {
    const double period = 1.0 / std::max(0.1, status_rate_);
    if (next_status_stamp_.isValid() && now < next_status_stamp_) {
      return;
    }
    next_status_stamp_ = now + ros::Duration(period);

    std::ostringstream ss;
    ss << "mode=" << modeName(current_mode_)
       << " override=" << (manual_mode_override_ ? "manual" : "startup")
       << " cmd_age=" << commandAge(now)
       << " vx=" << command_.velocity[0]
       << " vy=" << command_.velocity[1]
       << " wz=" << command_.yawSpeed;
    if (has_state_) {
      ss << " robot_mode=" << static_cast<int>(state_.Robotmode)
         << " gait=" << static_cast<int>(state_.gaitType)
         << " soc=" << static_cast<int>(state_.SOC);
    } else {
      ss << " state=waiting";
    }

    std_msgs::String msg;
    msg.data = ss.str();
    status_pub_.publish(msg);
  }

  double commandAge(const ros::Time& now) const
  {
    if (!last_cmd_stamp_.isValid()) {
      return -1.0;
    }
    return (now - last_cmd_stamp_).toSec();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber mode_sub_;
  ros::Publisher status_pub_;
  std::unique_ptr<FDSC::UnitreeConnection> conn_;
  FDSC::highCmd command_;
  FDSC::highState state_;
  geometry_msgs::Twist latest_cmd_;

  std::string cmd_vel_topic_;
  std::string mode_command_topic_;
  std::string status_topic_;
  std::string connection_settings_;
  double control_rate_ = 500.0;
  double command_timeout_sec_ = 0.35;
  double max_linear_x_ = 0.35;
  double max_linear_y_ = 0.25;
  double max_angular_z_ = 0.8;
  double body_height_ = 0.1;
  double foot_raise_height_ = 0.13;
  double stand_down_duration_ = 2.0;
  double recovery_duration_ = 2.0;
  double stand_up_duration_ = 1.0;
  double force_stand_duration_ = 1.0;
  double status_rate_ = 2.0;
  bool startup_sequence_ = true;
  bool manual_mode_override_ = false;
  bool has_state_ = false;
  FDSC::GaitType gait_type_ = FDSC::GaitType::TROT;
  FDSC::ROBOTModeHigh current_mode_ = FDSC::ROBOTModeHigh::IDLE;
  FDSC::ROBOTModeHigh requested_mode_ = FDSC::ROBOTModeHigh::VEL_WALK;
  ros::Time startup_begin_;
  ros::Time last_cmd_stamp_;
  ros::Time next_status_stamp_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unitree_highlevel_bridge");
  UnitreeHighlevelBridge bridge;
  bridge.spin();
  return 0;
}