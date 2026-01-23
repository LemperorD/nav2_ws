#include "decision_simple/decision_simple.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"                           
#include "tf2/exceptions.h"                     

namespace decision_simple
{

DecisionSimple::DecisionSimple(const rclcpp::NodeOptions & options)
: Node("decision_simple", options)
{
  // ===== params =====
  frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");      

  robot_status_topic_ = this->declare_parameter<std::string>("referee_robot_status_topic", "referee/robot_status");
  goal_pose_topic_    = this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");

  chassis_mode_topic_ = this->declare_parameter<std::string>("chassis_mode_topic", "chassis_mode");
  debug_attack_pose_topic_ = this->declare_parameter<std::string>("debug_attack_pose_topic", "debug_attack_pose");

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  detector_armors_topic_ = this->declare_parameter<std::string>("detector_armors_topic", "detector/armors");
  tracker_target_topic_  = this->declare_parameter<std::string>("tracker_target_topic",  "tracker/target");
#endif

  supply_x_   = this->declare_parameter<double>("supply_x", 0.0);
  supply_y_   = this->declare_parameter<double>("supply_y", 0.0);
  supply_yaw_ = this->declare_parameter<double>("supply_yaw", 0.0);

  default_x_   = this->declare_parameter<double>("default_x", 4.65);
  default_y_   = this->declare_parameter<double>("default_y", -3.5);
  default_yaw_ = this->declare_parameter<double>("default_yaw", 0.0);

  default_arrive_xy_tol_ = this->declare_parameter<double>("default_arrive_xy_tol", 0.30);   
  supply_arrive_xy_tol_  = this->declare_parameter<double>("supply_arrive_xy_tol", 0.30);   

  hp_enter_supply_ = this->declare_parameter<int>("hp_enter_supply", 120);
  hp_exit_supply_  = this->declare_parameter<int>("hp_exit_supply", 300);
  ammo_min_        = this->declare_parameter<int>("ammo_min", 0);

  combat_max_distance_ = this->declare_parameter<double>("combat_max_distance", 8.0);
  attack_hold_sec_     = this->declare_parameter<double>("attack_hold_sec", 1.0);

  attacked_hold_sec_   = this->declare_parameter<double>("attacked_hold_sec", 1.5);          

  tick_hz_         = this->declare_parameter<double>("tick_hz", 20.0);
  default_goal_hz_ = this->declare_parameter<double>("default_goal_hz", 2.0);
  supply_goal_hz_  = this->declare_parameter<double>("supply_goal_hz", 2.0);
  attack_goal_hz_  = this->declare_parameter<double>("attack_goal_hz", 10.0);

  // ===== TF init =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());                           
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);                   

  // ===== pubs =====
  goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, rclcpp::SensorDataQoS());
  chassis_mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>(chassis_mode_topic_, rclcpp::QoS(10));
  debug_attack_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(debug_attack_pose_topic_, rclcpp::QoS(10));

  // ===== subs =====
  robot_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
    robot_status_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onRobotStatus, this, std::placeholders::_1));

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
    detector_armors_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onArmors, this, std::placeholders::_1));

  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    tracker_target_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onTarget, this, std::placeholders::_1));
#endif

  // init
  setState(State::DEFAULT);

  
  setChassisMode(chassisFollowed);

  // timer
  const double hz = std::max(1e-6, tick_hz_);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
  timer_ = this->create_wall_timer(period, std::bind(&DecisionSimple::tick, this));

  RCLCPP_INFO(this->get_logger(),
    "decision_simple(min+mode) started. ns=%s goal_pose=%s robot_status=%s chassis_mode=%s",
    this->get_namespace(), goal_pose_topic_.c_str(), robot_status_topic_.c_str(), chassis_mode_topic_.c_str());
}

// ================= callbacks =================
void DecisionSimple::onRobotStatus(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_robot_status_ = *msg;
  has_robot_status_ = true;
}

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
void DecisionSimple::onArmors(const auto_aim_interfaces::msg::Armors::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_armors_ = *msg;
  has_armors_ = true;
}

void DecisionSimple::onTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_target_opt_ = *msg;
}
#endif
// ================= tick：决策 + 底盘模式切换 =================
void DecisionSimple::tick()
{
  // snapshot
  pb_rm_interfaces::msg::RobotStatus rs;
  bool has_rs = false;

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  auto_aim_interfaces::msg::Armors armors;
  bool has_armors = false;
  std::optional<auto_aim_interfaces::msg::Target> target_opt;
#endif

  {
    std::lock_guard<std::mutex> lk(mtx_);
    has_rs = has_robot_status_;
    if (has_rs) rs = last_robot_status_;

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
    has_armors = has_armors_;
    if (has_armors) armors = last_armors_;
    target_opt = last_target_opt_;
#endif
  }

  if (!has_rs) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for robot_status...");
    return;
  }

  const auto now = this->now();

  // 受到攻击检测
  if (rs.is_hp_deduced) {
    last_attacked_ = now;
  }
  const bool attacked_recent =
    (last_attacked_.nanoseconds() != 0) &&
    ((now - last_attacked_).seconds() <= attacked_hold_sec_);

  // 到达判断（中心点/补给点）
  const bool at_center = isNear(default_x_, default_y_, default_arrive_xy_tol_);
  const bool at_supply = isNear(supply_x_, supply_y_, supply_arrive_xy_tol_);

  // ================= 1) 补给保持 =================
  if (state_ == State::SUPPLY) {
    if (!isStatusRecovered(rs)) {
     
      if (attacked_recent) setChassisMode(littleTES);             
      else setChassisMode(at_supply ? littleTES : goHome);        

      const auto goal = makePoseXYZYaw(frame_id_, supply_x_, supply_y_, supply_yaw_);
      publishGoalThrottled(goal, last_supply_pub_, supply_goal_hz_);
      return;
    }
    
  }

  // ================= 2) 状态差 -> 进补给 =================
  if (isStatusBad(rs)) {
    setState(State::SUPPLY);

    
    if (attacked_recent) setChassisMode(littleTES);               
    else setChassisMode(at_supply ? littleTES : goHome);          

    const auto goal = makePoseXYZYaw(frame_id_, supply_x_, supply_y_, supply_yaw_);
    publishGoalThrottled(goal, last_supply_pub_, supply_goal_hz_);
    return;
  }

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  // ================= 3) 状态好 -> 有敌攻击 =================
  bool enemy = false;
  if (has_armors || target_opt.has_value()) {
    enemy = detectEnemy(armors, target_opt);
  }
  if (enemy) last_enemy_seen_ = now;

  const bool enemy_recent =
    (last_enemy_seen_.nanoseconds() != 0) &&
    ((now - last_enemy_seen_).seconds() <= attack_hold_sec_);

  if (enemy_recent) {
    setState(State::ATTACK);

   
    if (attacked_recent) setChassisMode(littleTES);               
    else setChassisMode(chassisFollowed);                         

    geometry_msgs::msg::PoseStamped attack_goal;
    if (buildAttackGoal(attack_goal, armors, target_opt)) {
      last_attack_goal_ = attack_goal;
      has_last_attack_goal_ = true;
      debug_attack_pose_pub_->publish(attack_goal);
      publishGoalThrottled(attack_goal, last_attack_pub_, attack_goal_hz_);
      return;
    }

    if (has_last_attack_goal_) {
      debug_attack_pose_pub_->publish(last_attack_goal_);
      publishGoalThrottled(last_attack_goal_, last_attack_pub_, attack_goal_hz_);
    }
    return;
  }
#endif

  // ================= 4) 默认：去中心点 =================
  setState(State::DEFAULT);

 
  if (attacked_recent) setChassisMode(littleTES);                
  else setChassisMode(at_center ? littleTES : chassisFollowed);   

  const auto goal = makePoseXYZYaw(frame_id_, default_x_, default_y_, default_yaw_);
  publishGoalThrottled(goal, last_default_pub_, default_goal_hz_);
}

  

// ================= helpers =================
geometry_msgs::msg::PoseStamped DecisionSimple::makePoseXYZYaw(const std::string & frame, double x, double y, double yaw) const
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.header.stamp = this->now();
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

bool DecisionSimple::isStatusBad(const pb_rm_interfaces::msg::RobotStatus & rs) const
{
  const int hp = static_cast<int>(rs.current_hp);
  const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
  return (hp < hp_enter_supply_) || (ammo < ammo_min_);
}

bool DecisionSimple::isStatusRecovered(const pb_rm_interfaces::msg::RobotStatus & rs) const
{
  const int hp = static_cast<int>(rs.current_hp);
  const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
  return (hp >= hp_exit_supply_) && (ammo >= ammo_min_);
}

// 获取机器人在 map 下的位置
bool DecisionSimple::getRobotPoseMap(double & x, double & y, double & yaw) 
{
  try {
    const auto tf = tf_buffer_->lookupTransform(frame_id_, base_frame_id_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    yaw = tf2::getYaw(tf.transform.rotation);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TF lookup failed (%s -> %s): %s", frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
    return false;
  }
}

// 是否到达某点（只看平面距离）
bool DecisionSimple::isNear(double gx, double gy, double tol_xy) 
{
  double x, y, yaw;
  (void)yaw;
  if (!getRobotPoseMap(x, y, yaw)) return false;
  const double dx = x - gx;
  const double dy = y - gy;
  return std::hypot(dx, dy) <= tol_xy;
}

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
bool DecisionSimple::detectEnemy(const auto_aim_interfaces::msg::Armors & armors,
                                const std::optional<auto_aim_interfaces::msg::Target> & target_opt) const
{
  if (target_opt.has_value() && target_opt->tracking) {
    const double x = target_opt->position.x;
    const double y = target_opt->position.y;
    const double z = target_opt->position.z;
    const double dist = std::sqrt(x*x + y*y + z*z);
    return dist <= combat_max_distance_;
  }

  for (const auto & a : armors.armors) {
    const double x = a.pose.position.x;
    const double y = a.pose.position.y;
    const double z = a.pose.position.z;
    const double dist = std::sqrt(x*x + y*y + z*z);
    if (dist <= combat_max_distance_) return true;
  }
  return false;
}

bool DecisionSimple::buildAttackGoal(geometry_msgs::msg::PoseStamped & out,
                                    const auto_aim_interfaces::msg::Armors & armors,
                                    const std::optional<auto_aim_interfaces::msg::Target> & target_opt) const
{
  if (target_opt.has_value() && target_opt->tracking) {
    const std::string frame = (!target_opt->header.frame_id.empty()) ? target_opt->header.frame_id : frame_id_;
    out = makePoseXYZYaw(frame, target_opt->position.x, target_opt->position.y, target_opt->yaw);
    return true;
  }

  if (armors.armors.empty()) return false;

  const auto * best = &armors.armors.front();
  double best_dist = 1e18;
  for (const auto & a : armors.armors) {
    const double x = a.pose.position.x;
    const double y = a.pose.position.y;
    const double z = a.pose.position.z;
    const double dist = std::sqrt(x*x + y*y + z*z);
    if (dist < best_dist) {
      best_dist = dist;
      best = &a;
    }
  }

  const std::string frame = (!armors.header.frame_id.empty()) ? armors.header.frame_id : frame_id_;
  out = makePoseXYZYaw(frame, best->pose.position.x, best->pose.position.y, 0.0);
  return true;
}
#endif

void DecisionSimple::setState(State s)
{
  if (state_ == s) return;
  state_ = s;

  //不再用 state_ 去发布 chassis_mode
  RCLCPP_INFO(this->get_logger(), "State -> %u", static_cast<unsigned>(state_));
}

// 
void DecisionSimple::publishChassisMode(chassisMode mode)
{
  std_msgs::msg::UInt8 m;
  m.data = static_cast<uint8_t>(mode);
  chassis_mode_pub_->publish(m);
}

// 只在发生变化时发一次，避免刷屏
void DecisionSimple::setChassisMode(chassisMode mode)
{
  if (current_chassis_mode_ == mode) return;
  current_chassis_mode_ = mode;
  publishChassisMode(mode);
  RCLCPP_INFO(this->get_logger(), "ChassisMode -> %u", static_cast<unsigned>(mode));
}

void DecisionSimple::publishGoalThrottled(const geometry_msgs::msg::PoseStamped & goal, rclcpp::Time & last_pub, double hz)
{
  const double period = (hz <= 1e-6) ? 1e9 : (1.0 / hz);
  const auto now = this->now();

  if (last_pub.nanoseconds() == 0 || (now - last_pub).seconds() >= period) {
    last_pub = now;
    goal_pose_pub_->publish(goal);
  }
}
}

// namespace decision_simple

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(decision_simple::DecisionSimple)
