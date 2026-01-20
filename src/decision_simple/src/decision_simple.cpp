#include "decision_simple/decision_simple.hpp"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace decision_simple
{

static double steadySeconds()
{
  using clock = std::chrono::steady_clock;
  const auto now = clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

DecisionSimple::DecisionSimple(const rclcpp::NodeOptions & options)
: Node("decision_simple", options)
{
  // ===== parameters (尽量沿用 XML 名称/默认值) =====
  scenario_ = this->declare_parameter<std::string>("scenario", "main");

  referee_robot_status_topic_ = this->declare_parameter<std::string>("referee_robot_status_topic", "referee/robot_status");
  referee_game_status_topic_  = this->declare_parameter<std::string>("referee_game_status_topic",  "referee/game_status");
  referee_rfid_status_topic_  = this->declare_parameter<std::string>("referee_rfid_status_topic",  "referee/rfid_status");

  goal_pose_topic_         = this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
  debug_attack_pose_topic_ = this->declare_parameter<std::string>("debug_attack_pose_topic", "debug_attack_pose");

  chassis_mode_topic_ = this->declare_parameter<std::string>("chassis_mode_topic", "chassis_mode");
  cmd_spin_topic_     = this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  cmd_vel_topic_      = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  cmd_gimbal_topic_   = this->declare_parameter<std::string>("cmd_gimbal_topic", "cmd_gimbal");

  sub_mode_topic_ = this->declare_parameter<std::string>("sub_mode_topic", "sub_mode");
  nav_globalCostmap_topic_ = this->declare_parameter<std::string>("nav_globalCostmap_topic", "nav_globalCostmap");

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  detector_armors_topic_ = this->declare_parameter<std::string>("detector_armors_topic", "detector/armors");
  tracker_target_topic_  = this->declare_parameter<std::string>("tracker_target_topic",  "tracker/target");
#endif

  frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

  supply_x_   = this->declare_parameter<double>("supply_x", 0.0);
  supply_y_   = this->declare_parameter<double>("supply_y", 0.0);
  supply_yaw_ = this->declare_parameter<double>("supply_yaw", 0.0);

  default_x_   = this->declare_parameter<double>("default_x", 4.65);
  default_y_   = this->declare_parameter<double>("default_y", -3.5);
  default_yaw_ = this->declare_parameter<double>("default_yaw", 0.0);

  hp_survival_enter_ = this->declare_parameter<int>("hp_survival_enter", 120);
  hp_survival_exit_  = this->declare_parameter<int>("hp_survival_exit",  300);
  hp_supply_done_    = this->declare_parameter<int>("hp_supply_done",    399);

  heat_ok_default_   = this->declare_parameter<int>("heat_ok_default",   350);
  heat_supply_done_  = this->declare_parameter<int>("heat_supply_done",  100);
  ammo_min_          = this->declare_parameter<int>("ammo_min", 0);

  combat_max_distance_ = this->declare_parameter<double>("combat_max_distance", 8.0);
  combat_cooldown_sec_ = this->declare_parameter<double>("combat_cooldown_sec", 3.0);

  default_goal_hz_ = this->declare_parameter<double>("default_goal_hz", 5.0);
  tick_hz_         = this->declare_parameter<double>("tick_hz", 20.0);

  spin_speed_ = this->declare_parameter<double>("spin_speed", 7.0);

  require_game_running_ = this->declare_parameter<bool>("require_game_running", false);
  enable_attacked_feedback_ = this->declare_parameter<bool>("enable_attacked_feedback", false);

  // ===== pubs (QoS：goal_pose 用 best_effort，匹配 bt_navigator) =====
  chassis_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>(chassis_mode_topic_, rclcpp::QoS(10));
  cmd_spin_pub_     = this->create_publisher<std_msgs::msg::Float64>(cmd_spin_topic_, rclcpp::QoS(10));
  cmd_vel_pub_      = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
  cmd_gimbal_pub_   = this->create_publisher<geometry_msgs::msg::Vector3>(cmd_gimbal_topic_, rclcpp::QoS(10));

  goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, rclcpp::SensorDataQoS());
  debug_attack_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(debug_attack_pose_topic_, rclcpp::QoS(10));

  // ===== subs =====
  robot_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
    referee_robot_status_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onRobotStatus, this, std::placeholders::_1));

  game_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::GameStatus>(
    referee_game_status_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onGameStatus, this, std::placeholders::_1));

  rfid_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::RfidStatus>(
    referee_rfid_status_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onRfidStatus, this, std::placeholders::_1));

  sub_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    sub_mode_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onSubMode, this, std::placeholders::_1));

  costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
    nav_globalCostmap_topic_, rclcpp::QoS(1),
    std::bind(&DecisionSimple::onCostmap, this, std::placeholders::_1));



#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
    detector_armors_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onArmors, this, std::placeholders::_1));

  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    tracker_target_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimple::onTarget, this, std::placeholders::_1));
#endif

  // ===== init (对应 Initialize) =====
  current_mode_ = -1;
  SetChassisMode(1);

  // ===== timer =====
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / tick_hz_));
  timer_ = this->create_wall_timer(period, [this]() {
    if (scenario_ == "main") tickMain();
    else tickTest();
  });

  RCLCPP_INFO(this->get_logger(),
    "decision_simple started. scenario=%s, ns=%s, goal_pose_topic=%s",
    scenario_.c_str(), this->get_namespace(), goal_pose_topic_.c_str());
}

// ================= callbacks =================
void DecisionSimple::onRobotStatus(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg)
{
  last_robot_status_ = *msg;
  has_robot_status_ = true;
}

void DecisionSimple::onGameStatus(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg)
{
  last_game_status_ = *msg;
  has_game_status_ = true;
}

void DecisionSimple::onRfidStatus(const pb_rm_interfaces::msg::RfidStatus::SharedPtr msg)
{
  last_rfid_status_ = *msg;
  has_rfid_status_ = true;
}

void DecisionSimple::onSubMode(const std_msgs::msg::Bool::SharedPtr msg)
{
  sub_mode_ = msg->data;
}

void DecisionSimple::onCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  last_costmap_ = *msg;
  has_costmap_ = true;
}


#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
void DecisionSimple::onArmors(const auto_aim_interfaces::msg::Armors::SharedPtr msg)
{
  last_armors_ = *msg;
  has_armors_ = true;
}

void DecisionSimple::onTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  last_target_ = *msg;
  has_target_ = true;
}
#endif

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

// ================= “BT条件节点”实现 =================
bool DecisionSimple::IsStatusOK(int ammo_min, int heat_max, int hp_min) const
{
  if (!has_robot_status_) return false;
  const int ammo = static_cast<int>(last_robot_status_.projectile_allowance_17mm);
  const int hp   = static_cast<int>(last_robot_status_.current_hp);
  // 注意：RobotStatus 里没有 heat 字段时你需要改这里；fake_referee 当前没发布 heat
  // 但 XML 用 heat 门控，所以这里先用 shooter_barrel_heat_limit 或者直接当 0（最保守做法：缺字段就认为 ok）
  int heat = 0;
  // 如果你们 RobotStatus.msg 里有 shooter_17mm_1_barrel_heat，就改成：
  // heat = static_cast<int>(last_robot_status_.shooter_17mm_1_barrel_heat);

  return (ammo >= ammo_min) && (heat <= heat_max) && (hp >= hp_min);
}

bool DecisionSimple::IsGameStatus(int expected_progress, int min_remain, int max_remain) const
{
  if (!has_game_status_) return false;
  const int prog = static_cast<int>(last_game_status_.game_progress);
  const int t    = static_cast<int>(last_game_status_.stage_remain_time);
  return (prog == expected_progress) && (t >= min_remain) && (t <= max_remain);
}

bool DecisionSimple::IsRfidDetected_FriendlySupplyNonExchange() const
{
  if (!has_rfid_status_) return false;
  // XML 的 IsRfidDetected 里只开启了 friendly_supply_zone_non_exchange
  return static_cast<bool>(last_rfid_status_.friendly_supply_zone_non_exchange);
}

bool DecisionSimple::IsCombatCooldownReady() const
{
  const double now = steadySeconds();
  return now >= combat_cooldown_ready_time_;
}

bool DecisionSimple::IsAttacked(uint8_t & armor_id_out) const
{
  if (!has_robot_status_) return false;
  const bool attacked = static_cast<bool>(last_robot_status_.is_hp_deduced);
  armor_id_out = static_cast<uint8_t>(last_robot_status_.armor_id);
  return attacked;
}

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
bool DecisionSimple::IsDetectEnemy() const
{
  if (!has_armors_) return false;
  // 简化版：只要存在 armor 且距离 < combat_max_distance_ 且 id 在 {1,2,3,4,5,7} 就算检测到
  const std::vector<std::string> allowed{"1","2","3","4","5","7"};
  for (const auto & a : last_armors_.armors) {
    if (std::find(allowed.begin(), allowed.end(), a.number) == allowed.end()) {
      continue;
    }
    const double x = a.pose.position.x;
    const double y = a.pose.position.y;
    const double z = a.pose.position.z;
    const double dist = std::sqrt(x*x + y*y + z*z);
    if (dist <= combat_max_distance_) {
      return true;
    }
  }
  return false;
}
#endif

// ================= “BT动作节点”实现 =================
void DecisionSimple::SetChassisMode(int mode)
{
  if (current_mode_ == mode) return;

  std_msgs::msg::Int32 m;
  m.data = mode;
  chassis_mode_pub_->publish(m);

  current_mode_ = mode;

  const double now = steadySeconds();
  if (mode == 2) combat_cooldown_ready_time_ = now + combat_cooldown_sec_;
  else combat_cooldown_ready_time_ = now;

  RCLCPP_INFO(this->get_logger(), "SetChassisMode -> %d, combat_cooldown_ready_time=%.3f",
              current_mode_, combat_cooldown_ready_time_);
}

void DecisionSimple::PublishSpinSpeed(double spin_speed)
{
  std_msgs::msg::Float64 m;
  m.data = spin_speed;
  cmd_spin_pub_->publish(m);
}

void DecisionSimple::PubNav2Goal(const geometry_msgs::msg::PoseStamped & goal)
{
  goal_pose_pub_->publish(goal);
}

void DecisionSimple::PubNav2GoalXYZYaw(double x, double y, double yaw)
{
  PubNav2Goal(makePoseXYZYaw(frame_id_, x, y, yaw));
}

void DecisionSimple::PublishTwistFor(const geometry_msgs::msg::Twist & tw, std::chrono::milliseconds dur)
{
  const auto end = this->now() + rclcpp::Duration(dur);
  while (rclcpp::ok() && this->now() < end) {
    cmd_vel_pub_->publish(tw);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);
}

void DecisionSimple::PublishGimbalAbsoluteFor(double yaw, double pitch, std::chrono::milliseconds dur)
{
  const auto end = this->now() + rclcpp::Duration(dur);
  while (rclcpp::ok() && this->now() < end) {
    geometry_msgs::msg::Vector3 v;
    v.x = yaw;
    v.y = pitch;
    v.z = 0.0;
    cmd_gimbal_pub_->publish(v);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
}

// ================= CalculateAttackPose（代码版）=================
bool DecisionSimple::CalculateAttackPose(geometry_msgs::msg::PoseStamped & attack_pose_out)
{
#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  if (!has_target_) return false;
  if (!last_target_.tracking) return false;

  // frame 优先用 target 的 frame_id（很多工程里就是 map/odom）
  const std::string frame = (!last_target_.header.frame_id.empty())
                              ? last_target_.header.frame_id
                              : frame_id_;

  const double x = last_target_.position.x;
  const double y = last_target_.position.y;
  const double yaw = last_target_.yaw;

  attack_pose_out = makePoseXYZYaw(frame, x, y, yaw);
  return true;
#else
  (void)attack_pose_out;
  return false;
#endif
}


// ================= 子树复刻：Main / SubmodeCheck / ModeSelect / Execute / rmul_supply =================
void DecisionSimple::SubmodeCheck()
{
  // XML：只有 survival 时才做；并且强制 success
  if (current_mode_ != 3) return;

  // XML：先 PubNav2Goal(0,0,0) 再读 sub_mode
  PubNav2GoalXYZYaw(supply_x_, supply_y_, supply_yaw_);
}

void DecisionSimple::ModeSelect()
{
  // XML 优先级：survival > combat > default

  // survival 条件（进入/保持）
  const bool enter_survival = !IsStatusOK(ammo_min_, heat_ok_default_, hp_survival_enter_);
  const bool maintain_survival = (current_mode_ == 3) && !IsStatusOK(ammo_min_, heat_ok_default_, hp_survival_exit_);
  if (enter_survival || maintain_survival) {
    SetChassisMode(3);
    return;
  }

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  // combat 条件
  if (IsDetectEnemy()) {
    SetChassisMode(2);
    return;
  }
#endif

  // default 条件（带战斗冷却门）
  if (current_mode_ == 1) return;

  if (current_mode_ != 2) {
    SetChassisMode(1);
    return;
  }

  if (IsCombatCooldownReady()) {
    SetChassisMode(1);
  }
}

void DecisionSimple::rmul_supply()
{
  // XML: WhileDoElse( IsRfidDetected(non_exchange), RetryUntilSuccessful(IsStatusOK heat<=100 hp>=399), else PubGoal supply )

  const bool in_supply_zone = IsRfidDetected_FriendlySupplyNonExchange();

  if (in_supply_zone) {
    // 在补给区：一直等到 “补给完成条件” 满足
    const bool done = IsStatusOK(ammo_min_, heat_supply_done_, hp_supply_done_);
    if (!done) {
      // 等待，不发导航（保持在补给区）
      return;
    }
    // done：什么都不做，等 ModeSelect 把你切回默认/战斗
    return;
  }

  // 不在补给区：去补给点
  PubNav2GoalXYZYaw(supply_x_, supply_y_, supply_yaw_);
}

void DecisionSimple::Execute()
{
  // 对应 XML Execute

  if (current_mode_ == 3) {
    // Survival：sub_mode 决定 supply vs attack
    if (sub_mode_ == false) {
      rmul_supply();
      return;
    }
    geometry_msgs::msg::PoseStamped attack_pose;
    if (CalculateAttackPose(attack_pose)) {
      debug_attack_pose_pub_->publish(attack_pose);
      PubNav2Goal(attack_pose);
    }
    return;
  }

  if (current_mode_ == 2) {
    geometry_msgs::msg::PoseStamped attack_pose;
    if (CalculateAttackPose(attack_pose)) {
      debug_attack_pose_pub_->publish(attack_pose);
      PubNav2Goal(attack_pose);
    }
    return;
  }

  if (current_mode_ == 1) {
    // Default：5Hz 发布固定目标点
    static rclcpp::Time last_pub = this->now();
    const double dt = (this->now() - last_pub).seconds();
    if (dt >= (1.0 / default_goal_hz_)) {
      last_pub = this->now();
      PubNav2GoalXYZYaw(default_x_, default_y_, default_yaw_);
    }
    return;
  }
}

void DecisionSimple::tickMain()
{
  // 对应 XML Main 的“一轮”

  if (require_game_running_) {
    if (!IsGameStatus(4, 0, 420)) return;
  }

  // XML check_game_start：不阻塞，这里只做日志（可删）
  if (has_game_status_ && !IsGameStatus(4, 0, 420)) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Game not in RUNNING[4] or remain_time out of range, but continue (XML AlwaysSuccess).");
  }

  // XML PublishSpinSpeed(7.0, 200)
  PublishSpinSpeed(spin_speed_);

  // XML SubmodeCheck
  SubmodeCheck();

  // XML ModeSelect
  ModeSelect();

  // XML Execute
  Execute();

  // 可选：复刻 test_attacked_feedback 的“被打反应”
  if (enable_attacked_feedback_) {
    test_attacked_feedback();
  }
}

// ================= test_*（可选场景） =================
void DecisionSimple::tickTest()
{
  if (scenario_ == "test_attack_pose") test_attack_pose();
  else if (scenario_ == "test_attacked_feedback") test_attacked_feedback();
#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
  else if (scenario_ == "test_is_detect_enemy") test_is_detect_enemy();
#endif
  else {
    // 未知就退回 main
    tickMain();
  }
}

void DecisionSimple::test_attack_pose()
{
  geometry_msgs::msg::PoseStamped attack_pose;
  if (CalculateAttackPose(attack_pose)) {
    debug_attack_pose_pub_->publish(attack_pose);
    PubNav2Goal(attack_pose);
  }
}

void DecisionSimple::test_attacked_feedback()
{
  uint8_t armor_id = 0;
  if (attack_seq_state_ == AttackSeqState::IDLE) {
    if (!IsAttacked(armor_id)) return;

    // 用 armor_id 做一个简单映射（你们可以按车体定义重映射）
    double yaw = 0.0;
    if (armor_id == 0) yaw = 0.0;          // front
    else if (armor_id == 1) yaw = 1.57;    // left
    else if (armor_id == 2) yaw = -1.57;   // right
    else if (armor_id == 3) yaw = 3.14;    // back
    double pitch = 0.0;

    // 先发云台绝对角 500ms
    attack_seq_state_ = AttackSeqState::GIMBAL;
    attack_seq_deadline_ = this->now() + rclcpp::Duration(std::chrono::milliseconds(500));

    PublishGimbalAbsoluteFor(yaw, pitch, std::chrono::milliseconds(500));

    // 进入下一段 twist
    attack_seq_state_ = AttackSeqState::TW1;
    attack_seq_deadline_ = this->now() + rclcpp::Duration(std::chrono::milliseconds(2000));
  }

  if (attack_seq_state_ == AttackSeqState::TW1) {
    geometry_msgs::msg::Twist tw; tw.angular.z = 3.14;
    PublishTwistFor(tw, std::chrono::milliseconds(2000));
    attack_seq_state_ = AttackSeqState::TW2;
    return;
  }

  if (attack_seq_state_ == AttackSeqState::TW2) {
    geometry_msgs::msg::Twist tw; tw.linear.x = 1.0;
    PublishTwistFor(tw, std::chrono::milliseconds(1000));
    attack_seq_state_ = AttackSeqState::TW3;
    return;
  }

  if (attack_seq_state_ == AttackSeqState::TW3) {
    geometry_msgs::msg::Twist tw; tw.linear.x = 1.0; tw.angular.z = 3.14;
    PublishTwistFor(tw, std::chrono::milliseconds(1000));
    attack_seq_state_ = AttackSeqState::IDLE;
    return;
  }
}

#ifdef DECISION_SIMPLE_HAS_AUTO_AIM
void DecisionSimple::test_is_detect_enemy()
{
  // XML：没检测到敌人就原地高速旋转 3s
  if (!IsDetectEnemy()) {
    geometry_msgs::msg::Twist tw;
    tw.angular.z = 6.28;
    PublishTwistFor(tw, std::chrono::milliseconds(3000));
  }
}
#endif

} // namespace decision_simple

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(decision_simple::DecisionSimple)
