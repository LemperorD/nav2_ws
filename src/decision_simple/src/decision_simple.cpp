// ============================
// Black-box Behavior Spec (for TDD)
// ============================
// Inputs:
// - referee/game_status: game_progress
// - referee/robot_status: current_hp, projectile_allowance_17mm, is_hp_deduced
// - (optional) enemy topics: detector/armors, tracker/target
//
// Outputs:
// - chassis_mode: default 1
// - goal_pose: default default_point
//
// Global gate:
// 1) If robot_status has not been received, do not publish decision outputs.
// 2) If require_game_running=true, decisions are enabled only when
//    game_progress == RUNNING and start_delay_sec has elapsed.
//
// Priority (high -> low):
// 1) Supply-first:
//    - If hp below enter threshold OR ammo below threshold, enter SUPPLY.

#include "decision_simple/node/decision_simple.hpp"

#include <algorithm>
#include <cmath>

#include "decision_simple/adapter/transform.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
namespace decision_simple {

  DecisionSimple::DecisionSimple(const rclcpp::NodeOptions& options)
      : Node("decision_simple", options) {
    environment_ = std::make_unique<EnvironmentContext>();

    // ===== params =====
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id",
                                                          "base_footprint");

    robot_status_topic_ = this->declare_parameter<std::string>(
        "referee_robot_status_topic", "referee/robot_status");
    goal_pose_topic_ = this->declare_parameter<std::string>("goal_pose_topic",
                                                            "goal_pose");

    chassis_mode_topic_ = this->declare_parameter<std::string>(
        "chassis_mode_topic", "chassis_mode");
    debug_attack_pose_topic_ = this->declare_parameter<std::string>(
        "debug_attack_pose_topic", "debug_attack_pose");
    game_status_topic_ = this->declare_parameter<std::string>(
        "referee_game_status_topic", "referee/game_status");
    require_game_running_ = this->declare_parameter<bool>(
        "require_game_running", true);
    start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 5.0);
    default_spin_keep_xy_tol_ = this->declare_parameter<double>(
        "default_spin_keep_xy_tol", 0.80);
    detector_armors_topic_ = this->declare_parameter<std::string>(
        "detector_armors_topic", "detector/armors");
    tracker_target_topic_ = this->declare_parameter<std::string>(
        "tracker_target_topic", "tracker/target");

    supply_x_ = this->declare_parameter<double>("supply_x", 0.0);
    supply_y_ = this->declare_parameter<double>("supply_y", 0.0);
    supply_yaw_ = this->declare_parameter<double>("supply_yaw", 0.0);

    default_x_ = this->declare_parameter<double>("default_x", 2.0);
    default_y_ = this->declare_parameter<double>("default_y", 0.5);
    default_yaw_ = this->declare_parameter<double>("default_yaw", 0.0);

    default_arrive_xy_tol_ = this->declare_parameter<double>(
        "default_arrive_xy_tol", 0.30);
    supply_arrive_xy_tol_ = this->declare_parameter<double>(
        "supply_arrive_xy_tol", 0.30);

    hp_enter_supply_ = this->declare_parameter<int>("hp_survival_enter", 120);
    hp_exit_supply_ = this->declare_parameter<int>("hp_survival_exit", 300);
    ammo_min_ = this->declare_parameter<int>("ammo_min", 0);

    combat_max_distance_ = this->declare_parameter<double>(
        "combat_max_distance", 8.0);
    attack_hold_sec_ = this->declare_parameter<double>("combat_cooldown_sec",
                                                       1.0);

    attacked_hold_sec_ = this->declare_parameter<double>("attacked_hold_sec",
                                                         1.5);

    tick_hz_ = this->declare_parameter<double>("tick_hz", 20.0);
    default_goal_hz_ = this->declare_parameter<double>("default_goal_hz", 2.0);
    supply_goal_hz_ = this->declare_parameter<double>("supply_goal_hz", 2.0);
    attack_goal_hz_ = this->declare_parameter<double>("attack_goal_hz", 10.0);

    // ===== TF init =====
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ===== pubs =====
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_, rclcpp::SensorDataQoS());
    chassis_mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        chassis_mode_topic_, rclcpp::QoS(10));
    debug_attack_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            debug_attack_pose_topic_, rclcpp::QoS(10));

    // ===== subs =====
    robot_status_sub_ =
        this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
            robot_status_topic_, rclcpp::QoS(10),
            std::bind(&DecisionSimple::onRobotStatus, this,
                      std::placeholders::_1));
    game_status_sub_ =
        this->create_subscription<pb_rm_interfaces::msg::GameStatus>(
            game_status_topic_, rclcpp::QoS(10),
            std::bind(&DecisionSimple::onGameStatus, this,
                      std::placeholders::_1));
    armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
        detector_armors_topic_, rclcpp::QoS(10),
        std::bind(&DecisionSimple::onArmors, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        tracker_target_topic_, rclcpp::QoS(10),
        std::bind(&DecisionSimple::onTarget, this, std::placeholders::_1));

    // init
    setState(State::DEFAULT);

    setChassisMode(ChassisMode::CHASSIS_FOLLOWED);

    // timer
    const double hz = std::max(1e-6, tick_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz));
    timer_ = this->create_wall_timer(period,
                                     std::bind(&DecisionSimple::tick, this));

    RCLCPP_INFO(this->get_logger(),
                "decision_simple(min+mode) started. ns=%s goal_pose=%s "
                "robot_status=%s chassis_mode=%s",
                this->get_namespace(), goal_pose_topic_.c_str(),
                robot_status_topic_.c_str(), chassis_mode_topic_.c_str());
  }

  // ================= callbacks =================
  void DecisionSimple::onRobotStatus(
      const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_robot_status_ = *msg;
    has_robot_status_ = true;
  }
  void DecisionSimple::onGameStatus(
      const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);

    const uint8_t prev = last_game_status_;
    last_game_status_ = msg->game_progress;
    has_game_status_ = true;
    // 从“非比赛中” -> “比赛中(4)”，开始新一轮倒计时
    if (prev != 4 && last_game_status_ == 4) {
      match_started_ = true;
      match_start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(),
                  "Game started detected, delaying decision for %.1f seconds",
                  start_delay_sec_);
    }

    // 从“比赛中(4)” -> “非比赛中”，复位，为下一局做准备
    if (prev == 4 && last_game_status_ != 4) {
      match_started_ = false;
      match_start_time_ = rclcpp::Time(0, 0,
                                       this->get_clock()->get_clock_type());
      // 清掉上一局遗留的默认点小陀螺锁存
      default_spin_latched_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Game left running state, reset match-start gate.");
    }
  }
  void DecisionSimple::onArmors(
      const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_armors_ = *msg;
    has_armors_ = true;
  }

  void DecisionSimple::onTarget(
      const auto_aim_interfaces::msg::Target::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_target_opt_ = *msg;
  }
  // ================= tick：决策 + 底盘模式切换 =================
  void DecisionSimple::tick() {
    // snapshot
    pb_rm_interfaces::msg::RobotStatus rs;
    bool has_rs = false;
    bool has_gs = false;
    bool match_started = false;
    rclcpp::Time match_start_time;
    auto_aim_interfaces::msg::Armors armors;
    bool has_armors = false;
    std::optional<auto_aim_interfaces::msg::Target> target_opt;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      has_rs = has_robot_status_;
      if (has_rs) {
        rs = last_robot_status_;
      }
      has_gs = has_game_status_;
      match_started = match_started_;
      match_start_time = match_start_time_;
      has_armors = has_armors_;
      if (has_armors) {
        armors = last_armors_;
      }
      target_opt = last_target_opt_;
    }

    if (!has_rs) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Waiting for robot_status...");
      return;
    }

    const auto now = this->now();
    // 比赛开始+5s延时
    if (require_game_running_) {
      if (!has_gs) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Waiting for game_status...");
        return;
      }

      if (!match_started) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Waiting for match start (game_status == 4)...");
        return;
      }

      const double elapsed = (now - match_start_time).seconds();
      if (elapsed < start_delay_sec_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Match started, delaying decision: %.1f / %.1f s",
                             elapsed, start_delay_sec_);
        return;
      }
    }

    // 受到攻击检测
    if (rs.is_hp_deduced) {
      last_attacked_ = now;
    }
    const bool attacked_recent = (last_attacked_.nanoseconds() != 0)
                              && ((now - last_attacked_).seconds()
                                  <= attacked_hold_sec_);

    // 到达判断（中心点/补给点）
    const bool at_center = isNear(default_x_, default_y_,
                                  default_arrive_xy_tol_);
    const bool in_center_keep_spin = isNear(
        default_x_, default_y_, default_spin_keep_xy_tol_);  // 确认是否在大圈

    // ================= 1) 补给保持 =================
    if (state_ == State::SUPPLY) {
      if (!isStatusRecovered(rs)) {
        setChassisMode(ChassisMode::CHASSIS_FOLLOWED);

        const auto goal = makePoseXYZYaw(frame_id_, supply_x_, supply_y_,
                                         supply_yaw_);
        publishGoalThrottled(goal, last_supply_pub_, supply_goal_hz_);
        return;
      }
    }

    // ================= 2) 状态差 -> 进补给 =================
    if (isStatusBad(rs)) {
      setState(State::SUPPLY);

      setChassisMode(ChassisMode::CHASSIS_FOLLOWED);

      const auto goal = makePoseXYZYaw(frame_id_, supply_x_, supply_y_,
                                       supply_yaw_);
      publishGoalThrottled(goal, last_supply_pub_, supply_goal_hz_);
      return;
    }

    // ================= 3) 状态好 -> 有敌攻击 =================
    bool enemy = false;
    if (has_armors || target_opt.has_value()) {
      enemy = detectEnemy(armors, target_opt);
    }
    if (enemy) {
      last_enemy_seen_ = now;
    }

    const bool enemy_recent = (last_enemy_seen_.nanoseconds() != 0)
                           && ((now - last_enemy_seen_).seconds()
                               <= attack_hold_sec_);

    if (enemy_recent) {
      setState(State::ATTACK);
      default_spin_latched_ = false;
      if (attacked_recent) {
        setChassisMode(ChassisMode::LITTLE_TES);
      } else {
        setChassisMode(ChassisMode::CHASSIS_FOLLOWED);
      }

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
        publishGoalThrottled(last_attack_goal_, last_attack_pub_,
                             attack_goal_hz_);
      }
      return;
    }
    // ================= 4) 默认：去中心点 =================
    setState(State::DEFAULT);
    // 1. 先进入 0.3m 小圈 -> 小陀螺模式
    if (at_center) {
      default_spin_latched_ = true;
    }
    // 2. 出了 0.8m 大圈，取消陀螺
    if (!in_center_keep_spin) {
      default_spin_latched_ = false;
    }
    if (attacked_recent) {
      setChassisMode(ChassisMode::LITTLE_TES);
    } else {
      setChassisMode(default_spin_latched_ ? ChassisMode::LITTLE_TES
                                           : ChassisMode::CHASSIS_FOLLOWED);
    }
    const auto goal = makePoseXYZYaw(frame_id_, default_x_, default_y_,
                                     default_yaw_);
    publishGoalThrottled(goal, last_default_pub_, default_goal_hz_);
  }

  // ================= helpers =================
  geometry_msgs::msg::PoseStamped DecisionSimple::makePoseXYZYaw(
      const std::string& frame, double x, double y, double yaw) const {
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

  bool DecisionSimple::isStatusBad(
      const pb_rm_interfaces::msg::RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp < hp_enter_supply_) || (ammo <= ammo_min_);
  }

  bool DecisionSimple::isStatusRecovered(
      const pb_rm_interfaces::msg::RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp >= hp_exit_supply_) && (ammo > ammo_min_);
  }

  // 获取机器人在 map 下的位置
  bool DecisionSimple::getRobotPoseMap(double& x, double& y, double& yaw) {
    try {
      const auto tf = tf_buffer_->lookupTransform(frame_id_, base_frame_id_,
                                                  tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      yaw = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "TF lookup failed (%s -> %s): %s",
                            frame_id_.c_str(), base_frame_id_.c_str(),
                            ex.what());
      return false;
    }
  }

  // 是否到达某点（只看平面距离）
  bool DecisionSimple::isNear(double gx, double gy, double tol_xy) {
    double x, y, yaw;
    (void)yaw;
    if (!getRobotPoseMap(x, y, yaw)) {
      return false;
    }
    const double dx = x - gx;
    const double dy = y - gy;
    return std::hypot(dx, dy) <= tol_xy;
  }

  bool DecisionSimple::detectEnemy(
      const auto_aim_interfaces::msg::Armors& armors,
      const std::optional<auto_aim_interfaces::msg::Target>& target_opt) const {
    if (target_opt.has_value() && target_opt->tracking) {
      const double x = target_opt->position.x;
      const double y = target_opt->position.y;
      const double z = target_opt->position.z;
      const double dist = std::sqrt(x * x + y * y + z * z);
      return dist <= combat_max_distance_;
    }

    for (const auto& a : armors.armors) {
      const double x = a.pose.position.x;
      const double y = a.pose.position.y;
      const double z = a.pose.position.z;
      const double dist = std::sqrt(x * x + y * y + z * z);
      if (dist <= combat_max_distance_) {
        return true;
      }
    }
    return false;
  }

  bool DecisionSimple::buildAttackGoal(
      geometry_msgs::msg::PoseStamped& out,
      const auto_aim_interfaces::msg::Armors& armors,
      const std::optional<auto_aim_interfaces::msg::Target>& target_opt) const {
    if (target_opt.has_value() && target_opt->tracking) {
      const std::string frame = (!target_opt->header.frame_id.empty())
                                  ? target_opt->header.frame_id
                                  : frame_id_;
      out = makePoseXYZYaw(frame, target_opt->position.x,
                           target_opt->position.y, target_opt->yaw);
      return true;
    }

    if (armors.armors.empty()) {
      return false;
    }

    const auto* best = &armors.armors.front();
    double best_dist = 1e18;
    for (const auto& a : armors.armors) {
      const double x = a.pose.position.x;
      const double y = a.pose.position.y;
      const double z = a.pose.position.z;
      const double dist = std::sqrt(x * x + y * y + z * z);
      if (dist < best_dist) {
        best_dist = dist;
        best = &a;
      }
    }

    const std::string frame = (!armors.header.frame_id.empty())
                                ? armors.header.frame_id
                                : frame_id_;
    out = makePoseXYZYaw(frame, best->pose.position.x, best->pose.position.y,
                         0.0);
    return true;
  }

  void DecisionSimple::setState(State s) {
    if (state_ == s) {
      return;
    }
    state_ = s;

    // 不再用 state_ 去发布 chassis_mode
    RCLCPP_INFO(this->get_logger(), "State -> %u",
                static_cast<unsigned>(state_));
  }

  //
  void DecisionSimple::publishChassisMode(ChassisMode mode) {
    std_msgs::msg::UInt8 m;
    m.data = static_cast<uint8_t>(mode);
    chassis_mode_pub_->publish(m);
  }

  void DecisionSimple::setChassisMode(ChassisMode mode) {
    current_chassis_mode_ = mode;
    publishChassisMode(mode);
  }

  void DecisionSimple::publishGoalThrottled(
      const geometry_msgs::msg::PoseStamped& goal, rclcpp::Time& last_pub,
      double hz) {
    const double period = (hz <= 1e-6) ? 1e9 : (1.0 / hz);
    const auto now = this->now();

    if (last_pub.nanoseconds() == 0 || (now - last_pub).seconds() >= period) {
      last_pub = now;
      goal_pose_pub_->publish(goal);
    }
  }
}  // namespace decision_simple

// namespace decision_simple

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(decision_simple::DecisionSimple)