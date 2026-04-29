#include "decision_simple/node/decision_simple.hpp"
namespace decision_simple {
  using nanoseconds = int64_t;

  DecisionSimple::DecisionSimple(const rclcpp::NodeOptions& options)
      : Node("decision_simple", options) {
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
    detector_armors_topic_ = this->declare_parameter<std::string>(
        "detector_armors_topic", "detector/armors");
    tracker_target_topic_ = this->declare_parameter<std::string>(
        "tracker_target_topic", "tracker/target");

    const bool require_game_running = this->declare_parameter<bool>(
        "require_game_running", true);
    const double start_delay_sec = this->declare_parameter<double>(
        "start_delay_sec", 5.0);
    const double default_spin_keep_xy_tol_ = this->declare_parameter<double>(
        "default_spin_keep_xy_tol", 0.80);
    const double supply_x = this->declare_parameter<double>("supply_x", 0.0);
    const double supply_y = this->declare_parameter<double>("supply_y", 0.0);
    const double supply_yaw = this->declare_parameter<double>("supply_yaw",
                                                              0.0);
    const double default_x_ = this->declare_parameter<double>("default_x", 2.0);
    const double default_y_ = this->declare_parameter<double>("default_y", 0.5);
    const double default_yaw_ = this->declare_parameter<double>("default_yaw",
                                                                0.0);
    const double default_arrive_xy_tol_ = this->declare_parameter<double>(
        "default_arrive_xy_tol", 0.30);
    const int hp_enter_supply = this->declare_parameter<int>(
        "hp_survival_enter", 120);
    const int hp_exit_supply = this->declare_parameter<int>("hp_survival_exit",
                                                            300);
    const int ammo_min = this->declare_parameter<int>("ammo_min", 0);
    const double combat_max_distance = this->declare_parameter<double>(
        "combat_max_distance", 8.0);

    const ContextConfig context_config{hp_enter_supply,
                                       hp_exit_supply,
                                       ammo_min,
                                       combat_max_distance,
                                       require_game_running,
                                       start_delay_sec,
                                       default_x_,
                                       default_y_,
                                       default_yaw_,
                                       supply_x,
                                       supply_y,
                                       supply_yaw,
                                       default_arrive_xy_tol_,
                                       default_spin_keep_xy_tol_};
    environment_ = std::make_unique<EnvironmentContext>(context_config);
    controller_ = std::make_unique<Decision>(context_config);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_, rclcpp::SensorDataQoS());
    chassis_mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        chassis_mode_topic_, rclcpp::QoS(10));
    debug_attack_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            debug_attack_pose_topic_, rclcpp::QoS(10));

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
    setAndLogState(State::DEFAULT);
    publishChassisMode(ChassisMode::CHASSIS_FOLLOWED);

    // timer
    const double hz = std::max(1e-6, tick_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz));
    timer_ = this->create_wall_timer(period,
                                     std::bind(&DecisionSimple::tick, this));

    tick_hz_ = this->declare_parameter<double>("tick_hz", 20.0);
    default_goal_hz_ = this->declare_parameter<double>("default_goal_hz", 2.0);
    supply_goal_hz_ = this->declare_parameter<double>("supply_goal_hz", 2.0);
    attack_goal_hz_ = this->declare_parameter<double>("attack_goal_hz", 10.0);

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
    environment_->onRobotStatus(ConvertRobotStatus(msg));
  }

  void DecisionSimple::onGameStatus(
      const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    environment_->onGameStatus(ConvertGameStatus(msg),
                               this->now().nanoseconds());

    // 从“非比赛中” -> “比赛中(4)”，开始新一轮倒计时
    if (environment_->isGameStarted()) {
      RCLCPP_INFO(this->get_logger(),
                  "Game started detected, delaying decision for %.1f seconds",
                  start_delay_sec_);
    }

    // 从“比赛中(4)” -> “非比赛中”，复位，为下一局做准备
    if (environment_->isGameOver()) {
      // 清掉上一局遗留的默认点小陀螺锁存
      default_spin_latched_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Game left running state, reset match-start gate.");
      environment_->resetGameOver();
    }
  }

  void DecisionSimple::onArmors(
      const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);

    environment_->onArmors(ConvertArmors(msg));
  }

  void DecisionSimple::onTarget(
      const auto_aim_interfaces::msg::Target::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);

    environment_->onTarget(ConvertTarget(msg));
  }

  void DecisionSimple::tick() {
    // snapshot
    double x{}, y{}, yaw{};
    const auto now = this->now();

    auto readiness = environment_->checkReadiness(now.nanoseconds());

    if (readiness.status != Readiness::Status::READY) {
      handleGateLog(readiness);
      return;
    }

    if (this->getRobotPoseMap({x, y, yaw})) {
      environment_->updatePose(x, y, yaw);
    }

    Snapshot snapshot = environment_->getSnapshot(makeStamped(now));

    auto action = controller_->computeAction(snapshot);

    executeAction(action);
  }

  Stamp DecisionSimple::makeStamped(const rclcpp::Time time) {
    Stamp stamp{static_cast<int32_t>(time.seconds()),
                static_cast<uint32_t>(time.nanoseconds() % 1000000000UL)};
    return stamp;
  }

  void DecisionSimple::executeAction(DecisionAction action) {
    setAndLogState(action.next_state);
    publishChassisMode(action.chassis_mode);

    if (action.should_publish_goal) {
      publishGoal(action);
    }
  }

  void DecisionSimple::publishGoal(const DecisionAction& action) {
    const auto goal = makePoseXYZYaw(
        frame_id_, {action.target_x, action.target_y, action.target_yaw});

    // For attack state, also publish debug pose
    if (action.next_state == State::ATTACK) {
      debug_attack_pose_pub_->publish(goal);
    }

    // Throttle based on state
    if (action.next_state == State::SUPPLY) {
      publishGoalThrottled(goal, last_supply_pub_, supply_goal_hz_);
    } else if (action.next_state == State::ATTACK) {
      publishGoalThrottled(goal, last_attack_pub_, attack_goal_hz_);
    } else {
      publishGoalThrottled(goal, last_default_pub_, default_goal_hz_);
    }
  }

  geometry_msgs::msg::PoseStamped DecisionSimple::makePoseXYZYaw(
      const std::string& frame, const Pose2D& position) const {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame;
    p.header.stamp = this->now();
    p.pose.position.x = position.x;
    p.pose.position.y = position.y;
    p.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, position.yaw);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    return p;
  }

  // 获取机器人在 map 下的位置
  bool DecisionSimple::getRobotPoseMap(Pose2D position) {
    try {
      const auto tf = tf_buffer_->lookupTransform(frame_id_, base_frame_id_,
                                                  tf2::TimePointZero);
      position.x = tf.transform.translation.x;
      position.y = tf.transform.translation.y;
      const double qx = tf.transform.rotation.x;
      const double qy = tf.transform.rotation.y;
      const double qz = tf.transform.rotation.z;
      const double qw = tf.transform.rotation.w;
      position.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "TF lookup failed (%s -> %s): %s",
                            frame_id_.c_str(), base_frame_id_.c_str(),
                            ex.what());
      return false;
    }
  }

  void DecisionSimple::setAndLogState(State s) {
    environment_->setState(s);

    if (environment_->isStateChanged()) {
      RCLCPP_INFO(this->get_logger(), "State -> %u",
                  static_cast<unsigned>(state_));
    }
  }

  void DecisionSimple::publishChassisMode(ChassisMode mode) {
    std_msgs::msg::UInt8 m;
    m.data = static_cast<uint8_t>(mode);
    chassis_mode_pub_->publish(m);
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

  void DecisionSimple::handleGateLog(Readiness& readiness) {
    (void)readiness;
    return;
  }

}  // namespace decision_simple

// namespace decision_simple

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(decision_simple::DecisionSimple)