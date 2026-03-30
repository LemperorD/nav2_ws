#include "decision_simple/decision_simple.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"                           
#include "tf2/exceptions.h"                     
#include "std_msgs/msg/u_int8.hpp"

namespace decision_simple
{

DecisionSimpleNode::DecisionSimpleNode(const rclcpp::NodeOptions & options)
: Node("decision_simple", options)
{
  // params
  onConfigure();

  // TF init
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());                           
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 

  // pubs
  chassis_mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>(chassis_mode_topic_, rclcpp::QoS(10));

  // action client
  nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // subs
  robot_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
    robot_status_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimpleNode::onRobotStatus, this, std::placeholders::_1));
  game_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::GameStatus>(
    game_status_topic_, rclcpp::QoS(10),
    std::bind(&DecisionSimpleNode::onGameStatus, this, std::placeholders::_1));

  // init state & chassis_mode
  setState(attack); setChassisMode(chassisFollowed);

  // timer
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / tick_hz_));
  timer_ = this->create_wall_timer(period, std::bind(&DecisionSimpleNode::tick, this));

  // info
  std::cout << "\033[32mCurrent State:" << state_ << "\033[0m" << std::endl;
  std::cout << "\033[32mCurrent State:" << state_ << "\033[0m" << std::endl;
  std::cout << "\033[32mCurrent Chassis Mode:" << chassis_mode_ << "\033[0m" << std::endl;
}

DecisionSimpleNode::~DecisionSimpleNode() {
  timer_-> reset();
  std::cout << "\033[32mDecision-Simple Node destroyed!\033[0m" << std::endl;
}

void DecisionSimpleNode::onConfigure() {
  require_game_running_ =this->declare_parameter<bool>("require_game_running", true); // for debug

  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_footprint");

  game_status_topic_ = this->declare_parameter<std::string>("referee_game_status_topic", "referee/game_status");
  robot_status_topic_ = this->declare_parameter<std::string>("referee_robot_status_topic", "referee/robot_status");
  chassis_mode_topic_ = this->declare_parameter<std::string>("chassis_mode_topic", "chassis_mode");

  start_delay_sec_ =this->declare_parameter<double>("start_delay_sec", 5.0);
  tol_radius_ =this->declare_parameter<double>("tol_radius", 0.80);

  supply_x_   = this->declare_parameter<double>("supply_x", 0.0);
  supply_y_   = this->declare_parameter<double>("supply_y", 0.0);

  default_x_   = this->declare_parameter<double>("default_x", 4.65);
  default_y_   = this->declare_parameter<double>("default_y", -3.5);

  tol_radius_min_ = this->declare_parameter<double>("tol_radius_min", 0.30);
  tol_radius_max_ = this->declare_parameter<double>("tol_radius_max", 0.80);

  hp_attack_ = this->declare_parameter<int>("hp_attack", 300);
  hp_heal_  = this->declare_parameter<int>("hp_heal", 120);       

  tick_hz_ = this->declare_parameter<double>("tick_hz", 20.0);
  goal_hz_ = this->declare_parameter<double>("goal_hz", 2.0);

  send_goal_options_.feedback_callback =
    [this](GoalHandleNav2::SharedPtr,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      std::cout << "Goal Feedback: Distance Remaining: " 
                << feedback->distance_remaining << std::endl;
    };

  send_goal_options_.result_callback =
    [this](const GoalHandleNav2::WrappedResult & result)
    {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        at_goal_ = true;
        tol_radius_ = tol_radius_max_;
        std::cout << "\033[32mGoal succeeded!\033[0m" << std::endl;
      } else {
        std::cout << "\033[31mGoal failed!\033[0m" << std::endl;
      }
    };
}

void DecisionSimpleNode::onRobotStatus(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) {
  last_robot_status_ = *msg;
  if(state_ == attack) isNeedHeal_ = isStatusBad(last_robot_status_);
  if(state_ == heal) isNeedHeal_ = isStatusRecovered(last_robot_status_);
}

void DecisionSimpleNode::onGameStatus(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg)
{
  const uint8_t prev = last_game_status_;
  last_game_status_ = msg->game_progress;
  // 从“非比赛中” -> “比赛中(4)”，开始倒计时
  if (prev != 4 && last_game_status_ == 4) {
    match_started_ = true;
    match_start_time_ = this->now();
  }
}

void DecisionSimpleNode::tick()
{
  if (require_game_running_) {
    if (!match_started_) {
      std::cout << "\033[33mwaiting for game...\033[0m" << std::endl;
      return;
    }
    const double elapsed = (now() - match_start_time_).seconds();
    if (elapsed < start_delay_sec_) {
      std::cout << "\033[33mwaiting for start_delay...\033[0m" << std::endl;
      return;
    }
  }

  uint8_t current_state = isNeedHeal_ ? heal : attack;
  setState(current_state); setChassisMode();
}

void DecisionSimpleNode::setState(uint8_t s) {
  if (state_ == s) {
    if (!isNear(tol_radius_max_)) pubGoal();
    return;
  }
  state_ = s;
  std::cout << "\033[32mCurrent State:" << state_ << "\033[0m" << std::endl;
  pubGoal();
}

void DecisionSimpleNode::setChassisMode() {
  uint8_t mode;
  if (isNear() && at_goal_) mode = littleTES;
  else mode = chassisFollowed;

  if (chassis_mode_ != mode) {
    chassis_mode_ = mode;
    std::cout << "\033[32mCurrent Chassis Mode:" << chassis_mode_ << "\033[0m" << std::endl;
  }
  std_msgs::msg::UInt8 m;
  m.data = chassis_mode_;
  chassis_mode_pub_->publish(m);
}

void DecisionSimpleNode::setChassisMode(uint8_t mode) {
  if (chassis_mode_ == mode) return;
  chassis_mode_ = mode;
  std_msgs::msg::UInt8 m;
  m.data = chassis_mode_;
  chassis_mode_pub_->publish(m);
}

void DecisionSimpleNode::pubGoal()
{
  if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      std::cout << "\033[33mNav2 action server not available...\033[0m" << std::endl;
    return;
  }

  current_goal_.pose.header.frame_id = map_frame_id_;
  if(state_ == attack) {
    current_goal_.pose.pose.position.set__x(default_x_);
    current_goal_.pose.pose.position.set__y(default_y_);
  } else {
    current_goal_.pose.pose.position.set__x(supply_x_);
    current_goal_.pose.pose.position.set__y(supply_y_);
  }
  current_goal_.pose.pose.orientation.w = 1.0;

  nav_action_client_->async_send_goal(current_goal_, send_goal_options_);
  std::cout << "\033[32mNew Goal Publish: " 
            << "x: " << current_goal_.pose.pose.position.x << ", "
            << "y: " << current_goal_.pose.pose.position.y << std::endl;
  tol_radius_ = tol_radius_min_; at_goal_ = false;
}

inline bool DecisionSimpleNode::isNear() {
  double current_x, current_y;
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_id_, base_frame_id_, tf2::TimePointZero);
    current_x = tf.transform.translation.x;
    current_y = tf.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    std::cout << "\033[33mNo TF!\033[0m" << std::endl;
    return false;
  }

  const double dx = current_x - current_goal_.pose.pose.position.x;
  const double dy = current_y - current_goal_.pose.pose.position.y;
  return std::hypot(dx, dy) <= tol_radius_;
}

inline bool DecisionSimpleNode::isNear(double tol_radius) {
  double current_x, current_y;
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_id_, base_frame_id_, tf2::TimePointZero);
    current_x = tf.transform.translation.x;
    current_y = tf.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    std::cout << "\033[33mNo TF!\033[0m" << std::endl;
    return false;
  }

  const double dx = current_x - current_goal_.pose.pose.position.x;
  const double dy = current_y - current_goal_.pose.pose.position.y;
  return std::hypot(dx, dy) <= tol_radius;
}

inline bool DecisionSimpleNode::isStatusBad(const pb_rm_interfaces::msg::RobotStatus & rs) {
  const int hp = static_cast<int>(rs.current_hp);
  return hp < hp_heal_;
}

inline bool DecisionSimpleNode::isStatusRecovered(const pb_rm_interfaces::msg::RobotStatus & rs) {
  const int hp = static_cast<int>(rs.current_hp);
  return hp > hp_attack_;
}

} // namespace decision_simple

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(decision_simple::DecisionSimpleNode)