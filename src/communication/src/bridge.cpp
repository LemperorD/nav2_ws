#include "communication/bridge.hpp"
#include "misc/termcolor.hpp"

#include <algorithm>
#include <cstring>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace bridge
{

BridgeNode::BridgeNode(const rclcpp::NodeOptions & options)
: Node("bridge", options),
  port_name_("/dev/ttyACM0"),
  baud_rate_(115200)
{
  this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<double>("Yaw_bias", 0.0);
  this->declare_parameter<int>("max_dwa_size", 15);
  this->declare_parameter<double>("vel_trans_scale", 40.0);

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("Yaw_bias", Yaw_bias_);
  this->get_parameter("max_dwa_size", max_dwa_size_);
  this->get_parameter("vel_trans_scale", vel_trans_scale_);

  com_ = std::make_shared<SerialCommunicationClass>(this, port_name_, baud_rate_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  lidar_connected_ = isLidarConnected();
  std::cout << "Lidar connected: " << std::boolalpha << lidar_connected_ << std::endl;
  
  // 模板类桥接器
  bridge_twist_pc_ = std::make_shared<RosSerialBridge
    <geometry_msgs::msg::Twist>>(
      this, "/cmd_vel", true,
      std::bind(&BridgeNode::encodeTwist, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::sendDataFrame, com_.get(), std::placeholders::_1, std::placeholders::_2),
      nullptr, nullptr);

  bridge_Yaw_mcu_ = std::make_shared<RosSerialBridge
    <std_msgs::msg::Float32>>(
      this, "/serial/Yaw", false, nullptr,
      std::bind(&BridgeNode::decodeYaw, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::receiveDataFrame, com_.get()),
      nullptr);

  bridge_TESspeed_mcu_ = std::make_shared<RosSerialBridge
    <geometry_msgs::msg::Twist>>(
      this, "/serial/TES_speed", false, nullptr,
      std::bind(&BridgeNode::decodeTESspeed, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::receiveDataFrame, com_.get()),
      nullptr);

  bridge_EnemyPos_mcu_ = std::make_shared<RosSerialBridge
    <geometry_msgs::msg::Point>>(
      this, "/serial/EnemyPos", false, nullptr,
      std::bind(&BridgeNode::decodeEnemyPos, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::receiveDataFrame, com_.get()),
      nullptr);

  // 定时器
  gimbal_vision_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&BridgeNode::publishTransformGimbalVision, this));

  referee_rx_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&BridgeNode::publishRefereeData, this));

  // 多输入/输出的订阅和发布
  chassis_mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "chassis_mode", 10,
    [this](const std_msgs::msg::UInt8::SharedPtr msg) {
      chassis_mode_ = static_cast<uint8_t>(msg->data);
    });

  robot_status_pub_ = this->create_publisher<pb_rm_interfaces::msg::RobotStatus>(
    "/referee/robot_status", 10);

  game_status_pub_ = this->create_publisher<pb_rm_interfaces::msg::GameStatus>(
    "/referee/game_status", 10);

  rfid_status_pub_ = this->create_publisher<pb_rm_interfaces::msg::RfidStatus>(
    "/referee/rfid_status", 10);
}

BridgeNode::~BridgeNode()
{
  std::cout << "Shutting down BridgeNode..." << std::endl;

  // reset timers
  if (gimbal_vision_timer_) gimbal_vision_timer_.reset();
  if (referee_rx_timer_) referee_rx_timer_.reset();

  // reset bridge
  bridge_twist_pc_.reset(); bridge_Yaw_mcu_.reset();
  bridge_TESspeed_mcu_.reset(); bridge_EnemyPos_mcu_.reset();

  // reset subscribers and publishers
  chassis_mode_sub_.reset(); robot_status_pub_.reset();
  game_status_pub_.reset(); rfid_status_pub_.reset();

  // reset serial communication
  com_.reset();

  std::cout << "BridgeNode shutdown complete." << std::endl;
}

uint8_t* BridgeNode::encodeTwist(const geometry_msgs::msg::Twist& msg)
{
  geometry_msgs::msg::Twist twist_chassis = transformVelocityToChassis(msg, yaw_diff_ * M_PI / 180.0);

  const float vx = static_cast<float>(vel_trans_scale_ * msg.linear.x);
  const float vy = static_cast<float>(vel_trans_scale_ * msg.linear.y);
  const float wz = static_cast<float>(msg.angular.z);

  const float vx_Y = static_cast<float>(vel_trans_scale_ * twist_chassis.linear.x);
  const float vy_Y = static_cast<float>(vel_trans_scale_ * twist_chassis.linear.y);

  uint8_t* payload = new uint8_t[26]();

  payload[0] = chassis_mode_;

  com_->writeFloatLE(&payload[1], static_cast<float>(angle_init_));

  payload[5] = true;

  com_->writeFloatLE(&payload[6], vx_Y); 
  com_->writeFloatLE(&payload[10], vy_Y);

  com_->writeFloatLE(&payload[14], vx);
  com_->writeFloatLE(&payload[18], vy);
  com_->writeFloatLE(&payload[22], -wz);

  return payload;
}

std_msgs::msg::Float32 BridgeNode::decodeYaw(const uint8_t* payload)
{
  std_msgs::msg::Float32 msg;
  std::memcpy(&yaw_diff_, payload + 7, sizeof(float));
  msg.data = yaw_diff_;
  // publishTransformGimbalYaw(msg.data);
  return msg;
}

geometry_msgs::msg::Twist BridgeNode::decodeTESspeed(const uint8_t* payload)
{
  geometry_msgs::msg::Twist msg;
  msg.angular.z = static_cast<double>(com_->readFloatLE(&payload[3]));
  return msg;
}

geometry_msgs::msg::Point BridgeNode::decodeEnemyPos(const uint8_t* payload)
{
  geometry_msgs::msg::Point msg;
  int16_t x, y;
  std::memcpy(&x, payload + 11, sizeof(int16_t));
  std::memcpy(&y, payload + 13, sizeof(int16_t));
  msg.x = static_cast<double>(x); msg.y = static_cast<double>(y);
  return msg;
}

void BridgeNode::publishTransformGimbalVision()
{
  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    transformStamped = tf_buffer_->lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    if(lidar_connected_) std::cerr << "Could not get transform: " << ex.what() << std::endl;
    return;
  }

  tf2::Quaternion q(
    transformStamped.transform.rotation.x,
    transformStamped.transform.rotation.y,
    transformStamped.transform.rotation.z,
    transformStamped.transform.rotation.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  geometry_msgs::msg::TransformStamped gimbal_tf;
  gimbal_tf.header.stamp = this->get_clock()->now();
  gimbal_tf.header.frame_id = "base_footprint";
  gimbal_tf.child_frame_id = "gimbal_yaw_vision";

  gimbal_tf.transform.translation.x = 0.0;
  gimbal_tf.transform.translation.y = 0.0;
  gimbal_tf.transform.translation.z = 0.0;

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, -yaw);

  gimbal_tf.transform.rotation.x = q_yaw.x();
  gimbal_tf.transform.rotation.y = q_yaw.y();
  gimbal_tf.transform.rotation.z = q_yaw.z();
  gimbal_tf.transform.rotation.w = q_yaw.w();

  tf_broadcaster_->sendTransform(gimbal_tf);
}

void BridgeNode::publishTransformGimbalYaw(double yaw)
{
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = this->get_clock()->now();
  tf_msg.header.frame_id = "gimbal_yaw_odom";
  tf_msg.child_frame_id = "gimbal_yaw";

  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, -dwa_filter(yaw) + Yaw_bias_);

  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf_msg);
}

inline double BridgeNode::dwa_filter(double sample)
{
  dwa_.push_back(sample);
  if (dwa_.size() > max_dwa_size_) dwa_.pop_front();

  double sum = 0.0;
  for (double x : dwa_) sum += x;
  return dwa_.empty() ? sample : (sum / dwa_.size());
}

inline geometry_msgs::msg::Twist BridgeNode::transformVelocityToChassis(
  const geometry_msgs::msg::Twist & twist_in, double yaw_diff)
{
  geometry_msgs::msg::Twist out;
  out.linear.x = twist_in.linear.x * std::cos(yaw_diff) - twist_in.linear.y * std::sin(yaw_diff);
  out.linear.y = twist_in.linear.x * std::sin(yaw_diff) + twist_in.linear.y * std::cos(yaw_diff);
  out.angular.z = twist_in.angular.z;
  return out;
}

void BridgeNode::publishRefereeData()
{

  const uint8_t *payload = com_->receiveRefereeFrame();
  if (payload == nullptr) {
    std::cerr << "Failed to receive referee frame." << std::endl;
    return;
  }

  // const uint8_t game_type = payload[0] >> 4; // TODO: 解析比赛类型数据
  uint8_t game_progress;
  std::memcpy(&game_progress, payload, sizeof(uint8_t)); // 解析比赛进程!
  game_progress = (game_progress >> 4); // 取高4位作为比赛进程
  //game_progress = 4;

  uint16_t current_hp, ammo17, heat1, heat2; // 当前血量、剩余17mm弹量、热量1、热量2
  std::memcpy(&current_hp, payload + 1, sizeof(uint16_t));
  std::memcpy(&ammo17, payload + 3, sizeof(uint16_t));
  std::memcpy(&heat1, payload + 5, sizeof(uint16_t));
  // std::memcpy(&heat2, payload + 7, sizeof(uint16_t));
   
  uint32_t rfid;
  std::memcpy(&rfid, payload + 9, sizeof(uint32_t));

  // publish robot status
  pb_rm_interfaces::msg::RobotStatus robot_status;
  robot_status.current_hp = current_hp;
  robot_status.projectile_allowance_17mm = ammo17;
  robot_status.shooter_17mm_1_barrel_heat = heat1;
  robot_status_pub_->publish(robot_status);

  // publish game status
  pb_rm_interfaces::msg::GameStatus game_status;
  game_status.game_progress = game_progress;
  game_status_pub_->publish(game_status);

  // publish RFID status
  pb_rm_interfaces::msg::RfidStatus rfid_status;
  rfid_status = rfid2ros(rfid);
  rfid_status_pub_->publish(rfid_status);

  com_->clearRefereeFrameFlag();
}

pb_rm_interfaces::msg::RfidStatus BridgeNode::rfid2ros(uint32_t rfid)
{
  pb_rm_interfaces::msg::RfidStatus status;
  status.base_gain_point = (rfid >> 0) & 0x01;
  status.central_highland_gain_point = (rfid >> 1) & 0x01;
  status.enemy_central_highland_gain_point = (rfid >> 2) & 0x01;
  status.friendly_trapezoidal_highland_gain_point = (rfid >> 3) & 0x01;
  status.enemy_trapezoidal_highland_gain_point = (rfid >> 4) & 0x01;
  status.friendly_fly_ramp_front_gain_point = (rfid >> 5) & 0x01;
  status.friendly_fly_ramp_back_gain_point = (rfid >> 6) & 0x01;
  status.enemy_fly_ramp_front_gain_point = (rfid >> 7) & 0x01;
  status.enemy_fly_ramp_back_gain_point = (rfid >> 8) & 0x01;
  status.friendly_central_highland_lower_gain_point = (rfid >> 9) & 0x01;
  status.friendly_central_highland_upper_gain_point = (rfid >> 10) & 0x01;
  status.enemy_central_highland_lower_gain_point = (rfid >> 11) & 0x01;
  status.enemy_central_highland_upper_gain_point = (rfid >> 12) & 0x01;
  status.friendly_highway_lower_gain_point = (rfid >> 13) & 0x01;
  status.friendly_highway_upper_gain_point = (rfid >> 14) & 0x01;
  status.enemy_highway_lower_gain_point = (rfid >> 15) & 0x01;
  status.enemy_highway_upper_gain_point = (rfid >> 16) & 0x01;
  status.friendly_fortress_gain_point = (rfid >> 17) & 0x01;
  status.friendly_outpost_gain_point = (rfid >> 18) & 0x01;
  status.friendly_supply_zone_non_exchange = (rfid >> 19) & 0x01;
  status.friendly_supply_zone_exchange = (rfid >> 20) & 0x01;
  status.friendly_big_resource_island = (rfid >> 21) & 0x01;
  status.enemy_big_resource_island = (rfid >> 22) & 0x01;
  status.center_gain_point = (rfid >> 23) & 0x01;
  return status;
}

}  // namespace bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bridge::BridgeNode)