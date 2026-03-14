#include "communication/bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

uint16_t readU16LE(const uint8_t *src)
{
  return static_cast<uint16_t>(src[0]) |
         (static_cast<uint16_t>(src[1]) << 8);
}

uint32_t readU32LE(const uint8_t *src)
{
  return static_cast<uint32_t>(src[0]) |
         (static_cast<uint32_t>(src[1]) << 8) |
         (static_cast<uint32_t>(src[2]) << 16) |
         (static_cast<uint32_t>(src[3]) << 24);
}
}  // namespace

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

  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>(
      "referee/robot_status", 10);
  game_status_pub_ =
    this->create_publisher<std_msgs::msg::UInt8>(
      "referee/game_status", 10);
  rfid_status_pub_ =
    this->create_publisher<std_msgs::msg::UInt32>(
      "referee/rfid_status", 10);

  referee_rx_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&BridgeNode::publishRefereeData, this));

  bridge_twist_pc_ =
    std::make_shared<RosSerialBridge<geometry_msgs::msg::Twist>>(
      this, "/cmd_vel", true,
      std::bind(&BridgeNode::encodeTwist, this, std::placeholders::_1),
      nullptr,
      std::bind(
        &SerialCommunicationClass::sendDataFrame,
        com_.get(),
        std::placeholders::_1,
        std::placeholders::_2),
      nullptr,
      nullptr);

  bridge_Yaw_mcu_ =
    std::make_shared<RosSerialBridge<std_msgs::msg::Float32>>(
      this, "/serial/Yaw", false,
      nullptr,
      std::bind(&BridgeNode::decodeYaw, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::receiveDataFrame, com_.get()),
      nullptr);

  bridge_TESspeed_mcu_ =
    std::make_shared<RosSerialBridge<geometry_msgs::msg::Twist>>(
      this, "/serial/TES_speed", false,
      nullptr,
      std::bind(&BridgeNode::decodeTESspeed, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::receiveDataFrame, com_.get()),
      nullptr);

  bridge_EnemyPos_mcu_ =
    std::make_shared<RosSerialBridge<geometry_msgs::msg::Point>>(
      this, "/serial/EnemyPos", false,
      nullptr,
      std::bind(&BridgeNode::decodeEnemyPos, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::receiveDataFrame, com_.get()),
      nullptr);

  gimbal_vision_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&BridgeNode::publishTransformGimbalVision, this));

  chassis_mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "chassis_mode", 10,
    [this](const std_msgs::msg::UInt8::SharedPtr msg) {
      chassis_mode_ = static_cast<uint8_t>(msg->data);
    });
}

BridgeNode::~BridgeNode()
{
  RCLCPP_INFO(this->get_logger(), "BridgeNode shutting down...");

  if (gimbal_vision_timer_) {
    gimbal_vision_timer_.reset();
  }
  if (referee_rx_timer_) {
    referee_rx_timer_.reset();
  }

  chassis_mode_sub_.reset();

  bridge_twist_pc_.reset();
  bridge_Yaw_mcu_.reset();
  bridge_TESspeed_mcu_.reset();
  bridge_EnemyPos_mcu_.reset();

  robot_status_pub_.reset();
  game_status_pub_.reset();
  rfid_status_pub_.reset();

  com_.reset();

  RCLCPP_INFO(this->get_logger(), "BridgeNode shutdown complete.");
}

uint8_t* BridgeNode::encodeTwist(const geometry_msgs::msg::Twist& msg)
{
  geometry_msgs::msg::Twist twist_chassis =
    transformVelocityToChassis(msg, yaw_diff_ * kPi / 180.0);

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
  const float yaw = com_->readFloatLE(&payload[7]);
  yaw_diff_ = static_cast<double>(yaw);
  msg.data = yaw;
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
  int16_t x = 0;
  int16_t y = 0;
  std::memcpy(&x, payload + 11, sizeof(int16_t));
  std::memcpy(&y, payload + 13, sizeof(int16_t));
  msg.x = static_cast<double>(x);
  msg.y = static_cast<double>(y);
  return msg;
}

void BridgeNode::publishTransformGimbalVision()
{
  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    transformStamped = tf_buffer_->lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
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

double BridgeNode::dwa_filter(double sample)
{
  dwa_.push_back(sample);
  if (static_cast<int>(dwa_.size()) > max_dwa_size_) {
    dwa_.pop_front();
  }

  double sum = 0.0;
  for (double x : dwa_) {
    sum += x;
  }
  return dwa_.empty() ? sample : (sum / static_cast<double>(dwa_.size()));
}

geometry_msgs::msg::Twist BridgeNode::transformVelocityToChassis(
  const geometry_msgs::msg::Twist & twist_in, double yaw_diff)
{
  geometry_msgs::msg::Twist out;
  out.linear.x =
    twist_in.linear.x * std::cos(yaw_diff) -
    twist_in.linear.y * std::sin(yaw_diff);
  out.linear.y =
    twist_in.linear.x * std::sin(yaw_diff) +
    twist_in.linear.y * std::cos(yaw_diff);
  out.angular.z = twist_in.angular.z;
  return out;
}

void BridgeNode::publishRefereeData()
{
  if (!com_->hasNewRefereeFrame()) {
    return;
  }

  const uint8_t *payload = com_->receiveRefereeFrame();
  if (payload == nullptr) {
    return;
  }

  const uint8_t game_byte = payload[0];
  const uint8_t game_progress = (game_byte >> 4) & 0x0F;

  const uint16_t current_hp = readU16LE(payload + 1);
  const uint16_t heat1 = readU16LE(payload + 3);
  const uint16_t heat2 = readU16LE(payload + 5);
  const uint16_t ammo17 = readU16LE(payload + 7);
  const uint32_t rfid = readU32LE(payload + 9);

  (void)heat1;
  (void)heat2;

  pb_rm_interfaces::msg::RobotStatus rs;
  rs.current_hp = current_hp;
  rs.projectile_allowance_17mm = ammo17;
  robot_status_pub_->publish(rs);

  std_msgs::msg::UInt8 gs;
  gs.data = game_progress;
  game_status_pub_->publish(gs);

  std_msgs::msg::UInt32 rf;
  rf.data = rfid;
  rfid_status_pub_->publish(rf);

  com_->clearRefereeFrameFlag();
}

}  // namespace bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bridge::BridgeNode)