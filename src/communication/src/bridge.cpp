#include "communication/bridge.hpp"
#include <algorithm>
#include <cstring>

namespace bridge
{

BridgeNode::BridgeNode(const rclcpp::NodeOptions & options)
: Node("bridge", options),
  port_name_("/dev/ttyACM0"),
  baud_rate_(115200)
{
  this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->get_parameter("port_name", port_name_);
  this->get_parameter("baud_rate", baud_rate_);

  com_ = std::make_shared<SerialCommunicationClass>(this, port_name_, baud_rate_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  bridge_twist_pc_ = std::make_shared<RosSerialBridge
    <geometry_msgs::msg::Twist>>(
      this, "/cmd_vel", true,
      std::bind(&BridgeNode::encodeTwist, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::sendDataFrame, com_.get(), std::placeholders::_1, std::placeholders::_2),
      nullptr
    );

  bridge_Yaw_mcu_ = std::make_shared<RosSerialBridge
    <std_msgs::msg::Float64>>(
      this, "/serial/Yaw", false,
      nullptr,
      std::bind(&BridgeNode::decodeYaw, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::receiveDataFrame, com_.get())
    );

  bridge_TESspeed_mcu_ = std::make_shared<RosSerialBridge
    <geometry_msgs::msg::Twist>>(
      this, "/serial/TES_speed", false,
      nullptr,
      std::bind(&BridgeNode::decodeTESspeed, this, std::placeholders::_1),
      nullptr,
      std::bind(&SerialCommunicationClass::receiveDataFrame, com_.get())
    );

  gimbal_vision_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(30),
  std::bind(&BridgeNode::publishTransformGimbalVision, this));
}

uint8_t* BridgeNode::encodeTwist(const geometry_msgs::msg::Twist& msg)
{
  float vx = static_cast<float>(msg.linear.x);
  float vy = static_cast<float>(msg.linear.y);
  float wz = static_cast<float>(msg.angular.z); // TBD: change to chassis frame

  float vx_Y = static_cast<float>(msg.linear.x);
  float vy_Y = static_cast<float>(msg.linear.y); // TBD: change to YAW frame

  uint8_t* payload = new uint8_t[26]();

  payload[0] = littleTES; // TBD: get from behavior
  com_->writeFloatLE(&payload[1], angle_init_); // TBD: get from relocalization TF
  payload[5] = true; // TBD: get child_mode from behavior

  com_->writeFloatLE(&payload[6], vx_Y);
  com_->writeFloatLE(&payload[10], vy_Y);
  
  com_->writeFloatLE(&payload[14], vx);
  com_->writeFloatLE(&payload[18], vy);
  com_->writeFloatLE(&payload[22], wz);

  // std::cout << "[ ";
  // for (size_t i = 0; i < 26; i++) {
  //   std::cout << std::setw(2) << std::setfill('0')
  //     << std::hex << std::uppercase
  //     << static_cast<int>(payload[i]) << " ";
  // }
  // std::cout << "]" << std::dec << std::endl;

  // RCLCPP_INFO(this->get_logger(), "Encoded Twist: vx=%.2f, vy=%.2f, wz=%.2f", vx, vy, wz);
  return payload;
}

std_msgs::msg::Float64 BridgeNode::decodeYaw(const uint8_t* payload)
{
  std_msgs::msg::Float64 msg;
  msg.data = static_cast<double>(com_->readFloatLE(&payload[7]));
  return msg;
}

geometry_msgs::msg::Twist BridgeNode::decodeTESspeed(const uint8_t* payload)
{
  geometry_msgs::msg::Twist msg;
  msg.angular.z = static_cast<double>(com_->readFloatLE(&payload[3]));
  return msg;
}

void BridgeNode::publishTransformGimbalVision()
{
  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    transformStamped = tf_buffer_->lookupTransform(
      "odom", "base_footprint",
      tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  tf2::Quaternion q(
    transformStamped.transform.rotation.x,
    transformStamped.transform.rotation.y,
    transformStamped.transform.rotation.z,
    transformStamped.transform.rotation.w
  );

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  geometry_msgs::msg::TransformStamped gimbal_tf;
  gimbal_tf.header.stamp = this->get_clock()->now();
  gimbal_tf.header.frame_id = "base_footprint";
  gimbal_tf.child_frame_id = "gimbal_yaw_vision";

  gimbal_tf.transform.translation.x = 0.0;
  gimbal_tf.transform.translation.y = 0.0;
  gimbal_tf.transform.translation.z = 0.0;

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0, 0, -yaw);

  gimbal_tf.transform.rotation.x = q_yaw.x();
  gimbal_tf.transform.rotation.y = q_yaw.y();
  gimbal_tf.transform.rotation.z = q_yaw.z();
  gimbal_tf.transform.rotation.w = q_yaw.w();
  
  tf_broadcaster_->sendTransform(gimbal_tf);
}

}// namespace bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<bridge::BridgeNode>(options));
  rclcpp::shutdown();
  return 0;
}