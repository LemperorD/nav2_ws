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
}

uint8_t* BridgeNode::encodeTwist(const geometry_msgs::msg::Twist& msg)
{
  float vx = static_cast<float>(msg.linear.x);
  float vy = static_cast<float>(msg.linear.y);
  float wz = static_cast<float>(msg.angular.z);

  uint8_t* payload{};
  com_->writeFloatLE(&payload[0], vx);
  com_->writeFloatLE(&payload[4], vy);
  com_->writeFloatLE(&payload[8], wz);
  return payload;
}

std_msgs::msg::Float64 BridgeNode::decodeYaw(const uint8_t* payload)
{
  std_msgs::msg::Float64 msg;
  msg.data = static_cast<double>(com_->readFloatLE(&payload[0]));
  return msg;
}

}  // namespace bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<bridge::BridgeNode>(options));
  rclcpp::shutdown();
  return 0;
}