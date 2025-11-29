#ifndef BRIDGE_NODE_HPP
#define BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "communication/ros_serial_bridge.hpp"
#include "communication/Com.h"

namespace bridge
{

typedef enum
{
  chassisFollowed = 1,
  littleTES,
  goHome,
} fightState;

class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options);

private:
  uint8_t* encodeTwist(const geometry_msgs::msg::Twist& msg);
  std_msgs::msg::Float64 decodeYaw(const uint8_t* payload);
  geometry_msgs::msg::Twist decodeTESspeed(const uint8_t* payload);

private:
  std::string port_name_;
  int baud_rate_;
  float angle_init_ = 0.0f;
  bool angle_calibrated_ = false;
  std::shared_ptr<SerialCommunicationClass> com_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_twist_pc_;
  std::shared_ptr<RosSerialBridge<std_msgs::msg::Float64>> bridge_Yaw_mcu_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_TESspeed_mcu_;
};

}  // namespace bridge

#endif  // BRIDGE_NODE_HPP
