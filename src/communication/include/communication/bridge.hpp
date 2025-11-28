#ifndef BRIDGE_NODE_HPP
#define BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "communication/ros_serial_bridge.hpp"
#include "communication/Com.h"

using Array25 = std::array<uint8_t,25>;

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
  Array25 encodeTwist(const geometry_msgs::msg::Twist& msg);
  std_msgs::msg::Float64 decodeYaw(const Array25& payload);

private:
  std::shared_ptr<SerialCommunicationClass> com_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist, Array25>> bridge_twist_pc_;
  std::shared_ptr<RosSerialBridge<std_msgs::msg::Float64, Array25>> bridge_Yaw_mcu_;
};

}  // namespace bridge

#endif  // BRIDGE_NODE_HPP
