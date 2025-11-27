#ifndef BRIDGE_NODE_HPP
#define BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "communication/ros_serial_bridge.hpp"
#include "communication/Com.h"

using Array25 = std::array<uint8_t,25>;

namespace bridge
{

class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options);

private:
  Array25 encodeTwist(const geometry_msgs::msg::Twist& msg);

private:
  std::shared_ptr<SerialCommunicationClass> com_;
  std::shared_ptr<
    RosSerialBridge<geometry_msgs::msg::Twist, Array25>> bridge_twist_;
};

}  // namespace bridge

#endif  // BRIDGE_NODE_HPP
