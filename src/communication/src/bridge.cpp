#include "communication/bridge.hpp"
#include <algorithm>
#include <cstring>

namespace bridge
{

BridgeNode::BridgeNode(const rclcpp::NodeOptions & options)
: Node("bridge", options)
{
  com_ = std::make_shared<SerialCommunicationClass>(this);

  bridge_twist_ = std::make_shared<
    RosSerialBridge<geometry_msgs::msg::Twist, Array25>>(
      this, "/cmd_vel", true,
      std::bind(&BridgeNode::encodeTwist, this, std::placeholders::_1),
      [](const Array25&){ return geometry_msgs::msg::Twist{}; },
      std::bind(&SerialCommunicationClass::sendArray25, com_.get(), std::placeholders::_1)
    );
}

Array25 BridgeNode::encodeTwist(const geometry_msgs::msg::Twist& msg)
{
  float vx = static_cast<float>(msg.linear.x);
  float vy = static_cast<float>(msg.linear.y);
  float wz = static_cast<float>(msg.angular.z);

  Array25 payload{};
  com_->writeFloatLE(&payload[0], vx);
  com_->writeFloatLE(&payload[4], vy);
  com_->writeFloatLE(&payload[8], wz);
  return payload;
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