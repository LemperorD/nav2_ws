#ifndef ROS_SERIAL_BRIDGE_HPP
#define ROS_SERIAL_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>

template<typename RosMsgT, typename PayloadT>
class RosSerialBridge {
public:
  using EncoderFunc = std::function<PayloadT(const RosMsgT&)>;
  using DecoderFunc = std::function<RosMsgT(const PayloadT&)>;

  RosSerialBridge(
      rclcpp::Node* node,
      const std::string& ros_topic_name,
      bool ros_to_serial,// true: ROS → 电控, 反之为false
      EncoderFunc encoder,
      DecoderFunc decoder,
      std::function<void(const PayloadT&)> serial_sender)
      : node_(node),
        encoder_(encoder),
        decoder_(decoder),
        serial_sender_(serial_sender)
  {
    auto qos = rclcpp::QoS(10);

    if (ros_to_serial) {
      sub_ = node_->create_subscription<RosMsgT>(
        ros_topic_name, qos,
        [this](const typename RosMsgT::SharedPtr msg){
          PayloadT payload = encoder_(*msg);
          serial_sender_(payload);
        });
    }

    else {
      pub_ = node_->create_publisher<RosMsgT>(ros_topic_name, qos);
    }
  }

  // 电控 → ROS，通过这个入口就能完成回传
  void receiveFromSerial(const PayloadT& payload) {
    if (!pub_) return;
    RosMsgT msg = decoder_(payload);
    pub_->publish(msg);
  }

private:
  rclcpp::Node* node_;
  EncoderFunc encoder_;
  DecoderFunc decoder_;
  std::function<void(const PayloadT&)> serial_sender_;
  typename rclcpp::Publisher<RosMsgT>::SharedPtr pub_;
  typename rclcpp::Subscription<RosMsgT>::SharedPtr sub_;
};

#endif // ROS_SERIAL_BRIDGE_HPP
