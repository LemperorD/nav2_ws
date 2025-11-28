#ifndef ROS_SERIAL_BRIDGE_HPP
#define ROS_SERIAL_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>

template<typename RosMsgT>
class RosSerialBridge {
public:
  using EncoderFunc = std::function<const uint8_t*(const RosMsgT&)>;
  using DecoderFunc = std::function<RosMsgT(const uint8_t*)>;

  RosSerialBridge(
      rclcpp::Node* node,
      const std::string& ros_topic_name,
      bool ros_to_serial,// true: ROS → 电控, 反之为false
      EncoderFunc encoder,
      DecoderFunc decoder,
      std::function<void(const uint8_t*, size_t)> serial_sender,
      std::function<const uint8_t*(void)> serial_receiver)
      : node_(node),
        encoder_(encoder),
        decoder_(decoder),
        serial_sender_(serial_sender),
        serial_receiver_(serial_receiver)
  {
    auto qos = rclcpp::QoS(10);

    if (ros_to_serial) {
      sub_ = node_->create_subscription<RosMsgT>(
        ros_topic_name, qos,
        [this](const typename RosMsgT::SharedPtr msg){
          const uint8_t* payload = encoder_(*msg);
          serial_sender_(payload, sizeof(payload));
        });
    }

    else {
      pub_ = node_->create_publisher<RosMsgT>(ros_topic_name, qos);
      if (!pub_) return;
      RosMsgT msg = decoder_(serial_receiver_());
      pub_->publish(msg);
    }
  }

private:
  rclcpp::Node* node_;
  EncoderFunc encoder_;
  DecoderFunc decoder_;
  std::function<void(const uint8_t*, size_t)> serial_sender_;
  std::function<const uint8_t*(void)> serial_receiver_;
  typename rclcpp::Publisher<RosMsgT>::SharedPtr pub_;
  typename rclcpp::Subscription<RosMsgT>::SharedPtr sub_;
};

#endif // ROS_SERIAL_BRIDGE_HPP
