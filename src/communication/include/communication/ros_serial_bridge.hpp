#ifndef ROS_SERIAL_BRIDGE_HPP
#define ROS_SERIAL_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <atomic>

template<typename RosMsgT>
class RosSerialBridge {
public:
  using EncoderFunc = std::function<const uint8_t*(const RosMsgT&)>;
  using DecoderFunc = std::function<RosMsgT(const uint8_t*)>;

  RosSerialBridge(
      rclcpp::Node* node,
      const std::string& ros_topic_name,
      bool ros_to_serial,// true: ROS → MCU, 反之为false
      EncoderFunc encoder,
      DecoderFunc decoder,
      std::function<void(const uint8_t*, size_t)> serial_sender,
      std::function<const uint8_t*(void)> serial_receiver,
      const uint8_t* payload)
      : node_(node),
        encoder_(encoder),
        decoder_(decoder),
        serial_sender_(serial_sender),
        serial_receiver_(serial_receiver),
        payload_(payload)
  {
    auto qos = rclcpp::QoS(10);

    if (ros_to_serial) {
      sub_ = node_->create_subscription<RosMsgT>(
        ros_topic_name, qos,
        [this](const typename RosMsgT::SharedPtr msg){
          payload_ = encoder_(*msg);
          serial_sender_(payload_, 26);
        });
    }
    else {
      pub_ = node_->create_publisher<RosMsgT>(ros_topic_name, qos);
      recv_thread_ = std::thread([this]() {
        rclcpp::Rate r(200); // 200Hz = 5ms
        while (rclcpp::ok()) {
          payload_ = serial_receiver_();
          if (!payload_) continue;
          RosMsgT msg = decoder_(payload_);
          pub_->publish(msg);
          r.sleep();
        }
      });
    }
  }

  ~RosSerialBridge()
  {
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
    if (payload_) {
      payload_ = nullptr;
    }
  }

private:
  rclcpp::Node* node_;
  EncoderFunc encoder_;
  DecoderFunc decoder_;
  std::function<void(const uint8_t*, size_t)> serial_sender_;
  std::function<const uint8_t*(void)> serial_receiver_;
  typename rclcpp::Publisher<RosMsgT>::SharedPtr pub_;
  std::thread recv_thread_;
  typename rclcpp::Subscription<RosMsgT>::SharedPtr sub_;
  const uint8_t* payload_ = nullptr;
};

#endif // ROS_SERIAL_BRIDGE_HPP
