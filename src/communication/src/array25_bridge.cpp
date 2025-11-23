#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "communication/Com.h"

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode() : Node("array25_bridge") {
    com_ = std::make_shared<SerialCommunicationClass>(this);
    pub_rx_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/array25_rx", 10);
    pub_cd15_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/cd15_rx", 10);

    sub_tx_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/array25_tx", 10,
      [this](std_msgs::msg::UInt8MultiArray::SharedPtr msg){
        if (msg->data.size() != 25) {
          RCLCPP_WARN(get_logger(), "TX size=%zu, expected 25", msg->data.size());
          return;
        }
        std::array<uint8_t,25> payload{};
        std::copy_n(msg->data.begin(), 25, payload.begin());
        com_->sendArray25(payload);
      });

    // 收到 25B 的桥接
    com_->setArray25Callback([this](const std::array<uint8_t,25>& rx){
      std_msgs::msg::UInt8MultiArray out;
      out.data.assign(rx.begin(), rx.end());
      pub_rx_->publish(out);
    });

    // 抓取 0xCD/15 的原始负载并发布
    com_->setRawFrameCallback([this](uint8_t cmd, const uint8_t* pl, uint8_t len){
      if (cmd == 0xCD && len == 15) {
        std_msgs::msg::UInt8MultiArray out;
        out.data.resize(15);
        std::copy(pl, pl + 15, out.data.begin());
        pub_cd15_->publish(out);
      }
    });
  }
private:
  std::shared_ptr<SerialCommunicationClass> com_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_rx_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_cd15_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_tx_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}