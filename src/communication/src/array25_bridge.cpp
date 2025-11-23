#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "communication/Com.h"
#include <algorithm>
#include <cstring>

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode() : Node("array25_bridge") {
    // 1. 串口通信类
    com_ = std::make_shared<SerialCommunicationClass>(this);

    // 2. ROS 话题：电控 → ROS
    pub_rx_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/array25_rx", 10);
    pub_cd15_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/cd15_rx", 10);

    // 3. ROS 话题：ROS → 电控（通用 25B 数组）
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

    // 4. 订阅底盘速度，并自动打包成 25B 发送给电控
    //    话题名字做成参数，默认用 fake_vel_transform 输出的 /cmd_vel
    this->declare_parameter<std::string>("chassis_cmd_vel_topic", "/cmd_vel");
    std::string chassis_topic;
    this->get_parameter("chassis_cmd_vel_topic", chassis_topic);

    sub_chassis_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      chassis_topic, 10,
      std::bind(&BridgeNode::chassisVelCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "array25_bridge: subscribe chassis cmd_vel from '%s'", chassis_topic.c_str());

    // 5. 收到 25B 的桥接（CMD=0xE1，LEN=25）
    com_->setArray25Callback([this](const std::array<uint8_t,25>& rx){
      std_msgs::msg::UInt8MultiArray out;
      out.data.assign(rx.begin(), rx.end());
      pub_rx_->publish(out);
    });

    // 6. 抓取 0xCD/15 的原始负载并发布
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
  // 将 float 小端写进 4 个字节
  static void writeFloatLE(uint8_t *dst, float value)
  {
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "float must be 4 bytes");
    std::memcpy(&bits, &value, sizeof(float));
    dst[0] = static_cast<uint8_t>(bits & 0xFFu);
    dst[1] = static_cast<uint8_t>((bits >> 8) & 0xFFu);
    dst[2] = static_cast<uint8_t>((bits >> 16) & 0xFFu);
    dst[3] = static_cast<uint8_t>((bits >> 24) & 0xFFu);
  }

  // 新增：底盘速度回调，打包 25B 并发送给电控
  void chassisVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 这里直接用 vx, vy, wz（Twist 中单位一般是 m/s, rad/s）
    float vx = static_cast<float>(msg->linear.x);
    float vy = static_cast<float>(msg->linear.y);
    float wz = static_cast<float>(msg->angular.z);

    std::array<uint8_t, 25> payload{};
    // 当前约定：前 3 个 float32（小端）分别放 vx, vy, wz
    // [0..3]  : vx
    // [4..7]  : vy
    // [8..11] : wz
    // [12..24]: 预留，暂时填 0，将来可以扩展模式位 / 标志位
    writeFloatLE(&payload[0],  vx);
    writeFloatLE(&payload[4],  vy);
    writeFloatLE(&payload[8],  wz);

    // TODO：和电控已经约定好了 25B 里的具体字段位置，
    //       就把上面的打包逻辑改成对应的缩放 / 整型 / bit 。

    if (!com_->sendArray25(payload)) {
      RCLCPP_WARN(get_logger(), "Failed to send chassis velocity frame over serial");
    }
  }

private:
  std::shared_ptr<SerialCommunicationClass> com_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_rx_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_cd15_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_tx_;

  // 新增：底盘速度订阅
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_chassis_vel_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
