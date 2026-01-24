#ifndef BRIDGE_NODE_HPP
#define BRIDGE_NODE_HPP

#include <cmath>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
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
} chassisMode;

class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options);
  ~BridgeNode() override;

public: // 大Yaw电机编码值解码
  inline double encoderToRad(float encoder)
  {
    constexpr double TWO_PI = 2.0 * M_PI;
    float out = (static_cast<double>(encoder) / 8192.0) * TWO_PI;
    if (out > TWO_PI)
      out -= TWO_PI;
    else if (out < 0.0)
      out += TWO_PI;
    return out;
  }

private: // 编解码函数
  uint8_t* encodeTwist(const geometry_msgs::msg::Twist& msg);
  std_msgs::msg::Float64 decodeYaw(const uint8_t* payload);
  geometry_msgs::msg::Twist decodeTESspeed(const uint8_t* payload);

private: // create a frame for vision
  rclcpp::TimerBase::SharedPtr gimbal_vision_timer_;
  void publishTransformGimbalVision();

private: // generate TF from gimbal_yaw_odom to gimbal_yaw 
  void publishTransformGimbalYaw(double Yaw);

private: // 滑动窗口滤波,窗口数组成员变量,窗口长度成员变量
  inline double dwa_filter(double sample);
  std::deque<double> dwa_;
  int max_dwa_size_ = 15;

private:
  // 参数
  std::string port_name_;
  int baud_rate_;
  double Yaw_bias_ = 0.0;

  float angle_init_ = 0.0f;
  bool angle_calibrated_ = false;
  std::shared_ptr<SerialCommunicationClass> com_;

  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_twist_pc_;
  std::shared_ptr<RosSerialBridge<std_msgs::msg::Float64>> bridge_Yaw_mcu_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_TESspeed_mcu_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 多输入/输出的额外sub/pub可在此添加
  // Additional sub/pub for multi-input/output can be added here
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;

  // 多输入/输出所使用的成员变量可在此添加
  // Member variables used for multi-input/output can be added here
  uint8_t chassis_mode_;
};

}  // namespace bridge

#endif  // BRIDGE_NODE_HPP
