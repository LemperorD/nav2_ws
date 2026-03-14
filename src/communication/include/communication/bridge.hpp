#pragma once

#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "communication/Com.h"
#include "communication/ros_serial_bridge.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace bridge
{

class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options);
  ~BridgeNode() override;

private:
  void publishRefereeData();

private:
  uint8_t* encodeTwist(const geometry_msgs::msg::Twist& msg);
  std_msgs::msg::Float32 decodeYaw(const uint8_t* payload);
  geometry_msgs::msg::Twist decodeTESspeed(const uint8_t* payload);
  geometry_msgs::msg::Point decodeEnemyPos(const uint8_t* payload);

private:
  double dwa_filter(double sample);
  geometry_msgs::msg::Twist transformVelocityToChassis(
    const geometry_msgs::msg::Twist & twist_in, double yaw_diff);

private:
  void publishTransformGimbalVision();
  void publishTransformGimbalYaw(double yaw);

private:
  std::string port_name_ = "/dev/ttyACM0";
  int baud_rate_ = 115200;

  double Yaw_bias_ = 0.0;
  int max_dwa_size_ = 15;
  double vel_trans_scale_ = 40.0;
  double yaw_diff_ = 0.0;
  double angle_init_ = 0.0;
  uint8_t chassis_mode_ = 0;

  std::deque<double> dwa_;

  std::shared_ptr<SerialCommunicationClass> com_;

  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_twist_pc_;
  std::shared_ptr<RosSerialBridge<std_msgs::msg::Float32>> bridge_Yaw_mcu_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_TESspeed_mcu_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Point>> bridge_EnemyPos_mcu_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;
  rclcpp::TimerBase::SharedPtr gimbal_vision_timer_;

  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr rfid_status_pub_;
  rclcpp::TimerBase::SharedPtr referee_rx_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace bridge