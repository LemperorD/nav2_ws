#pragma once

#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "communication/Com.h"
#include "communication/ros_serial_bridge.hpp"

#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"

using json = nlohmann::json;

namespace bridge
{

typedef enum{
  chassisFollowed = 1,
  littleTES,
  goHome,
} chassisMode;

class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options);
  ~BridgeNode() override;

public: // 检测雷达是否连接
  inline bool isLidarConnected()
  {
    std::ifstream f("/home/ld/nav2_ws/config/mid360_user_config.json");
    if (!f.is_open()) {
      std::cerr << "无法打开配置文件\n"; return false;
    }

    json config;
    try{
      f >> config;
    } catch (...) {
      std::cerr << "JSON 解析失败\n"; return false;
    }

    std::string lidar_ip = config["lidar_configs"][0]["ip"];  // 读取雷达IP
    int point_port = config["MID360"]["lidar_net_info"]["point_data_port"]; // 读取端口（建议检测 point_data_port）
    // std::cout << "Lidar IP: " << lidar_ip << std::endl;
    // std::cout << "Point Port: " << point_port << std::endl;

    // 创建 socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      std::cerr << "socket 创建失败\n"; return false;
    }

    // 设置超时（0.5秒）
    struct timeval timeout;
    timeout.tv_sec = 0; timeout.tv_usec = 500000;

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(point_port);

    if (inet_pton(AF_INET, lidar_ip.c_str(), &addr.sin_addr) <= 0) {
      std::cerr << "IP地址无效\n";
      close(sock); return false;
    }

    int ret = connect(sock, (sockaddr *)&addr, sizeof(addr));

    close(sock);

    return (ret == 0);
  }

private: // 编解码函数
  uint8_t* encodeTwist(const geometry_msgs::msg::Twist& msg);
  std_msgs::msg::Float32 decodeYaw(const uint8_t* payload);
  geometry_msgs::msg::Twist decodeTESspeed(const uint8_t* payload);
  geometry_msgs::msg::Point decodeEnemyPos(const uint8_t* payload);

private: // dwa滤波器
  int max_dwa_size_ = 15;
  std::deque<double> dwa_;
  double dwa_filter(double sample);
  geometry_msgs::msg::Twist transformVelocityToChassis(const geometry_msgs::msg::Twist & twist_in, double yaw_diff);

private:
  void publishTransformGimbalVision();
  void publishTransformGimbalYaw(double yaw);

private:
  std::string port_name_ = "/dev/ttyACM0";
  int baud_rate_ = 115200;
  double Yaw_bias_ = 0.0;
  double vel_trans_scale_ = 40.0;
  double yaw_diff_ = 0.0;
  double angle_init_ = 0.0;

  std::shared_ptr<SerialCommunicationClass> com_;

  // 模板类桥接器
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_twist_pc_;
  std::shared_ptr<RosSerialBridge<std_msgs::msg::Float32>> bridge_Yaw_mcu_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Twist>> bridge_TESspeed_mcu_;
  std::shared_ptr<RosSerialBridge<geometry_msgs::msg::Point>> bridge_EnemyPos_mcu_;

  // timer
  rclcpp::TimerBase::SharedPtr gimbal_vision_timer_;
  rclcpp::TimerBase::SharedPtr referee_rx_timer_;

  // tf相关变量
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 多输入/输出的额外sub/pub可在此添加
  // Additional sub/pub for multi-input/output can be added here
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;

  // 多输入/输出所使用的成员变量可在此添加
  // Member variables used for multi-input/output can be added here
  uint8_t chassis_mode_ = chassisFollowed;

  // 判断是否有雷达连接
  bool lidar_connected_ = false;

private: // 裁判系统相关
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  void publishRefereeData();
  pb_rm_interfaces::msg::RfidStatus rfid2ros(uint32_t rfid);
};

}  // namespace bridge