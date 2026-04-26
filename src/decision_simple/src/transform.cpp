#include "decision_simple/adapter/transform.hpp"

#include <stdexcept>

namespace decision_simple {

  Quaternion ConvertQuaternion(const geometry_msgs::msg::Quaternion& ros_quat) {
    Quaternion quat;
    quat.x = ros_quat.x;
    quat.y = ros_quat.y;
    quat.z = ros_quat.z;
    quat.w = ros_quat.w;
    return quat;
  }

  Position ConvertPoint(const geometry_msgs::msg::Point& ros_point) {
    Position position;
    position.x = ros_point.x;
    position.y = ros_point.y;
    position.z = ros_point.z;
    return position;
  }

  Pose3D ConvertPose(const geometry_msgs::msg::Pose& ros_pose) {
    Pose3D pose;
    ConvertPoint(ros_pose.position);
    ConvertQuaternion(ros_pose.orientation);
    return pose;
  }

  RobotStatus ConvertRobotStatus(
      pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) {
    RobotStatus theRobotStatus;
    if (!msg) {
      throw std::invalid_argument(
          "ConvertRobotStatus received nullptr RobotStatus message");
    }

    theRobotStatus.robot_id = msg->robot_id;
    theRobotStatus.robot_level = msg->robot_level;
    theRobotStatus.current_hp = msg->current_hp;
    theRobotStatus.maximum_hp = msg->maximum_hp;
    theRobotStatus.shooter_barrel_cooling_value =
        msg->shooter_barrel_cooling_value;
    theRobotStatus.shooter_barrel_heat_limit = msg->shooter_barrel_heat_limit;
    theRobotStatus.shooter_17mm_1_barrel_heat = msg->shooter_17mm_1_barrel_heat;
    theRobotStatus.robot_pos = ConvertPose(msg->robot_pos);
    theRobotStatus.armor_id = msg->armor_id;
    theRobotStatus.hp_deduction_reason = msg->hp_deduction_reason;
    theRobotStatus.projectile_allowance_17mm = msg->projectile_allowance_17mm;
    theRobotStatus.remaining_gold_coin = msg->remaining_gold_coin;
    theRobotStatus.is_hp_deduced = msg->is_hp_deduced;

    return theRobotStatus;
  }

  Armors ConvertArmors(
      const auto_aim_interfaces::msg::Armors::SharedPtr& ros_armorsmsg) {
    Armors theArmors;

    if (!ros_armorsmsg) {
      throw std::invalid_argument(
          "ConvertRobotStatus received nullptr Armors message");
    }
    theArmors.header.stamp.nanosec = ros_armorsmsg->header.stamp.nanosec;
    theArmors.header.stamp.sec = ros_armorsmsg->header.stamp.sec;
    theArmors.header.frame_id = ros_armorsmsg->header.frame_id;

    // 调整目标容器大小
    theArmors.armors.resize(ros_armorsmsg->armors.size());

    // 使用 transform 进行批量转换映射
    std::transform(ros_armorsmsg->armors.begin(), ros_armorsmsg->armors.end(),
                   theArmors.armors.begin(), [](const auto& ros_armor) {
                     return ConvertArmor(ros_armor);  // 调用单体转换逻辑
                   });

    return theArmors;
  }

  Armor ConvertArmor(const auto_aim_interfaces::msg::Armor& ros_armormsg) {
    Armor theArmor;
    theArmor.pose = ConvertPose(ros_armormsg.pose);
    return theArmor;
  }
}  // namespace decision_simple
