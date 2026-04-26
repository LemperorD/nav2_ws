#include <stdexcept>

#include "decision_simple/adapter/transform.hpp"

namespace decision_simple {

  Quaternion ConvertQuaternion(const geometry_msgs::msg::Quaternion& ros_quat) {
    Quaternion quat;
    quat.x = ros_quat.x;
    quat.y = ros_quat.y;
    quat.z = ros_quat.z;
    quat.w = ros_quat.w;
    return quat;
  }

  Pose3D ConvertPose(const geometry_msgs::msg::Pose& ros_pose) {
    Pose3D pose;
    pose.pos_x = ros_pose.position.x;
    pose.pos_y = ros_pose.position.y;
    pose.pos_z = ros_pose.position.z;
    pose.orientation = ConvertQuaternion(ros_pose.orientation);
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

  GameStatus ConvertGameStatus(
      const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) {
    GameStatus status;
    if (!msg) {
      throw std::invalid_argument(
          "ConvertGameStatus received nullptr GameStatus message");
    }
    status.game_progress = msg->game_progress;
    status.stage_remain_time = msg->stage_remain_time;
    return status;
  }

  Armors ConvertArmors(const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    Armors armors;
    if (!msg) {
      throw std::invalid_argument(
          "ConvertArmors received nullptr Armors message");
    }

    armors.frame_id = msg->header.frame_id;
    for (const auto& ros_armor : msg->armors) {
      Armor armor;
      armor.pose = ConvertPose(ros_armor.pose);
      // armor_id will be set to 0 by default
      armors.items.push_back(armor);
    }

    return armors;
  }

  Target ConvertTarget(const auto_aim_interfaces::msg::Target::SharedPtr msg) {
    Target target;
    if (!msg) {
      throw std::invalid_argument(
          "ConvertTarget received nullptr Target message");
    }

    target.frame_id = msg->header.frame_id;
    target.position.pos_x = msg->position.x;
    target.position.pos_y = msg->position.y;
    target.position.pos_z = msg->position.z;
    target.yaw = msg->yaw;
    target.tracking = msg->tracking;

    return target;
  }

}  // namespace decision_simple
