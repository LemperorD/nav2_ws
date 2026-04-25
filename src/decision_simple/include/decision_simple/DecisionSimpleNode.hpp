#include "decision_simple.hpp"
#include "rclcpp/rclcpp.hpp"

namespace decision_simple {

  class DecisionSimpleNode : public rclcpp::Node {
  public:
    explicit DecisionSimpleNode(const rclcpp::NodeOptions& options);

  private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr chassis_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        debug_attack_pose_pub_;

    rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr
        robot_status_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr
        game_status_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr
        armors_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr
        target_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  };
}  // namespace decision_simple