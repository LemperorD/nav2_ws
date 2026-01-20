#include "rclcpp/rclcpp.hpp"
#include "decision_simple/decision_simple.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<decision_simple::DecisionSimple>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
