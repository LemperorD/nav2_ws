#ifndef DECISION_SIMPLE_HPP
#define DECISION_SIMPLE_HPP

#include <rclcpp/rclcpp.hpp>

namespace decision_simple
{

class DecisionSimple : public rclcpp::Node
{
public:
  explicit DecisionSimple(const rclcpp::NodeOptions & options);

private:
};

} // namespace decision_simple

#endif // DECISION_SIMPLE_HPP