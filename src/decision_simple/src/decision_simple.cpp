#include "decision_simple/decision_simple.hpp"

namespace decision_simple
{

DecisionSimple::DecisionSimple(const rclcpp::NodeOptions & options)
: Node("decision_simple", options)
{
    
}

} // namespace decision_simple

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(decision_simple::DecisionSimple)