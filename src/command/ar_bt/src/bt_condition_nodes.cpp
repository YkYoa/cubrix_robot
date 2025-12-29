#include "ar_bt/bt_condition_nodes.h"

namespace ar_bt
{

void registerARBTConditionNodes(BT::BehaviorTreeFactory& factory,
                                 rclcpp::Node::SharedPtr node,
                                 std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
{
  // IsAtJoint - requires node and planning_interface
  BT::NodeBuilder is_at_joint_builder = [node, planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<IsAtJoint>(name, config, node, planning_interface);
  };

  // IsPlanValid - standalone, only needs blackboard
  BT::NodeBuilder is_plan_valid_builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<IsPlanValid>(name, config);
  };

  // IsRobotReady - requires node and planning_interface
  BT::NodeBuilder is_robot_ready_builder = [node, planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<IsRobotReady>(name, config, node, planning_interface);
  };

  // NOTE: AlwaysSuccess and AlwaysFailure are built-in to BehaviorTree.CPP v3
  // No need to register them - they're available by default

  // Register all condition nodes
  factory.registerBuilder<IsAtJoint>("IsAtJoint", is_at_joint_builder);
  factory.registerBuilder<IsPlanValid>("IsPlanValid", is_plan_valid_builder);
  factory.registerBuilder<IsRobotReady>("IsRobotReady", is_robot_ready_builder);

  RCLCPP_INFO(node->get_logger(), "Registered AR BT condition nodes: IsAtJoint, IsPlanValid, IsRobotReady");
}

} // namespace ar_bt
