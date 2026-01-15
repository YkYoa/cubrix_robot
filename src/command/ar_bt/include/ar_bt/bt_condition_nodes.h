#ifndef AR_BT__BT_CONDITION_NODES_H_
#define AR_BT__BT_CONDITION_NODES_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <ar_planning_interface/ar_planning_interface.h>
#include <vector>
#include <string>
#include <cmath>

namespace ar_bt
{

/**
 * @brief Base class for AR BT condition nodes with planning interface access
 */
class ARBTConditionNode : public BT::ConditionNode
{
public:
  ARBTConditionNode(const std::string& name, const BT::NodeConfiguration& config,
                    rclcpp::Node::SharedPtr node,
                    std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
    : BT::ConditionNode(name, config)
    , node_(node)
    , planning_interface_(planning_interface)
  {}

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
};

/**
 * @brief Check if robot is at target joint configuration (within tolerance)
 * Input Ports:
 *   - target_joints: vector<double> - Target joint values
 *   - tolerance: double - Position tolerance in radians (default: 0.01)
 * Returns SUCCESS if at position, FAILURE otherwise
 */
class IsAtJoint : public ARBTConditionNode
{
public:
  IsAtJoint(const std::string& name, const BT::NodeConfiguration& config,
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
    : ARBTConditionNode(name, config, node, planning_interface)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<double>>("target_joints", "Target joint values"),
      BT::InputPort<double>("tolerance", 0.01, "Position tolerance in radians")
    };
  }

  BT::NodeStatus tick() override
  {
    auto target = getInput<std::vector<double>>("target_joints");
    if (!target) {
      RCLCPP_ERROR(node_->get_logger(), "IsAtJoint: missing 'target_joints' input");
      return BT::NodeStatus::FAILURE;
    }

    double tolerance = getInput<double>("tolerance").value_or(0.01);
    
    // Get current joint positions
    auto current_joints = planning_interface_->getCurrentJoints();
    
    if (current_joints.size() != target.value().size()) {
      RCLCPP_ERROR(node_->get_logger(), "IsAtJoint: joint count mismatch");
      return BT::NodeStatus::FAILURE;
    }

    // Check if all joints are within tolerance
    for (size_t i = 0; i < current_joints.size(); ++i) {
      if (std::abs(current_joints[i] - target.value()[i]) > tolerance) {
        return BT::NodeStatus::FAILURE;
      }
    }

    RCLCPP_DEBUG(node_->get_logger(), "IsAtJoint: Robot is at target position");
    return BT::NodeStatus::SUCCESS;
  }
};

/**
 * @brief Check if a valid plan exists on the blackboard
 * Input Ports:
 *   - plan_key: string - Blackboard key for the plan (default: "current_plan")
 * Returns SUCCESS if plan exists and is valid, FAILURE otherwise
 */
class IsPlanValid : public BT::ConditionNode
{
public:
  IsPlanValid(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("plan_key", "current_plan", "Blackboard key for the plan")
    };
  }

  BT::NodeStatus tick() override
  {
    auto plan_key = getInput<std::string>("plan_key").value_or("current_plan");
    
    // Check if plan exists on blackboard
    auto blackboard = config().blackboard;
    if (!blackboard) {
      return BT::NodeStatus::FAILURE;
    }

    // Try to get the plan from blackboard using getAny
    try {
      auto any_ref = blackboard->getAny(plan_key);
      if (!any_ref) {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }
  }
};

/**
 * @brief Check if robot is ready (not in error state, operational)
 * Returns SUCCESS if robot is ready, FAILURE otherwise
 */
class IsRobotReady : public ARBTConditionNode
{
public:
  IsRobotReady(const std::string& name, const BT::NodeConfiguration& config,
               rclcpp::Node::SharedPtr node,
               std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
    : ARBTConditionNode(name, config, node, planning_interface)
  {}

  static BT::PortsList providedPorts()
  {
    return {};  // No input ports needed
  }

  BT::NodeStatus tick() override
  {
    // Check if MoveGroup is available and operational
    try {
      // Try to get current state - if this works, robot is ready
      auto current_joints = planning_interface_->getCurrentJoints();
      if (current_joints.empty()) {
        RCLCPP_WARN(node_->get_logger(), "IsRobotReady: Could not get joint values");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "IsRobotReady: Exception - %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
};

// NOTE: AlwaysSuccess and AlwaysFailure are built-in to BehaviorTree.CPP v3
// No need to define them here - use the built-in versions

/**
 * @brief Register all AR BT condition nodes with the factory
 * @param factory BehaviorTree factory to register nodes with
 * @param node ROS2 node for logging
 * @param planning_interface Planning interface for state queries
 */
void registerARBTConditionNodes(BT::BehaviorTreeFactory& factory,
                                 rclcpp::Node::SharedPtr node,
                                 std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

} // namespace ar_bt

#endif // AR_BT__BT_CONDITION_NODES_H_
