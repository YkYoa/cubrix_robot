#include "ar_bt/bt_action_nodes.h"

namespace ar_bt
{

// === ARBTActionNode Base === 

ARBTActionNode::ARBTActionNode(const std::string& name, const BT::NodeConfiguration& config,
                               rclcpp::Node::SharedPtr node,
                               std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: BT::StatefulActionNode(name, config)
, node_(node)
, planning_interface_(planning_interface)
{
}

// === PlanToJoint ===

PlanToJoint::PlanToJoint(const std::string& name, const BT::NodeConfiguration& config,
                         rclcpp::Node::SharedPtr node,
                         std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: ARBTActionNode(name, config, node, planning_interface)
, planning_done_(false)
{
}

BT::PortsList PlanToJoint::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("target_joints", "Target joint values"),
    BT::OutputPort<bool>("success", "Planning succeeded")
  };
}

BT::NodeStatus PlanToJoint::onStart()
{
  auto target = getInput<std::vector<double>>("target_joints");
  if (!target) {
    RCLCPP_ERROR(node_->get_logger(), "PlanToJoint: missing 'target_joints' input");
    return BT::NodeStatus::FAILURE;
  }

  planning_done_ = false;
  planning_interface_->setTargetJoints(target.value());
  
  if (planning_interface_->plan(current_plan_)) {
    planning_done_ = true;
    setOutput("success", true);
    planning_interface_->visualizeTrajectory(current_plan_);
    RCLCPP_INFO(node_->get_logger(), "PlanToJoint: Planning succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    setOutput("success", false);
    RCLCPP_ERROR(node_->get_logger(), "PlanToJoint: Planning failed");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus PlanToJoint::onRunning()
{
  return planning_done_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void PlanToJoint::onHalted()
{
  planning_done_ = false;
  RCLCPP_INFO(node_->get_logger(), "PlanToJoint: Halted");
}

// === PlanToPose ===

PlanToPose::PlanToPose(const std::string& name, const BT::NodeConfiguration& config,
                       rclcpp::Node::SharedPtr node,
                       std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: ARBTActionNode(name, config, node, planning_interface)
, planning_done_(false)
{
}

BT::PortsList PlanToPose::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Target pose"),
    BT::OutputPort<bool>("success", "Planning succeeded")
  };
}

BT::NodeStatus PlanToPose::onStart()
{
  auto target = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
  if (!target) {
    RCLCPP_ERROR(node_->get_logger(), "PlanToPose: missing 'target_pose' input");
    return BT::NodeStatus::FAILURE;
  }

  planning_done_ = false;
  planning_interface_->setTargetPose(target.value());
  
  if (planning_interface_->plan(current_plan_)) {
    planning_done_ = true;
    setOutput("success", true);
    planning_interface_->visualizeTrajectory(current_plan_);
    RCLCPP_INFO(node_->get_logger(), "PlanToPose: Planning succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    setOutput("success", false);
    RCLCPP_ERROR(node_->get_logger(), "PlanToPose: Planning failed");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus PlanToPose::onRunning()
{
  return planning_done_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void PlanToPose::onHalted()
{
  planning_done_ = false;
  RCLCPP_INFO(node_->get_logger(), "PlanToPose: Halted");
}

// === MoveToJoint ===

MoveToJoint::MoveToJoint(const std::string& name, const BT::NodeConfiguration& config,
                         rclcpp::Node::SharedPtr node,
                         std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: ARBTActionNode(name, config, node, planning_interface)
, motion_done_(false)
{
}

BT::PortsList MoveToJoint::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("target_joints", "Target joint values")
  };
}

BT::NodeStatus MoveToJoint::onStart()
{
  auto target = getInput<std::vector<double>>("target_joints");
  if (!target) {
    RCLCPP_ERROR(node_->get_logger(), "MoveToJoint: missing 'target_joints' input");
    return BT::NodeStatus::FAILURE;
  }

  motion_done_ = false;
  planning_interface_->setTargetJoints(target.value());
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (planning_interface_->plan(plan)) {
    planning_interface_->visualizeTrajectory(plan);
    
    if (planning_interface_->execute(plan)) {
      motion_done_ = true;
      RCLCPP_INFO(node_->get_logger(), "MoveToJoint: Motion succeeded");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "MoveToJoint: Execution failed");
      return BT::NodeStatus::FAILURE;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "MoveToJoint: Planning failed");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveToJoint::onRunning()
{
  return motion_done_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void MoveToJoint::onHalted()
{
  motion_done_ = false;
  RCLCPP_INFO(node_->get_logger(), "MoveToJoint: Halted");
}

// === SetPlanner ===

SetPlanner::SetPlanner(const std::string& name, const BT::NodeConfiguration& config,
                       std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: BT::SyncActionNode(name, config)
, planning_interface_(planning_interface)
{
}

BT::PortsList SetPlanner::providedPorts()
{
  return {
    BT::InputPort<std::string>("pipeline", "ompl", "Planning pipeline (ompl/pilz)"),
    BT::InputPort<std::string>("planner", "RRTConnect", "Planner ID")
  };
}

BT::NodeStatus SetPlanner::tick()
{
  auto pipeline = getInput<std::string>("pipeline").value();
  auto planner = getInput<std::string>("planner").value();

  planning_interface_->setPlanningPipelineId(pipeline);
  planning_interface_->setPlannerId(planner);
  
  return BT::NodeStatus::SUCCESS;
}

// === SetBlendRadius ===

SetBlendRadius::SetBlendRadius(const std::string& name, const BT::NodeConfiguration& config,
                               std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: BT::SyncActionNode(name, config)
, planning_interface_(planning_interface)
{
}

BT::PortsList SetBlendRadius::providedPorts()
{
  return {
    BT::InputPort<double>("radius", 0.0, "Blend radius in meters")
  };
}

BT::NodeStatus SetBlendRadius::tick()
{
  auto radius = getInput<double>("radius").value();
  planning_interface_->setBlendRadius(radius);
  return BT::NodeStatus::SUCCESS;
}

// === SetVelocityScaling ===

SetVelocityScaling::SetVelocityScaling(const std::string& name, const BT::NodeConfiguration& config,
                                       std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: BT::SyncActionNode(name, config)
, planning_interface_(planning_interface)
{
}

BT::PortsList SetVelocityScaling::providedPorts()
{
  return {
    BT::InputPort<double>("factor", 0.5, "Velocity scaling factor (0.0-1.0)")
  };
}

BT::NodeStatus SetVelocityScaling::tick()
{
  auto factor = getInput<double>("factor").value();
  planning_interface_->setVelocityScaling(std::clamp(factor, 0.0, 1.0));
  return BT::NodeStatus::SUCCESS;
}

// === SetAccelerationScaling ===

SetAccelerationScaling::SetAccelerationScaling(const std::string& name, const BT::NodeConfiguration& config,
                                               std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: BT::SyncActionNode(name, config)
, planning_interface_(planning_interface)
{
}

BT::PortsList SetAccelerationScaling::providedPorts()
{
  return {
    BT::InputPort<double>("factor", 0.5, "Acceleration scaling factor (0.0-1.0)")
  };
}

BT::NodeStatus SetAccelerationScaling::tick()
{
  auto factor = getInput<double>("factor").value();
  planning_interface_->setAccelerationScaling(std::clamp(factor, 0.0, 1.0));
  return BT::NodeStatus::SUCCESS;
}

// === ExecutePlan (placeholder) ===

ExecutePlan::ExecutePlan(const std::string& name, const BT::NodeConfiguration& config,
                         rclcpp::Node::SharedPtr node,
                         std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
: ARBTActionNode(name, config, node, planning_interface)
, execution_done_(false)
{
}

BT::PortsList ExecutePlan::providedPorts()
{
  return {
    BT::OutputPort<bool>("success", "Execution succeeded")
  };
}

BT::NodeStatus ExecutePlan::onStart()
{
  // Note: This is a simplified version - in practice you'd get the plan from blackboard
  RCLCPP_WARN(node_->get_logger(), "ExecutePlan: Use MoveToJoint instead for now");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExecutePlan::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void ExecutePlan::onHalted()
{
  execution_done_ = false;
}

// === Registration ===

void registerARBTNodes(BT::BehaviorTreeFactory& factory,
                       rclcpp::Node::SharedPtr node,
                       std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface)
{
  BT::NodeBuilder plan_to_joint_builder = [node, planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<PlanToJoint>(name, config, node, planning_interface);
  };

  BT::NodeBuilder plan_to_pose_builder = [node, planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<PlanToPose>(name, config, node, planning_interface);
  };

  BT::NodeBuilder move_to_joint_builder = [node, planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<MoveToJoint>(name, config, node, planning_interface);
  };

  BT::NodeBuilder set_planner_builder = [planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<SetPlanner>(name, config, planning_interface);
  };

  BT::NodeBuilder set_blend_builder = [planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<SetBlendRadius>(name, config, planning_interface);
  };

  BT::NodeBuilder set_velocity_builder = [planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<SetVelocityScaling>(name, config, planning_interface);
  };

  BT::NodeBuilder set_accel_builder = [planning_interface](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<SetAccelerationScaling>(name, config, planning_interface);
  };

  factory.registerBuilder<PlanToJoint>("PlanToJoint", plan_to_joint_builder);
  factory.registerBuilder<PlanToPose>("PlanToPose", plan_to_pose_builder);
  factory.registerBuilder<MoveToJoint>("MoveToJoint", move_to_joint_builder);
  factory.registerBuilder<SetPlanner>("SetPlanner", set_planner_builder);
  factory.registerBuilder<SetBlendRadius>("SetBlendRadius", set_blend_builder);
  factory.registerBuilder<SetVelocityScaling>("SetVelocityScaling", set_velocity_builder);
  factory.registerBuilder<SetAccelerationScaling>("SetAccelerationScaling", set_accel_builder);
}

} // namespace ar_bt
