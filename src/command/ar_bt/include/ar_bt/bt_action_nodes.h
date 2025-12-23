#ifndef AR_BT__BT_ACTION_NODES_H_
#define AR_BT__BT_ACTION_NODES_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <ar_planning_interface/ar_planning_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <string>

namespace ar_bt
{

/**
 * @brief Base class for AR BT action nodes with planning interface access
 */
class ARBTActionNode : public BT::StatefulActionNode
{
public:
  ARBTActionNode(const std::string& name, const BT::NodeConfiguration& config,
                 rclcpp::Node::SharedPtr node,
                 std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
};

/**
 * @brief Plan to joint configuration
 * Input Ports:
 *   - target_joints: vector<double> - Target joint values
 * Output Ports:
 *   - success: bool - Planning succeeded
 */
class PlanToJoint : public ARBTActionNode
{
public:
  PlanToJoint(const std::string& name, const BT::NodeConfiguration& config,
              rclcpp::Node::SharedPtr node,
              std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
  bool planning_done_;
};

/**
 * @brief Plan to Cartesian pose
 * Input Ports:
 *   - target_pose: PoseStamped - Target pose
 * Output Ports:
 *   - success: bool - Planning succeeded
 */
class PlanToPose : public ARBTActionNode
{
public:
  PlanToPose(const std::string& name, const BT::NodeConfiguration& config,
             rclcpp::Node::SharedPtr node,
             std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
  bool planning_done_;
};

/**
 * @brief Execute planned trajectory
 * Input Ports:
 *   - plan: Plan - Trajectory to execute (from blackboard)
 * Output Ports:
 *   - success: bool - Execution succeeded
 */
class ExecutePlan : public ARBTActionNode
{
public:
  ExecutePlan(const std::string& name, const BT::NodeConfiguration& config,
              rclcpp::Node::SharedPtr node,
              std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool execution_done_;
};

/**
 * @brief Combined plan and execute to joint target
 * Input Ports:
 *   - target_joints: vector<double> - Target joint values
 */
class MoveToJoint : public ARBTActionNode
{
public:
  MoveToJoint(const std::string& name, const BT::NodeConfiguration& config,
              rclcpp::Node::SharedPtr node,
              std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool motion_done_;
};

/**
 * @brief Set planning pipeline and planner
 * Input Ports:
 *   - pipeline: string - Pipeline ID ("ompl", "pilz")
 *   - planner: string - Planner ID ("RRTConnect", "PTP", etc.)
 */
class SetPlanner : public BT::SyncActionNode
{
public:
  SetPlanner(const std::string& name, const BT::NodeConfiguration& config,
             std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
};

/**
 * @brief Set blend radius for PILZ sequences
 * Input Ports:
 *   - radius: double - Blend radius in meters
 */
class SetBlendRadius : public BT::SyncActionNode
{
public:
  SetBlendRadius(const std::string& name, const BT::NodeConfiguration& config,
                 std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
};

/**
 * @brief Set velocity scaling factor
 * Input Ports:
 *   - factor: double - Velocity scaling (0.0 to 1.0)
 */
class SetVelocityScaling : public BT::SyncActionNode
{
public:
  SetVelocityScaling(const std::string& name, const BT::NodeConfiguration& config,
                     std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
};

/**
 * @brief Set acceleration scaling factor
 * Input Ports:
 *   - factor: double - Acceleration scaling (0.0 to 1.0)
 */
class SetAccelerationScaling : public BT::SyncActionNode
{
public:
  SetAccelerationScaling(const std::string& name, const BT::NodeConfiguration& config,
                         std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
};

// Register all AR BT nodes with factory
void registerARBTNodes(BT::BehaviorTreeFactory& factory,
                       rclcpp::Node::SharedPtr node,
                       std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface);

} // namespace ar_bt

#endif // AR_BT__BT_ACTION_NODES_H_
