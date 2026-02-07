#include "ar_planning_interface/ar_planning_interface.h"

namespace ar_planning_interface
{

ArPlanningInterface::ArPlanningInterface(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
: node_(node),
  group_name_(group_name),
  logger_(rclcpp::get_logger("ar_planning_interface")),
  blend_radius_(0.0)  // Default: no blending
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, group_name);
  
  // Initialize visual tools
  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      node, "Base", "rviz_visual_tools", move_group_->getRobotModel());
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing(true);
}

ArPlanningInterface::~ArPlanningInterface()
{
}

bool ArPlanningInterface::setTargetPose(const geometry_msgs::msg::PoseStamped& pose)
{
  move_group_->setPoseTarget(pose);
  RCLCPP_INFO(logger_, "Target pose set.");
  return true;
}

bool ArPlanningInterface::setTargetJoints(const std::vector<double>& joints)
{
  move_group_->setJointValueTarget(joints);
  RCLCPP_INFO(logger_, "Target joint values set.");
  return true;
}

bool ArPlanningInterface::setTargetJoints(const std::map<std::string, double>& joints)
{
  move_group_->setJointValueTarget(joints);
  RCLCPP_INFO(logger_, "Target joint values set (map).");
  return true;
}

bool ArPlanningInterface::setNamedTarget(const std::string& name)
{
  if (move_group_->setNamedTarget(name)) {
    RCLCPP_INFO(logger_, "Named target '%s' set.", name.c_str());
    return true;
  }
  RCLCPP_ERROR(logger_, "Failed to set named target '%s'.", name.c_str());
  return false;
}

bool ArPlanningInterface::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  auto result = move_group_->plan(plan);
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger_, "Planning succeeded.");
    return true;
  }
  RCLCPP_ERROR(logger_, "Planning failed.");
  return false;
}

void ArPlanningInterface::visualizeTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  visual_tools_->deleteAllMarkers();
  
  // Resolve end effector link
  const moveit::core::JointModelGroup* joint_model_group = 
      move_group_->getCurrentState()->getJointModelGroup(group_name_);
  
  std::string ee_link = move_group_->getEndEffectorLink();
  
  // If no EE link defined, use the last link in the group
  if (ee_link.empty()) {
    const auto& link_names = joint_model_group->getLinkModelNames();
    if (!link_names.empty()) {
      ee_link = link_names.back();
    }
  }
  
  if (ee_link.empty()) {
    RCLCPP_ERROR(logger_, "Could not determine end effector link for visualization");
    return;
  }

  // Publish trajectory line using the specific link
  const moveit::core::LinkModel* link_model = 
      move_group_->getCurrentState()->getLinkModel(ee_link);
      
  visual_tools_->publishTrajectoryLine(plan.trajectory_, link_model, joint_model_group);
  
  // Publish waypoints as axes
  auto robot_state = move_group_->getCurrentState();
  const auto& points = plan.trajectory_.joint_trajectory.points;
  
  // Publish waypoints as spheres
  for (size_t i = 0; i < points.size(); ++i) {
    robot_state->setJointGroupPositions(joint_model_group, points[i].positions);
    const Eigen::Isometry3d& end_effector_state = 
        robot_state->getGlobalLinkTransform(ee_link);
    
    geometry_msgs::msg::Point pt;
    pt.x = end_effector_state.translation().x();
    pt.y = end_effector_state.translation().y();
    pt.z = end_effector_state.translation().z();
    
    visual_tools_->publishSphere(pt, rviz_visual_tools::BLUE, rviz_visual_tools::SMALL);
  }
  
  visual_tools_->trigger();
  RCLCPP_INFO(logger_, "Visualizing trajectory with %zu waypoints for link: %s", points.size(), ee_link.c_str());
}

bool ArPlanningInterface::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  auto result = move_group_->execute(plan);
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger_, "Execution succeeded.");
    return true;
  }
  RCLCPP_ERROR(logger_, "Execution failed.");
  return false;
}

bool ArPlanningInterface::move()
{
  auto result = move_group_->move();
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger_, "Move succeeded.");
    return true;
  }
  RCLCPP_ERROR(logger_, "Move failed.");
  return false;
}

void ArPlanningInterface::stop()
{
  move_group_->stop();
}

geometry_msgs::msg::PoseStamped ArPlanningInterface::getCurrentPose()
{
  return move_group_->getCurrentPose();
}

std::vector<double> ArPlanningInterface::getCurrentJoints()
{
  return move_group_->getCurrentJointValues();
}

void ArPlanningInterface::setPlanningPipelineId(const std::string& pipeline_id)
{
  move_group_->setPlanningPipelineId(pipeline_id);
  RCLCPP_INFO(logger_, "Planning pipeline set to: %s", pipeline_id.c_str());
}

void ArPlanningInterface::setPlannerId(const std::string& planner_id)
{
  move_group_->setPlannerId(planner_id);
  RCLCPP_INFO(logger_, "Planner ID set to: %s", planner_id.c_str());
}

void ArPlanningInterface::setVelocityScaling(double factor)
{
  move_group_->setMaxVelocityScalingFactor(factor);
  RCLCPP_INFO(logger_, "Velocity scaling set to: %.2f", factor);
}

void ArPlanningInterface::setAccelerationScaling(double factor)
{
  move_group_->setMaxAccelerationScalingFactor(factor);
  RCLCPP_INFO(logger_, "Acceleration scaling set to: %.2f", factor);
}

std::string ArPlanningInterface::getPlanningPipelineId() const
{
  return move_group_->getPlanningPipelineId();
}

std::string ArPlanningInterface::getPlannerId() const
{
  return move_group_->getPlannerId();
}

void ArPlanningInterface::setBlendRadius(double radius)
{
  blend_radius_ = radius;
  RCLCPP_INFO(logger_, "Blend radius set to: %.3f meters", radius);
}

double ArPlanningInterface::getBlendRadius() const
{
  return blend_radius_;
}

} // namespace ar_planning_interface
