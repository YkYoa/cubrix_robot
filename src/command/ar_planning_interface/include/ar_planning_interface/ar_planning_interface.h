#ifndef AR_PLANNING_INTERFACE__AR_PLANNING_INTERFACE_H_
#define AR_PLANNING_INTERFACE__AR_PLANNING_INTERFACE_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>
#include <memory>

namespace ar_planning_interface
{

/**
 * @brief Wrapper around MoveIt planning interface for easier integration
 * 
 * Provides a simplified interface to MoveIt's MoveGroupInterface with
 * support for planning, execution, and visualization.
 */
class ArPlanningInterface
{
public:
  using SharedPtr = std::shared_ptr<ArPlanningInterface>;

  /**
   * @brief Constructor
   * @param node ROS2 node for logging and callbacks
   * @param group_name Planning group name (default: "Arm")
   */
  explicit ArPlanningInterface(const rclcpp::Node::SharedPtr& node, const std::string& group_name = "Arm");
  
  /**
   * @brief Destructor
   */
  ~ArPlanningInterface();

  /**
   * @brief Set the target pose for the end effector
   * @param pose Target pose
   * @return true if target set successfully
   */
  bool setTargetPose(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief Set target joint values
   * @param joints Vector of joint values
   * @return true if target set successfully
   */
  bool setTargetJoints(const std::vector<double>& joints);

  /**
   * @brief Set target joint values by name map
   * @param joints Map of joint names to values
   */
  bool setTargetJoints(const std::map<std::string, double>& joints);

  /**
   * @brief Set named target (e.g., "home", "up")
   * @param name Name of the target
   */
  bool setNamedTarget(const std::string& name);

  /**
   * @brief Plan a trajectory to the set target
   * @param plan Output plan
   * @return true if planning succeeded
   */
  bool plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);

  /**
   * @brief Execute the plan
   * @param plan Plan to execute
   * @return true if execution succeeded
   */
  bool execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

  /**
   * @brief Visualize the planned trajectory in RViz
   * @param plan The plan whose trajectory to visualize
   */
  void visualizeTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

  /**
   * @brief Plan and execute in one go
   * @return true if succeeded
   */
  bool move();

  /**
   * @brief Get the current pose of the end effector
   */
  geometry_msgs::msg::PoseStamped getCurrentPose();

  /**
   * @brief Get the current joint values
   */
  std::vector<double> getCurrentJoints();

  /**
   * @brief Set the planning pipeline adapter
   * @param pipeline_id Pipeline ID (e.g., "ompl", "pilz_industrial_motion_planner")
   */
  void setPlanningPipelineId(const std::string& pipeline_id);

  /**
   * @brief Set the planner ID
   * @param planner_id Planner ID (e.g., "RRTConnect", "PTP")
   */
  void setPlannerId(const std::string& planner_id);

  /**
   * @brief Set velocity scaling factor
   * @param factor Scaling factor (0.0 to 1.0)
   */
  void setVelocityScaling(double factor);

  /**
   * @brief Set acceleration scaling factor
   * @param factor Scaling factor (0.0 to 1.0)
   */
  void setAccelerationScaling(double factor);

  /**
   * @brief Get the current planning pipeline ID
   */
  std::string getPlanningPipelineId() const;

  /**
   * @brief Get the current planner ID
   */
  std::string getPlannerId() const;

  /**
   * @brief Set blend radius for PILZ motion sequences
   * @param radius Blend radius in meters (0.0 = no blending, >0 = smooth blending)
   */
  void setBlendRadius(double radius);

  /**
   * @brief Get the current blend radius
   */
  double getBlendRadius() const;

  /**
   * @brief Get the underlying MoveGroupInterface for advanced operations
   */
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getMoveGroup() const { return move_group_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  std::string group_name_;
  rclcpp::Logger logger_;
  double blend_radius_;  // For PILZ blending
};

} // namespace ar_planning_interface

#endif // AR_PLANNING_INTERFACE__AR_PLANNING_INTERFACE_H_
