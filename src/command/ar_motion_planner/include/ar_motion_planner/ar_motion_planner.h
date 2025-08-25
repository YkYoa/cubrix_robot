#ifndef __AR_MOTION_PLANNER_H__
#define __AR_MOTION_PLANNER_H__

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_interface/planning_interface.h>


#include <pluginlib/class_loader.hpp>

namespace ar_motion_planner
{

class ArPlanningContext : public planning_interface::PlanningContext
{
public:
  ArPlanningContext(const std::string& name, const std::string& group);

  bool solve(planning_interface::MotionPlanResponse& res) override;

  bool solve(planning_interface::MotionPlanDetailedResponse& res, const planning_interface::PlannerTerminationCallback& callback) override;

  bool terminate() override;

  void clear() override;
};

class ArMotionPlanner : public planning_interface::PlannerManager
{
public:
  ArMotionPlanner();

  bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& package_path) override;

  bool canHandleRequest(const planning_interface::MotionPlanRequest& req, moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  std::string getDescription() const override
  {
    return "AR Motion Planner";
  }

  void getCapabilities(planning_interface::PlannerManager::Capabilities& cap) const override;

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

  void setNode(rclcpp::Node::SharedPtr node) override
  {
    node_ = node;
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
};

} // namespace ar_motion_planner

#endif // __AR_MOTION_PLANNER_H__
