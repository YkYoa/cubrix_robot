#ifndef AR_PLANNING_INTERFACE_H
#define AR_PLANNING_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace ar_planning_interface
{

class ArPlanningInterface
{
public:
  ArPlanningInterface(rclcpp::Node::SharedPtr node);

  bool init();

  bool plan(const moveit_msgs::msg::MotionPlanRequest& req, moveit_msgs::msg::MotionPlanResponse& res);

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
};

} // namespace ar_planning_interface

#endif // AR_PLANNING_INTERFACE_H
