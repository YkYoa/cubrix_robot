#include "ar_planning_interface/ar_planning_interface.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include "ar_common/common.h"

namespace ar_planning_interface
{

ArPlanningInterface::ArPlanningInterface(rclcpp::Node::SharedPtr node)
  : node_(node)
{
}

bool ArPlanningInterface::init()
{
  RCLCPP_INFO(node_->get_logger(), "Initializing ArPlanningInterface...");

  // Load the robot model
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node_));
  robot_model_ = robot_model_loader->getModel();
  if (!robot_model_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load robot model");
    return false;
  }

  // Create a planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  if (!planning_scene_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create planning scene");
    return false;
  }

  // Initialize the planning pipeline
  planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model_, node_, std::string(ar::getDefaultPlannerPipeline())));
  if (!planning_pipeline_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create planning pipeline: Check 'planning_plugin' parameter and MoveIt2 installation.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "ArPlanningInterface initialized successfully.");
  return true;
}

bool ArPlanningInterface::plan(const moveit_msgs::msg::MotionPlanRequest& req, moveit_msgs::msg::MotionPlanResponse& res)
{
  RCLCPP_INFO(node_->get_logger(), "Received motion plan request for group: %s", req.group_name.c_str());

  planning_interface::MotionPlanRequest planning_req = req;
  planning_interface::MotionPlanResponse planning_res;

  // Check if planning pipeline is initialized
  if (!planning_pipeline_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Planning pipeline not initialized. Cannot plan.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  // Check if planning scene is valid
  if (!planning_scene_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Planning scene not valid. Cannot plan.");
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  planning_pipeline_->generatePlan(planning_scene_, planning_req, planning_res);

  // Assign the trajectory directly if a plan was found
  if (planning_res.trajectory_)
  {
    res.trajectory_ = *planning_res.trajectory_;
  }

  if (planning_res.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    if (planning_res.trajectory_)
    {
      RCLCPP_INFO(node_->get_logger(), "Motion plan succeeded. Found trajectory with %zu points.", planning_res.trajectory_->joint_trajectory.points.size());
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Motion plan succeeded but no trajectory found. This should not happen.");
      res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Motion plan failed with error code: %d - %s", planning_res.error_code_.val, 
                 moveit::core::MoveItErrorCodes::errorStrings[planning_res.error_code_.val].c_str());
    res.error_code_.val = planning_res.error_code_.val;
    return false;
  }
}

} // namespace ar_planning_interface
