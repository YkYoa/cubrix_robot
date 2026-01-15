#ifndef AR_MOTION_PLANNER__AR_MOTION_PLANNER_H_
#define AR_MOTION_PLANNER__AR_MOTION_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <ar_planning_interface/ar_planning_interface.h>
#include "ar_motion_planner/planner_types.h"
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <thread>

namespace ar_motion_planner
{

/**
 * @brief ROS2 node for motion planning service
 * 
 * Provides a service interface for motion planning requests.
 * Wraps the ArPlanningInterface to expose planning functionality via ROS2 services.
 */
class MotionPlanner : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  MotionPlanner();
  
  /**
   * @brief Destructor
   */
  ~MotionPlanner();

private:
  /**
   * @brief Initialize ROS2 parameters
   * 
   * Loads planning group, pipeline, planner ID, and scaling factors from parameters.
   */
  void initializeParameters();
  
  /**
   * @brief Initialize ROS2 services
   * 
   * Creates the motion planning service server.
   */
  void initializeServices();
  
  /**
   * @brief Spin thread for executor
   * 
   * Runs the executor in a separate thread to handle service callbacks.
   */
  void spinThread();
  
  /**
   * @brief Handle motion planning service requests
   * @param request Planning request containing target pose/joints
   * @param response Planning response containing the plan
   */
  void planCallback(
    const std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request> request,
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response> response);
  
  /**
   * @brief Handle parameter change requests
   * @param parameters Vector of parameters to set
   * @return Result indicating success or failure
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters);

  // Members
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
  rclcpp::Service<moveit_msgs::srv::GetMotionPlan>::SharedPtr plan_service_;
  
  std::shared_ptr<std::thread> spin_thread_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  
  // Parameters
  std::string planning_group_;
  std::string planning_pipeline_;
  std::string planner_id_;
  double velocity_scaling_;
  double acceleration_scaling_;
  
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace ar_motion_planner

#endif // AR_MOTION_PLANNER__AR_MOTION_PLANNER_H_
