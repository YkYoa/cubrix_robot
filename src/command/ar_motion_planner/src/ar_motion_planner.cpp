#include "ar_motion_planner/ar_motion_planner.h"

namespace ar_motion_planner
{

MotionPlanner::MotionPlanner()
: Node("ar_motion_planner")
{
  RCLCPP_INFO(this->get_logger(), "Initializing AR Motion Planner...");
  
  // Initialize parameters first
  initializeParameters();
  
  // Create multi-threaded executor for non-blocking MoveIt operations
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(shared_from_this());
  
  // Start spinning in separate thread
  spin_thread_ = std::make_shared<std::thread>([this]() { this->spinThread(); });
  
  // Small delay to ensure executor is running before initializing MoveIt
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // Initialize planning interface with configured group
  planning_interface_ = std::make_shared<ar_planning_interface::ArPlanningInterface>(
    shared_from_this(), planning_group_);
  
  // Apply initial configuration
  planning_interface_->setPlanningPipelineId(planning_pipeline_);
  planning_interface_->setPlannerId(planner_id_);
  planning_interface_->setVelocityScaling(velocity_scaling_);
  planning_interface_->setAccelerationScaling(acceleration_scaling_);
  
  // Initialize services
  initializeServices();
  
  RCLCPP_INFO(this->get_logger(), "AR Motion Planner initialized successfully");
  RCLCPP_INFO(this->get_logger(), "  Planning Group: %s", planning_group_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Planning Pipeline: %s", planning_pipeline_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Planner ID: %s", planner_id_.c_str());
}

MotionPlanner::~MotionPlanner()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }
}

void MotionPlanner::initializeParameters()
{
  // Declare parameters with defaults
  this->declare_parameter<std::string>("planning_group", "Arm");
  this->declare_parameter<std::string>("planning_pipeline", "ompl");
  this->declare_parameter<std::string>("planner_id", "RRTConnect");
  this->declare_parameter<double>("velocity_scaling", 0.5);
  this->declare_parameter<double>("acceleration_scaling", 0.5);
  
  // Get parameter values
  planning_group_ = this->get_parameter("planning_group").as_string();
  planning_pipeline_ = this->get_parameter("planning_pipeline").as_string();
  planner_id_ = this->get_parameter("planner_id").as_string();
  velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
  acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();
  
  // Register parameter callback for dynamic reconfiguration
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MotionPlanner::parametersCallback, this, std::placeholders::_1));
}

void MotionPlanner::initializeServices()
{
  // Create motion planning service
  plan_service_ = this->create_service<moveit_msgs::srv::GetMotionPlan>(
    "~/plan",
    std::bind(&MotionPlanner::planCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(this->get_logger(), "Motion planning service ready at: ~/plan");
}

void MotionPlanner::spinThread()
{
  RCLCPP_INFO(this->get_logger(), "Executor thread started");
  executor_->spin();
  RCLCPP_INFO(this->get_logger(), "Executor thread stopped");
}

void MotionPlanner::planCallback(
  const std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request> request,
  std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received planning request");
  
  // Extract motion plan request
  const auto& motion_request = request->motion_plan_request;
  
  // Set target based on goal constraints
  if (!motion_request.goal_constraints.empty()) {
    const auto& goal = motion_request.goal_constraints[0];
    
    // Check if joint constraints exist
    if (!goal.joint_constraints.empty()) {
      RCLCPP_INFO(this->get_logger(), "Planning to joint target");
      std::vector<double> joint_values;
      for (const auto& jc : goal.joint_constraints) {
        joint_values.push_back(jc.position);
      }
      planning_interface_->setTargetJoints(joint_values);
    }
    // Check if position constraints exist
    else if (!goal.position_constraints.empty()) {
      RCLCPP_INFO(this->get_logger(), "Planning to pose target");
      // Create pose from position constraint
      geometry_msgs::msg::PoseStamped pose;
      pose.header = goal.position_constraints[0].header;
      pose.pose.position.x = goal.position_constraints[0].constraint_region.primitive_poses[0].position.x;
      pose.pose.position.y = goal.position_constraints[0].constraint_region.primitive_poses[0].position.y;
      pose.pose.position.z = goal.position_constraints[0].constraint_region.primitive_poses[0].position.z;
      
      // If orientation constraints exist, use them
      if (!goal.orientation_constraints.empty()) {
        pose.pose.orientation = goal.orientation_constraints[0].orientation;
      } else {
        pose.pose.orientation.w = 1.0;
      }
      
      planning_interface_->setTargetPose(pose);
    }
  }
  
  // Execute planning
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = planning_interface_->plan(plan);
  
  if (success) {
    response->motion_plan_response.trajectory = plan.trajectory_;
    response->motion_plan_response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    RCLCPP_INFO(this->get_logger(), "Planning succeeded");
  } else {
    response->motion_plan_response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    RCLCPP_ERROR(this->get_logger(), "Planning failed");
  }
}

rcl_interfaces::msg::SetParametersResult MotionPlanner::parametersCallback(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto& param : parameters) {
    if (param.get_name() == "planning_pipeline") {
      planning_pipeline_ = param.as_string();
      if (planning_interface_) {
        planning_interface_->setPlanningPipelineId(planning_pipeline_);
      }
      RCLCPP_INFO(this->get_logger(), "Updated planning pipeline to: %s", planning_pipeline_.c_str());
    }
    else if (param.get_name() == "planner_id") {
      planner_id_ = param.as_string();
      if (planning_interface_) {
        planning_interface_->setPlannerId(planner_id_);
      }
      RCLCPP_INFO(this->get_logger(), "Updated planner ID to: %s", planner_id_.c_str());
    }
    else if (param.get_name() == "velocity_scaling") {
      velocity_scaling_ = param.as_double();
      if (planning_interface_) {
        planning_interface_->setVelocityScaling(velocity_scaling_);
      }
      RCLCPP_INFO(this->get_logger(), "Updated velocity scaling to: %.2f", velocity_scaling_);
    }
    else if (param.get_name() == "acceleration_scaling") {
      acceleration_scaling_ = param.as_double();
      if (planning_interface_) {
        planning_interface_->setAccelerationScaling(acceleration_scaling_);
      }
      RCLCPP_INFO(this->get_logger(), "Updated acceleration scaling to: %.2f", acceleration_scaling_);
    }
  }
  
  return result;
}

} // namespace ar_motion_planner

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ar_motion_planner::MotionPlanner>();
  
  // Node spins in its own thread, so we just wait here
  rclcpp::shutdown();
  return 0;
}
