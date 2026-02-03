#include "ar_projects/motion_executor.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <chrono>

namespace ar_projects
{

class MotionExecutor::Impl
{
public:
  Impl(rclcpp::Node::SharedPtr node, const std::string& planning_group)
    : node_(node)
    , planning_group_(planning_group)
    , velocity_scaling_(0.5)
    , acceleration_scaling_(0.5)
  {
    // Create MoveIt interface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, planning_group_);
    
    // Set default planner (Pilz PTP for point-to-point)
    move_group_->setPlanningPipelineId("pilz");
    move_group_->setPlannerId("PTP");
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    
    RCLCPP_INFO(node_->get_logger(), "MotionExecutor initialized for group: %s", 
                planning_group_.c_str());
  }

  bool moveJoint(const std::vector<double>& joints)
  {
    if (!move_group_) {
      log("ERROR: MoveGroup interface not initialized");
      return false;
    }

    log("Moving to joint target: [" + jointsToString(joints) + "]");

    move_group_->setJointValueTarget(joints);

    auto result = move_group_->move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      log("ERROR: Motion execution failed with code: " + std::to_string(result.val));
      return false;
    }

    log("Motion completed successfully");
    return true;
  }

  bool moveToWaypoint(const std::string& waypoint_name)
  {
    auto it = waypoints_.find(waypoint_name);
    if (it == waypoints_.end()) {
      log("ERROR: Waypoint not found: " + waypoint_name);
      return false;
    }
    
    log("Moving to waypoint: " + waypoint_name);
    return moveJoint(it->second);
  }

  void delay(int ms)
  {
    log("Waiting " + std::to_string(ms) + " ms");
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  void setVelocityScaling(double factor)
  {
    velocity_scaling_ = factor;
    if (move_group_) {
      move_group_->setMaxVelocityScalingFactor(factor);
    }
  }

  void setAccelerationScaling(double factor)
  {
    acceleration_scaling_ = factor;
    if (move_group_) {
      move_group_->setMaxAccelerationScalingFactor(factor);
    }
  }

  std::vector<double> getCurrentJoints() const
  {
    if (move_group_) {
      return move_group_->getCurrentJointValues();
    }
    return {};
  }

  void setWaypoints(const std::map<std::string, std::vector<double>>& waypoints)
  {
    waypoints_ = waypoints;
  }

  void log(const std::string& msg)
  {
    RCLCPP_INFO(node_->get_logger(), "[MotionExecutor] %s", msg.c_str());
  }

  bool isReady() const
  {
    return move_group_ != nullptr;
  }

private:
  std::string jointsToString(const std::vector<double>& joints) const
  {
    std::string result;
    for (size_t i = 0; i < joints.size(); i++) {
      if (i > 0) result += ", ";
      result += std::to_string(joints[i]);
    }
    return result;
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::map<std::string, std::vector<double>> waypoints_;
  double velocity_scaling_;
  double acceleration_scaling_;
};

// Public interface implementation

MotionExecutor::MotionExecutor(rclcpp::Node::SharedPtr node, const std::string& planning_group)
  : impl_(std::make_unique<Impl>(node, planning_group))
{
}

MotionExecutor::~MotionExecutor() = default;

bool MotionExecutor::moveJoint(const std::vector<double>& joints)
{
  return impl_->moveJoint(joints);
}

bool MotionExecutor::moveToWaypoint(const std::string& waypoint_name)
{
  return impl_->moveToWaypoint(waypoint_name);
}

void MotionExecutor::delay(int ms)
{
  impl_->delay(ms);
}

void MotionExecutor::setVelocityScaling(double factor)
{
  impl_->setVelocityScaling(factor);
}

void MotionExecutor::setAccelerationScaling(double factor)
{
  impl_->setAccelerationScaling(factor);
}

std::vector<double> MotionExecutor::getCurrentJoints() const
{
  return impl_->getCurrentJoints();
}

void MotionExecutor::setWaypoints(const std::map<std::string, std::vector<double>>& waypoints)
{
  impl_->setWaypoints(waypoints);
}

void MotionExecutor::log(const std::string& msg)
{
  impl_->log(msg);
}

bool MotionExecutor::isReady() const
{
  return impl_->isReady();
}

}  // namespace ar_projects
