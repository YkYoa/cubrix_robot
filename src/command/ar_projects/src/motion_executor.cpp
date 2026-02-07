#include "ar_projects/motion_executor.h"
#include <ar_planning_interface/ar_planning_interface.h>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <yaml-cpp/yaml.h>

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
    , plan_caching_enabled_(true)
  {
    // Create ArPlanningInterface (includes visualization)
    planning_interface_ = std::make_shared<ar_planning_interface::ArPlanningInterface>(
      node_, planning_group_);
    
    // Set default planner (Pilz PTP for point-to-point)
    planning_interface_->setPlanningPipelineId("pilz");
    planning_interface_->setPlannerId("PTP");
    planning_interface_->setVelocityScaling(velocity_scaling_);
    planning_interface_->setAccelerationScaling(acceleration_scaling_);
    
    // Ensure plan cache directory exists
    // Default to a folder in home if not set
    plan_cache_dir_ = std::string(getenv("HOME")) + "/ar_plans";
    // We don't create it here anymore, as it might be changed
    
    RCLCPP_INFO(node_->get_logger(), "MotionExecutor (YAML) initialized for group: %s", 
                planning_group_.c_str());
  }

  void setPlanCacheDir(const std::string& dir)
  {
    plan_cache_dir_ = dir;
    if (plan_caching_enabled_ && !plan_cache_dir_.empty()) {
      try {
        std::filesystem::create_directories(plan_cache_dir_);
      } catch (const std::exception& e) {
        log("ERROR: Failed to create plan cache dir: " + std::string(e.what()));
      }
    }
  }

  void setLogCallback(std::function<void(const std::string&)> callback)
  {
    log_callback_ = callback;
  }

  bool moveJoint(const std::vector<double>& joints)
  {
    if (!planning_interface_) {
      log("ERROR: Planning interface not initialized");
      return false;
    }

    log("Moving to joint target: [" + jointsToString(joints) + "]");

    // Check for cached plan
    const auto current_joints = planning_interface_->getCurrentJoints();
    std::string plan_hash = computePlanHash(joints, current_joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool use_cached_plan = false;
    
    if (plan_caching_enabled_ && loadPlan(plan_hash, plan)) {
      if (isPlanStartStateValid(plan, current_joints, kStartTolerance)) {
        log("Using cached plan: " + plan_hash);
        use_cached_plan = true;
        
        // Visualize the cached plan so user sees the update
        if (planning_interface_) {
          // Update Goal State in RViz
          planning_interface_->setTargetJoints(joints);
          // Update Path in RViz
          planning_interface_->visualizeTrajectory(plan);
        }
      } else {
        log("Cached plan start state mismatch (max deviation: " +
            std::to_string(maxStartDeviation(plan, current_joints)) + 
            "), replanning");
      }
    } else {
      if (plan_caching_enabled_) {
        log("No cached plan found for hash: " + plan_hash);
      }
    }

    if (!use_cached_plan) {
      // Plan new trajectory
      // planning_interface_->setTargetJoints(joints); // Already implicitly set by intention, but ensuring it's set if we skipped it?
      // Actually, let's just set it always at the top, OR inside the cached block too?
      // Better to set it always if we want the visual update.
      
      // But wait, if we move it up, we need to be careful not to break logic.
      // Let's just add it to the cached block to be safe and explicit.
      planning_interface_->setTargetJoints(joints);
      
      if (!planning_interface_->plan(plan)) {
        log("ERROR: Planning failed");
        return false;
      }
      
      // Visualize trajectory in RViz
      planning_interface_->visualizeTrajectory(plan);
      
      // Cache the plan
      if (plan_caching_enabled_) {
        savePlan(plan_hash, plan);
        log("Cached plan: " + plan_hash);
      }
    }

    // Execute
    if (!planning_interface_->execute(plan)) {
      log("ERROR: Execution failed");
      return false;
    }

    log("Motion completed successfully");
    return true;
  }

  bool moveJoint(const geometry_msgs::msg::Pose& eef_pose, const std::string& end_effector_link = "")
  {
    if (!planning_interface_) {
      log("ERROR: Planning interface not initialized");
      return false;
    }

    log("Moving to EEF pose: [" + eef_poseToString(eef_pose) + "]");

    // Set end effector link if specified
    auto move_group = planning_interface_->getMoveGroup();
    if (!end_effector_link.empty() && move_group) {
      move_group->setEndEffectorLink(end_effector_link);
    }

    // Create pose stamped
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "Base";
    pose_stamped.pose = eef_pose;
    
    planning_interface_->setTargetPose(pose_stamped);
    
    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!planning_interface_->plan(plan)) {
      log("ERROR: Planning failed");
      return false;
    }
    
    // Visualize
    planning_interface_->visualizeTrajectory(plan);
    
    // Execute
    if (!planning_interface_->execute(plan)) {
      log("ERROR: Execution failed");
      return false;
    }

    log("Motion completed successfully");
    return true;
  }

  bool moveJoint(double x, double y, double z, 
                 double roll, double pitch, double yaw,
                 const std::string& end_effector_link = "")
  {
    geometry_msgs::msg::Pose pose = poseFromRPY(x, y, z, roll, pitch, yaw);
    return moveJoint(pose, end_effector_link);
  }

  bool moveJointPosition(double x, double y, double z,
                         const std::string& end_effector_link = "")
  {
    if (!planning_interface_) {
      log("ERROR: Planning interface not initialized");
      return false;
    }

    // Get current pose and keep orientation
    geometry_msgs::msg::Pose pose = planning_interface_->getCurrentPose().pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    
    log("Moving to position: [" + std::to_string(x) + ", " + 
        std::to_string(y) + ", " + std::to_string(z) + "] (keeping orientation)");
    
    return moveJoint(pose, end_effector_link);
  }

  geometry_msgs::msg::Pose getCurrentPose() const
  {
    if (planning_interface_) {
      return planning_interface_->getCurrentPose().pose;
    }
    return geometry_msgs::msg::Pose();
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
    if (planning_interface_) {
      planning_interface_->setVelocityScaling(factor);
    }
  }

  void setAccelerationScaling(double factor)
  {
    acceleration_scaling_ = factor;
    if (planning_interface_) {
      planning_interface_->setAccelerationScaling(factor);
    }
  }

  std::vector<double> getCurrentJoints() const
  {
    if (planning_interface_) {
      return planning_interface_->getCurrentJoints();
    }
    return {};
  }

  void setWaypoints(const std::map<std::string, std::vector<double>>& waypoints)
  {
    waypoints_ = waypoints;
  }

  void setPlanCaching(bool enabled)
  {
    plan_caching_enabled_ = enabled;
    log(std::string("Plan caching ") + (enabled ? "enabled" : "disabled"));
    
    if (enabled && !plan_cache_dir_.empty()) {
      try {
        std::filesystem::create_directories(plan_cache_dir_);
      } catch (...) {}
    }
  }

  void clearPlanCache()
  {
    try {
      for (const auto& entry : std::filesystem::directory_iterator(plan_cache_dir_)) {
        if (entry.path().extension() == ".yaml") {
          std::filesystem::remove(entry.path());
        }
      }
      log("Plan cache cleared");
    } catch (const std::exception& e) {
      log("ERROR: Failed to clear plan cache: " + std::string(e.what()));
    }
  }

  void log(const std::string& msg)
  {
    RCLCPP_INFO(node_->get_logger(), "[MotionExecutor] %s", msg.c_str());
    if (log_callback_) {
      log_callback_(msg);
    }
  }

  void stop()
  {
    if (planning_interface_) {
      planning_interface_->stop();
      log("Motion stopped by user");
    }
  }

  bool isReady() const
  {
    return planning_interface_ != nullptr;
  }

private:
  static constexpr double kStartTolerance = 0.01;

  bool isPlanStartStateValid(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::vector<double>& current_joints,
    double tolerance) const
  {
    const auto& traj = plan.trajectory_.joint_trajectory;
    if (traj.points.empty()) {
      return false;
    }
    const auto& start_positions = traj.points.front().positions;
    if (current_joints.empty() || start_positions.size() != current_joints.size()) {
      return false;
    }
    for (size_t i = 0; i < start_positions.size(); i++) {
      if (std::abs(start_positions[i] - current_joints[i]) > tolerance) {
        return false;
      }
    }
    return true;
  }

  double maxStartDeviation(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::vector<double>& current_joints) const
  {
    const auto& traj = plan.trajectory_.joint_trajectory;
    if (traj.points.empty()) {
      return 0.0;
    }
    const auto& start_positions = traj.points.front().positions;
    if (current_joints.empty() || start_positions.size() != current_joints.size()) {
      return 0.0;
    }
    double max_dev = 0.0;
    for (size_t i = 0; i < start_positions.size(); i++) {
      max_dev = std::max(max_dev, std::abs(start_positions[i] - current_joints[i]));
    }
    return max_dev;
  }

  std::string jointsToString(const std::vector<double>& joints) const
  {
    std::string result;
    for (size_t i = 0; i < joints.size(); i++) {
      if (i > 0) result += ", ";
      result += std::to_string(joints[i]);
    }
    return result;
  }

  std::string eef_poseToString(const geometry_msgs::msg::Pose& pose) const
  {
    return "pos(" + std::to_string(pose.position.x) + ", " +
           std::to_string(pose.position.y) + ", " +
           std::to_string(pose.position.z) + "), quat(" +
           std::to_string(pose.orientation.x) + ", " +
           std::to_string(pose.orientation.y) + ", " +
           std::to_string(pose.orientation.z) + ", " +
           std::to_string(pose.orientation.w) + ")";
  }

  geometry_msgs::msg::Pose poseFromRPY(double x, double y, double z,
                                        double roll, double pitch, double yaw) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Convert RPY to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    pose.orientation.w = cr * cp * cy + sr * sp * sy;
    pose.orientation.x = sr * cp * cy - cr * sp * sy;
    pose.orientation.y = cr * sp * cy + sr * cp * sy;
    pose.orientation.z = cr * cp * sy - sr * sp * cy;

    return pose;
  }

  std::string computePlanHash(const std::vector<double>& joints,
                              const std::vector<double>& start_joints) const
  {
    std::stringstream ss;
    ss << "target:";
    for (const auto& j : joints) {
      ss << std::fixed << std::setprecision(4) << j << "_";
    }
    ss << "start:";
    if (!start_joints.empty()) {
      for (const auto& j : start_joints) {
        double quantized = std::round(j / kStartTolerance) * kStartTolerance;
        ss << std::fixed << std::setprecision(2) << quantized << "_";
      }
    }
    ss << "vel:" << std::fixed << std::setprecision(2) << velocity_scaling_
       << "_acc:" << std::fixed << std::setprecision(2) << acceleration_scaling_;
    if (planning_interface_) {
      ss << "_pipe:" << planning_interface_->getPlanningPipelineId()
         << "_planner:" << planning_interface_->getPlannerId();
    }
    ss << "_group:" << planning_group_;
    size_t hash = std::hash<std::string>{}(ss.str());
    return std::to_string(hash);
  }

  bool loadPlan(const std::string& hash, moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    std::string filepath = plan_cache_dir_ + "/" + hash + ".yaml";
    
    try {
      if (!std::filesystem::exists(filepath)) {
        return false;
      }
      
      YAML::Node node = YAML::LoadFile(filepath);
      
      if (!node["trajectory"] || !node["trajectory"]["joint_trajectory"]) {
        return false;
      }
      
      const auto& traj_node = node["trajectory"]["joint_trajectory"];
      auto& joint_traj = plan.trajectory_.joint_trajectory;
      
      // Joint Names
      if (traj_node["joint_names"]) {
        for (const auto& name : traj_node["joint_names"]) {
          joint_traj.joint_names.push_back(name.as<std::string>());
        }
      }
      
      // Points
      if (traj_node["points"]) {
        for (const auto& point_node : traj_node["points"]) {
          trajectory_msgs::msg::JointTrajectoryPoint point;
          
          if (point_node["positions"]) {
             for (const auto& val : point_node["positions"]) {
               point.positions.push_back(val.as<double>());
             }
          }
          if (point_node["velocities"]) {
             for (const auto& val : point_node["velocities"]) {
               point.velocities.push_back(val.as<double>());
             }
          }
          if (point_node["accelerations"]) {
             for (const auto& val : point_node["accelerations"]) {
               point.accelerations.push_back(val.as<double>());
             }
          }
          
          if (point_node["time_from_start"]) {
             double sec = point_node["time_from_start"].as<double>();
             point.time_from_start.sec = static_cast<int32_t>(sec);
             point.time_from_start.nanosec = static_cast<uint32_t>((sec - point.time_from_start.sec) * 1e9);
          }
          
          joint_traj.points.push_back(point);
        }
      }
      
      return true;
    } catch (...) {
      return false;
    }
  }

  void savePlan(const std::string& hash, const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    if (plan_cache_dir_.empty()) return;
    std::string filepath = plan_cache_dir_ + "/" + hash + ".yaml";
    
    try {
      YAML::Node node;
      YAML::Node traj_node;
      YAML::Node joint_traj_node;
      
      const auto& trajectory = plan.trajectory_.joint_trajectory;
      
      // Joint Names
      for (const auto& name : trajectory.joint_names) {
        joint_traj_node["joint_names"].push_back(name);
      }
      
      // Points
      for (const auto& point : trajectory.points) {
        YAML::Node point_node;
        
        for (double val : point.positions) point_node["positions"].push_back(val);
        for (double val : point.velocities) point_node["velocities"].push_back(val);
        for (double val : point.accelerations) point_node["accelerations"].push_back(val);
        
        double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        point_node["time_from_start"] = time;
        
        joint_traj_node["points"].push_back(point_node);
      }
      
      traj_node["joint_trajectory"] = joint_traj_node;
      node["trajectory"] = traj_node;
      
      std::ofstream file(filepath);
      file << node;
    } catch (const std::exception& e) {
      log("WARNING: Error saving plan: " + std::string(e.what()));
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
  std::map<std::string, std::vector<double>> waypoints_;
  double velocity_scaling_;
  double acceleration_scaling_;
  bool plan_caching_enabled_;
  std::string plan_cache_dir_;
  std::function<void(const std::string&)> log_callback_;
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

bool MotionExecutor::moveJoint(const geometry_msgs::msg::Pose& eef_pose, 
                                const std::string& end_effector_link)
{
  return impl_->moveJoint(eef_pose, end_effector_link);
}

bool MotionExecutor::moveJoint(double x, double y, double z, 
                                double roll, double pitch, double yaw,
                                const std::string& end_effector_link)
{
  return impl_->moveJoint(x, y, z, roll, pitch, yaw, end_effector_link);
}

bool MotionExecutor::moveJointPosition(double x, double y, double z,
                                        const std::string& end_effector_link)
{
  return impl_->moveJointPosition(x, y, z, end_effector_link);
}

geometry_msgs::msg::Pose MotionExecutor::getCurrentPose() const
{
  return impl_->getCurrentPose();
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

void MotionExecutor::setPlanCaching(bool enabled)
{
  impl_->setPlanCaching(enabled);
}

void MotionExecutor::setPlanCacheDir(const std::string& dir)
{
  impl_->setPlanCacheDir(dir);
}

void MotionExecutor::clearPlanCache()
{
  impl_->clearPlanCache();
}

void MotionExecutor::log(const std::string& msg)
{
  impl_->log(msg);
}

void MotionExecutor::setLogCallback(std::function<void(const std::string&)> callback)
{
  impl_->setLogCallback(callback);
}

void MotionExecutor::stop()
{
  impl_->stop();
}

bool MotionExecutor::isReady() const
{
  return impl_->isReady();
}

}  // namespace ar_projects
