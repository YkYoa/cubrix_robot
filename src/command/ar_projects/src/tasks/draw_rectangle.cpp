#include "ar_projects/tasks/draw_rectangle.h"
#include "ar_projects/motion_executor.h"
#include <cmath>

namespace ar_projects
{

// Register this task
REGISTER_TASK("draw_rectangle", DrawRectangleTask);

DrawRectangleTask::DrawRectangleTask()
  // Initialize with safe defaults, but rely on configure()
  : width_(0.0)
  , height_(0.0)
  , joint_index_x_(-1)
  , joint_index_y_(-1)
  , delay_ms_(0)
  , use_cartesian_(false)
{
}

bool DrawRectangleTask::configure(const YAML::Node& params)
{
  // Get starting position
  if (params["start_waypoint"]) {
    start_waypoint_ = params["start_waypoint"].as<std::string>();
  }
  
  if (params["start_joints"]) {
    start_joints_.clear();
    for (const auto& j : params["start_joints"]) {
      start_joints_.push_back(j.as<double>());
    }
  }

  // Get rectangle dimensions (in meters for Cartesian, radians for joint)
  if (params["width"]) {
    width_ = params["width"].as<double>();
  }
  
  if (params["height"]) {
    height_ = params["height"].as<double>();
  }

  // Get joint indices for movement (only used in joint mode)
  if (params["joint_index_x"]) {
    joint_index_x_ = params["joint_index_x"].as<int>();
  }
  
  if (params["joint_index_y"]) {
    joint_index_y_ = params["joint_index_y"].as<int>();
  }

  // Get delay between corners
  if (params["delay_ms"]) {
    delay_ms_ = params["delay_ms"].as<int>();
  } else {
    delay_ms_ = 200; // Default if not specified
  }

  // Use Cartesian (end effector) mode
  if (params["use_cartesian"]) {
    use_cartesian_ = params["use_cartesian"].as<bool>();
  }

  return true;
}

bool DrawRectangleTask::execute(MotionExecutor& executor)
{
  executor.log("Starting draw_rectangle task");
  executor.log("Rectangle: width=" + std::to_string(width_) + 
               ", height=" + std::to_string(height_));
  executor.log("Mode: " + std::string(use_cartesian_ ? "Cartesian (EEF)" : "Joint space"));

  // If we have a waypoint name, resolve it first
  if (!start_waypoint_.empty()) {
    executor.log("Moving to start waypoint: " + start_waypoint_);
    if (!executor.moveToWaypoint(start_waypoint_)) {
      executor.log("ERROR: Failed to move to start waypoint");
      return false;
    }
  }

  if (use_cartesian_) {
    return executeCartesian(executor);
  } else {
    return executeJointSpace(executor);
  }
}

bool DrawRectangleTask::executeCartesian(MotionExecutor& executor)
{
  // Get current end effector pose as reference
  geometry_msgs::msg::Pose start_pose = executor.getCurrentPose();
  double start_x = start_pose.position.x;
  double start_y = start_pose.position.y;
  double start_z = start_pose.position.z;

  // Convert quaternion to RPY for consistent orientation
  double roll, pitch, yaw;
  quaternionToRPY(start_pose.orientation, roll, pitch, yaw);

  executor.log("Start EEF position: [" + std::to_string(start_x) + ", " +
               std::to_string(start_y) + ", " + std::to_string(start_z) + "]");
  executor.log("Drawing rectangle in Cartesian space (XY plane)...");

  // Rectangle corners in XY plane, keeping Z constant
  // Corner 0: start (x, y)
  // Corner 1: (x + width, y)
  // Corner 2: (x + width, y + height)
  // Corner 3: (x, y + height)
  
  struct Corner {
    double x_offset;
    double y_offset;
  };
  
  Corner corners[4] = {
    {0.0, 0.0},               // Corner 0: start
    {width_, 0.0},            // Corner 1: +X
    {width_, height_},        // Corner 2: +X, +Y
    {0.0, height_}            // Corner 3: +Y
  };

  for (int i = 0; i < 4; i++) {
    executor.log("Moving to corner " + std::to_string(i + 1) + "/4");
    
    double target_x = start_x + corners[i].x_offset;
    double target_y = start_y + corners[i].y_offset;
    
    // Use moveJoint with position and orientation (keep same Z and orientation)
    if (!executor.moveJoint(target_x, target_y, start_z, roll, pitch, yaw)) {
      executor.log("ERROR: Failed to move to corner " + std::to_string(i + 1));
      return false;
    }

    if (delay_ms_ > 0) {
      executor.delay(delay_ms_);
    }
  }

  // Return to start position
  executor.log("Returning to start position");
  if (!executor.moveJoint(start_x, start_y, start_z, roll, pitch, yaw)) {
    executor.log("ERROR: Failed to return to start");
    return false;
  }

  executor.log("draw_rectangle (Cartesian) task completed successfully");
  return true;
}

bool DrawRectangleTask::executeJointSpace(MotionExecutor& executor)
{
  // Get current joints as starting position if not already set
  if (start_joints_.empty() || start_waypoint_.empty()) {
    start_joints_ = executor.getCurrentJoints();
  }

  executor.log("Drawing rectangle in joint space...");

  for (int corner = 0; corner < 4; corner++) {
    executor.log("Moving to corner " + std::to_string(corner + 1) + "/4");
    
    auto corner_joints = computeCorner(corner);
    if (!executor.moveJoint(corner_joints)) {
      executor.log("ERROR: Failed to move to corner " + std::to_string(corner + 1));
      return false;
    }

    if (delay_ms_ > 0) {
      executor.delay(delay_ms_);
    }
  }

  // Return to start
  executor.log("Returning to start position");
  if (!executor.moveJoint(start_joints_)) {
    executor.log("ERROR: Failed to return to start");
    return false;
  }

  executor.log("draw_rectangle (joint space) task completed successfully");
  return true;
}

std::vector<double> DrawRectangleTask::computeCorner(int corner) const
{
  std::vector<double> joints = start_joints_;
  
  // Ensure we have enough joints
  if (joints.size() <= static_cast<size_t>(std::max(joint_index_x_, joint_index_y_))) {
    return joints;
  }

  // Apply offsets based on corner
  // 0: (0, 0) - start
  // 1: (+width, 0)
  // 2: (+width, +height)
  // 3: (0, +height)
  
  switch (corner) {
    case 0:
      // Start position - no offset
      break;
    case 1:
      joints[joint_index_x_] += width_;
      break;
    case 2:
      joints[joint_index_x_] += width_;
      joints[joint_index_y_] += height_;
      break;
    case 3:
      joints[joint_index_y_] += height_;
      break;
    default:
      break;
  }

  return joints;
}

void DrawRectangleTask::quaternionToRPY(const geometry_msgs::msg::Quaternion& q,
                                         double& roll, double& pitch, double& yaw) const
{
  // Convert quaternion to Euler angles (roll, pitch, yaw)
  double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
  } else {
    pitch = std::asin(sinp);
  }

  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace ar_projects
