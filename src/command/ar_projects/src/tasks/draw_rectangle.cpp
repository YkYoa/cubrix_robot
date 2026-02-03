#include "ar_projects/tasks/draw_rectangle.h"
#include "ar_projects/motion_executor.h"

namespace ar_projects
{

// Register this task
REGISTER_TASK("draw_rectangle", DrawRectangleTask);

DrawRectangleTask::DrawRectangleTask()
  : start_joints_({0, 0, 0, 0, 0, 0})
  , width_(0.1)
  , height_(0.05)
  , joint_index_x_(0)
  , joint_index_y_(1)
  , delay_ms_(200)
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

  // Get rectangle dimensions
  if (params["width"]) {
    width_ = params["width"].as<double>();
  }
  
  if (params["height"]) {
    height_ = params["height"].as<double>();
  }

  // Get joint indices for movement
  if (params["joint_index_x"]) {
    joint_index_x_ = params["joint_index_x"].as<int>();
  }
  
  if (params["joint_index_y"]) {
    joint_index_y_ = params["joint_index_y"].as<int>();
  }

  // Get delay between corners
  if (params["delay_ms"]) {
    delay_ms_ = params["delay_ms"].as<int>();
  }

  return true;
}

bool DrawRectangleTask::execute(MotionExecutor& executor)
{
  executor.log("Starting draw_rectangle task");
  executor.log("Rectangle: width=" + std::to_string(width_) + 
               ", height=" + std::to_string(height_));

  // If we have a waypoint name, resolve it first
  if (!start_waypoint_.empty()) {
    executor.log("Moving to start waypoint: " + start_waypoint_);
    if (!executor.moveToWaypoint(start_waypoint_)) {
      executor.log("ERROR: Failed to move to start waypoint");
      return false;
    }
    // Get current joints as the starting position
    start_joints_ = executor.getCurrentJoints();
  }

  // Draw rectangle by moving through 4 corners
  // Corner order: 0 (start) -> 1 (+width) -> 2 (+width, +height) -> 3 (+height) -> 0 (return)
  executor.log("Drawing rectangle with 4 corners...");

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

  executor.log("draw_rectangle task completed successfully");
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

}  // namespace ar_projects
