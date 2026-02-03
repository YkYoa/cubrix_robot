#ifndef AR_PROJECTS__MOTION_EXECUTOR_H_
#define AR_PROJECTS__MOTION_EXECUTOR_H_

#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace ar_projects
{

/**
 * @brief Interface for executing robot motions from tasks
 * 
 * Provides a simple API for code-based tasks to command robot motions.
 * Wraps MoveIt2 action clients internally.
 * 
 * Example usage:
 *   bool DrawRectangleTask::execute(MotionExecutor& executor) {
 *     for (int i = 0; i < 4; i++) {
 *       executor.moveJoint(corners[i]);
 *     }
 *     return true;
 *   }
 */
class MotionExecutor
{
public:
  /**
   * @brief Constructor
   * @param node ROS2 node for action clients
   * @param planning_group MoveIt planning group (default: "Arm")
   */
  explicit MotionExecutor(rclcpp::Node::SharedPtr node, 
                          const std::string& planning_group = "Arm");
  
  ~MotionExecutor();

  /**
   * @brief Move to joint configuration
   * @param joints Target joint values (6 values for 6-DOF arm)
   * @return true if motion succeeded
   */
  bool moveJoint(const std::vector<double>& joints);

  /**
   * @brief Move to named waypoint (requires waypoints map to be set)
   * @param waypoint_name Name of the waypoint
   * @return true if motion succeeded
   */
  bool moveToWaypoint(const std::string& waypoint_name);

  /**
   * @brief Wait for specified duration
   * @param ms Duration in milliseconds
   */
  void delay(int ms);

  /**
   * @brief Set velocity scaling factor
   * @param factor Velocity scaling (0.0 to 1.0)
   */
  void setVelocityScaling(double factor);

  /**
   * @brief Set acceleration scaling factor
   * @param factor Acceleration scaling (0.0 to 1.0)
   */
  void setAccelerationScaling(double factor);

  /**
   * @brief Get current joint positions
   * @return Current joint values
   */
  std::vector<double> getCurrentJoints() const;

  /**
   * @brief Set waypoints map for named waypoint lookup
   * @param waypoints Map of waypoint name to joint values
   */
  void setWaypoints(const std::map<std::string, std::vector<double>>& waypoints);

  /**
   * @brief Log message to execution log
   * @param msg Message to log
   */
  void log(const std::string& msg);

  /**
   * @brief Check if executor is ready
   * @return true if action clients are connected
   */
  bool isReady() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_EXECUTOR_H_
