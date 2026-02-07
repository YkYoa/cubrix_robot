#ifndef AR_PROJECTS__MOTION_EXECUTOR_H_
#define AR_PROJECTS__MOTION_EXECUTOR_H_

#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

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
   * @brief Move end effector to specified pose
   * @param eef_pose Target pose for end effector
   * @param end_effector_link Optional specific end effector link (empty = use default)
   * @return true if motion succeeded
   */
  bool moveJoint(const geometry_msgs::msg::Pose& eef_pose, 
                 const std::string& end_effector_link = "");

  /**
   * @brief Move end effector to specified position and orientation
   * @param x X position in meters
   * @param y Y position in meters
   * @param z Z position in meters
   * @param roll Roll angle in radians
   * @param pitch Pitch angle in radians
   * @param yaw Yaw angle in radians
   * @param end_effector_link Optional specific end effector link (empty = use default)
   * @return true if motion succeeded
   */
  bool moveJoint(double x, double y, double z, 
                 double roll, double pitch, double yaw,
                 const std::string& end_effector_link = "");

  /**
   * @brief Move end effector to specified position (keeping current orientation)
   * @param x X position in meters
   * @param y Y position in meters
   * @param z Z position in meters
   * @param end_effector_link Optional specific end effector link (empty = use default)
   * @return true if motion succeeded
   */
  bool moveJointPosition(double x, double y, double z,
                         const std::string& end_effector_link = "");

  /**
   * @brief Get current end effector pose
   * @return Current pose of the end effector
   */
  geometry_msgs::msg::Pose getCurrentPose() const;

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
   * @brief Enable or disable plan caching
   * @param enabled True to enable caching
   */
  void setPlanCaching(bool enabled);

  /**
   * @brief Set the directory for plan caching
   * @param dir Directory path
   */
  void setPlanCacheDir(const std::string& dir);

  /**
   * @brief Clear all cached plans from the configured cache directory
   */
  void clearPlanCache();

  /**
   * @brief Set a callback for logging messages
   * @param callback Function to call with log messages
   */
  void setLogCallback(std::function<void(const std::string&)> callback);

  /**
   * @brief Stop the current motion immediately
   */
  void stop();

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
