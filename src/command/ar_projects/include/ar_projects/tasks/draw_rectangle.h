#ifndef AR_PROJECTS__TASKS__DRAW_RECTANGLE_H_
#define AR_PROJECTS__TASKS__DRAW_RECTANGLE_H_

#include "ar_projects/tasks/task_base.h"
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace ar_projects
{

/**
 * @brief Task to draw a rectangle using robot motions
 * 
 * This demonstrates code-based tasks with loops. It supports two modes:
 * 
 * 1. Joint Space Mode (default): Moves through corners by modifying
 *    specified joint values.
 * 
 * 2. Cartesian Mode (use_cartesian: true): Uses end effector positioning
 *    to draw in the XY plane, keeping Z and orientation constant.
 * 
 * YAML config:
 *   task: "draw_rectangle"
 *   task_params:
 *     start_waypoint: "corner1"     # Optional starting waypoint
 *     width: 0.1                    # X size (meters for Cartesian, rad for joint)
 *     height: 0.05                  # Y size (meters for Cartesian, rad for joint)
 *     use_cartesian: true           # Use end effector positioning
 *     delay_ms: 200                 # Delay between corners
 *     # Joint mode only:
 *     joint_index_x: 0              # Which joint to modify for X
 *     joint_index_y: 1              # Which joint to modify for Y
 */
class DrawRectangleTask : public TaskBase
{
public:
  DrawRectangleTask();

  std::string name() const override { return "draw_rectangle"; }
  
  std::string description() const override 
  { 
    return "Draw a rectangle by moving through 4 corners"; 
  }

  /**
   * @brief Configure from YAML task_params
   */
  bool configure(const YAML::Node& params) override;

  /**
   * @brief Execute the rectangle drawing
   * 
   * Loop logic:
   *   for each corner (0-3):
   *     compute position based on corner
   *     move to corner (Cartesian or joint space)
   */
  bool execute(MotionExecutor& executor) override;

private:
  /**
   * @brief Execute using Cartesian (end effector) positioning
   */
  bool executeCartesian(MotionExecutor& executor);

  /**
   * @brief Execute using joint space control
   */
  bool executeJointSpace(MotionExecutor& executor);

  /**
   * @brief Compute corner joint values (joint mode only)
   * @param corner Corner index (0-3)
   * @return Joint values for that corner
   */
  std::vector<double> computeCorner(int corner) const;

  /**
   * @brief Convert quaternion to roll, pitch, yaw
   */
  void quaternionToRPY(const geometry_msgs::msg::Quaternion& q,
                       double& roll, double& pitch, double& yaw) const;

  // Configuration
  std::vector<double> start_joints_;
  std::string start_waypoint_;
  double width_ = 0.1;
  double height_ = 0.05;
  int joint_index_x_ = 0;   // Joint to modify for X movement (joint mode)
  int joint_index_y_ = 1;   // Joint to modify for Y movement (joint mode)
  int delay_ms_ = 200;      // Delay between corners
  bool use_cartesian_ = false;  // Use end effector positioning
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__TASKS__DRAW_RECTANGLE_H_
