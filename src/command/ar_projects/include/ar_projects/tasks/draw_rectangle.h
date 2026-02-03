#ifndef AR_PROJECTS__TASKS__DRAW_RECTANGLE_H_
#define AR_PROJECTS__TASKS__DRAW_RECTANGLE_H_

#include "ar_projects/tasks/task_base.h"
#include <vector>

namespace ar_projects
{

/**
 * @brief Task to draw a rectangle using robot motions
 * 
 * This demonstrates code-based tasks with loops. Given a starting
 * joint configuration, width and height offsets, it moves through
 * 4 corners of a rectangle.
 * 
 * YAML config:
 *   task: "draw_rectangle"
 *   task_params:
 *     start_waypoint: "corner1"     # or start_joints: [...]
 *     width: 0.1                     # X offset in joint space
 *     height: 0.05                   # Y offset in joint space
 *     joint_index_x: 0               # Which joint to modify for X
 *     joint_index_y: 1               # Which joint to modify for Y
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
   *     compute joint offsets based on corner position
   *     moveJoint to corner
   */
  bool execute(MotionExecutor& executor) override;

private:
  /**
   * @brief Compute corner joint values
   * @param corner Corner index (0-3)
   * @return Joint values for that corner
   */
  std::vector<double> computeCorner(int corner) const;

  // Configuration
  std::vector<double> start_joints_;
  std::string start_waypoint_;
  double width_ = 0.1;
  double height_ = 0.05;
  int joint_index_x_ = 0;  // Joint to modify for X movement
  int joint_index_y_ = 1;  // Joint to modify for Y movement
  int delay_ms_ = 200;     // Delay between corners
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__TASKS__DRAW_RECTANGLE_H_
