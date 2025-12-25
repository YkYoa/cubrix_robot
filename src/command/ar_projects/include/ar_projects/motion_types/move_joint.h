#ifndef AR_PROJECTS__MOTION_TYPES__MOVE_JOINT_H_
#define AR_PROJECTS__MOTION_TYPES__MOVE_JOINT_H_

#include "ar_projects/motion_types/base_motion.h"

namespace ar_projects
{

/**
 * @brief Move to joint configuration
 * 
 * YAML format:
 *   - type: "move_joint"
 *     joints: [0.0, 0.5, -0.3, 0, 0, 0]
 *     name: "MoveToPosition"
 * 
 * Or with waypoint reference:
 *   - type: "move_joint"
 *     waypoint: "home"
 *     name: "GoHome"
 */
class MoveJoint : public BaseMotion
{
public:
  MoveJoint();
  
  std::string toXml(int indent = 0) const override;
  bool fromYaml(const YAML::Node& node) override;

  void setJoints(const std::vector<double>& joints) { joints_ = joints; }
  void setWaypoint(const std::string& waypoint) { waypoint_ = waypoint; }
  
  const std::vector<double>& getJoints() const { return joints_; }
  const std::string& getWaypoint() const { return waypoint_; }

  /**
   * @brief Resolve waypoint to actual joint values
   * @param waypoints Map of waypoint names to joint values
   */
  bool resolveWaypoint(const std::map<std::string, std::vector<double>>& waypoints);

private:
  std::vector<double> joints_;
  std::string waypoint_;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__MOVE_JOINT_H_
