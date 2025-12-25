#ifndef AR_PROJECTS__MOTION_TYPES__CIRCLE_MOTION_H_
#define AR_PROJECTS__MOTION_TYPES__CIRCLE_MOTION_H_

#include "ar_projects/motion_types/base_motion.h"
#include <array>

namespace ar_projects
{

/**
 * @brief Circle motion - generates waypoints along a circular path
 * 
 * YAML format:
 *   - type: "circle"
 *     center: [0.4, 0.0, 0.3]      # Center position in Cartesian space
 *     radius: 0.1                   # Circle radius
 *     points: 8                     # Number of waypoints
 *     plane: "XY"                   # Plane of motion (XY, XZ, YZ)
 *     name: "DrawCircle"
 * 
 * Generates multiple MoveToJoint commands following the circle
 * Note: This requires IK to convert Cartesian points to joint space
 *       For now, it generates placeholder joint positions
 */
class CircleMotion : public BaseMotion
{
public:
  CircleMotion();
  
  std::string toXml(int indent = 0) const override;
  bool fromYaml(const YAML::Node& node) override;

  void setCenter(const std::array<double, 3>& center) { center_ = center; }
  void setRadius(double radius) { radius_ = radius; }
  void setPoints(int points) { points_ = points; }
  void setPlane(const std::string& plane) { plane_ = plane; }
  
  const std::array<double, 3>& getCenter() const { return center_; }
  double getRadius() const { return radius_; }
  int getPoints() const { return points_; }
  const std::string& getPlane() const { return plane_; }

  /**
   * @brief Set joint positions for each circle point
   * Must be set externally after IK computation
   */
  void setCircleJoints(const std::vector<std::vector<double>>& joints) { circle_joints_ = joints; }

private:
  std::array<double, 3> center_ = {0.4, 0.0, 0.3};
  double radius_ = 0.1;
  int points_ = 8;
  std::string plane_ = "XY";
  
  // Computed joint positions for each point on the circle
  std::vector<std::vector<double>> circle_joints_;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__MOTION_TYPES__CIRCLE_MOTION_H_
