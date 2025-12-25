#include "ar_projects/motion_types/move_joint.h"
#include <sstream>

namespace ar_projects
{

// Register this motion type
REGISTER_MOTION_TYPE("move_joint", MoveJoint);

MoveJoint::MoveJoint()
  : BaseMotion("move_joint")
{
}

std::string MoveJoint::toXml(int ind) const
{
  std::ostringstream xml;
  xml << indent(ind) << "<MoveToJoint target_joints=\"" << jointsToString(joints_) << "\"";
  if (!name_.empty()) {
    xml << " name=\"" << name_ << "\"";
  }
  xml << "/>\n";
  return xml.str();
}

bool MoveJoint::fromYaml(const YAML::Node& node)
{
  // Get name if specified
  if (node["name"]) {
    name_ = node["name"].as<std::string>();
  }

  // Check for waypoint reference
  if (node["waypoint"]) {
    waypoint_ = node["waypoint"].as<std::string>();
    // Joints will be resolved later by resolveWaypoint()
    return true;
  }

  // Otherwise, get joints directly
  if (node["joints"]) {
    joints_.clear();
    for (const auto& j : node["joints"]) {
      joints_.push_back(j.as<double>());
    }
    return true;
  }

  return false;
}

bool MoveJoint::resolveWaypoint(const std::map<std::string, std::vector<double>>& waypoints)
{
  if (waypoint_.empty()) {
    return true; // No waypoint to resolve
  }

  auto it = waypoints.find(waypoint_);
  if (it != waypoints.end()) {
    joints_ = it->second;
    return true;
  }

  return false; // Waypoint not found
}

}  // namespace ar_projects
