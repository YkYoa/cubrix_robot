#include "ar_projects/motion_types/circle_motion.h"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace ar_projects
{

// Register this motion type
REGISTER_MOTION_TYPE("circle", CircleMotion);

CircleMotion::CircleMotion()
  : BaseMotion("circle")
{
}

std::string CircleMotion::toXml(int ind) const
{
  std::ostringstream xml;
  
  // Add comment describing the circle
  xml << indent(ind) << "<!-- Circle: center=[" << center_[0] << "," << center_[1] << "," << center_[2]
      << "], radius=" << radius_ << ", plane=" << plane_ << " -->\n";

  // If we have computed joint positions, generate MoveToJoint for each
  if (!circle_joints_.empty()) {
    for (size_t i = 0; i < circle_joints_.size(); ++i) {
      std::ostringstream jstr;
      for (size_t j = 0; j < circle_joints_[i].size(); ++j) {
        if (j > 0) jstr << ";";
        jstr << circle_joints_[i][j];
      }
      xml << indent(ind) << "<MoveToJoint target_joints=\"" << jstr.str() << "\"";
      if (!name_.empty()) {
        xml << " name=\"" << name_ << "_" << (i + 1) << "\"";
      }
      xml << "/>\n";
    }
  } else {
    // Generate placeholder waypoints based on circle parameters
    // This creates joint-space approximations (will need IK in real usage)
    xml << indent(ind) << "<!-- NOTE: Circle motion requires IK. Using placeholder joints. -->\n";
    
    for (int i = 0; i < points_; ++i) {
      double angle = 2.0 * M_PI * i / points_;
      
      // Generate simple placeholder joint values
      // In real implementation, these would come from IK
      std::ostringstream jstr;
      jstr << std::fixed << std::setprecision(4);
      jstr << (0.5 * std::cos(angle)) << ";";  // j1
      jstr << (0.3 + 0.1 * std::sin(angle)) << ";";  // j2
      jstr << "-0.5;0;0;0";  // j3-j6
      
      xml << indent(ind) << "<MoveToJoint target_joints=\"" << jstr.str() << "\"";
      if (!name_.empty()) {
        xml << " name=\"" << name_ << "_" << (i + 1) << "\"";
      }
      xml << "/>\n";
    }
  }
  
  return xml.str();
}

bool CircleMotion::fromYaml(const YAML::Node& node)
{
  if (node["name"]) {
    name_ = node["name"].as<std::string>();
  }

  if (node["center"]) {
    auto center_node = node["center"];
    if (center_node.size() >= 3) {
      center_[0] = center_node[0].as<double>();
      center_[1] = center_node[1].as<double>();
      center_[2] = center_node[2].as<double>();
    }
  }

  if (node["radius"]) {
    radius_ = node["radius"].as<double>();
  }

  if (node["points"]) {
    points_ = node["points"].as<int>();
  }

  if (node["plane"]) {
    plane_ = node["plane"].as<std::string>();
  }

  return true;
}

}  // namespace ar_projects
