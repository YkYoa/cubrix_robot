#include "ar_projects/motion_types/set_velocity.h"
#include <sstream>

namespace ar_projects
{

// Register motion types
REGISTER_MOTION_TYPE("set_velocity", SetVelocityMotion);
REGISTER_MOTION_TYPE("set_acceleration", SetAccelerationMotion);

// SetVelocityMotion

SetVelocityMotion::SetVelocityMotion()
  : BaseMotion("set_velocity")
{
}

std::string SetVelocityMotion::toXml(int ind) const
{
  std::ostringstream xml;
  xml << indent(ind) << "<SetVelocityScaling factor=\"" << factor_ << "\"/>\n";
  return xml.str();
}

bool SetVelocityMotion::fromYaml(const YAML::Node& node)
{
  if (node["factor"]) {
    factor_ = node["factor"].as<double>();
  }
  return true;
}

// SetAccelerationMotion

SetAccelerationMotion::SetAccelerationMotion()
  : BaseMotion("set_acceleration")
{
}

std::string SetAccelerationMotion::toXml(int ind) const
{
  std::ostringstream xml;
  xml << indent(ind) << "<SetAccelerationScaling factor=\"" << factor_ << "\"/>\n";
  return xml.str();
}

bool SetAccelerationMotion::fromYaml(const YAML::Node& node)
{
  if (node["factor"]) {
    factor_ = node["factor"].as<double>();
  }
  return true;
}

}  // namespace ar_projects
