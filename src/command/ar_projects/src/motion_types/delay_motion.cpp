#include "ar_projects/motion_types/delay_motion.h"
#include <sstream>

namespace ar_projects
{

// Register this motion type
REGISTER_MOTION_TYPE("delay", DelayMotion);

DelayMotion::DelayMotion()
  : BaseMotion("delay")
{
}

std::string DelayMotion::toXml(int ind) const
{
  std::ostringstream xml;
  xml << indent(ind) << "<Delay delay_msec=\"" << duration_ms_ << "\">\n";
  xml << indent(ind + 1) << "<AlwaysSuccess/>\n";
  xml << indent(ind) << "</Delay>\n";
  return xml.str();
}

bool DelayMotion::fromYaml(const YAML::Node& node)
{
  if (node["name"]) {
    name_ = node["name"].as<std::string>();
  }

  if (node["duration_ms"]) {
    duration_ms_ = node["duration_ms"].as<int>();
    return true;
  }

  // Default duration if not specified
  return true;
}

}  // namespace ar_projects
