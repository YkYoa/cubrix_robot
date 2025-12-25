#include "ar_projects/motion_types/set_planner.h"
#include <sstream>

namespace ar_projects
{

// Register this motion type
REGISTER_MOTION_TYPE("set_planner", SetPlannerMotion);

SetPlannerMotion::SetPlannerMotion()
  : BaseMotion("set_planner")
{
}

std::string SetPlannerMotion::toXml(int ind) const
{
  std::ostringstream xml;
  xml << indent(ind) << "<SetPlanner pipeline=\"" << pipeline_ 
      << "\" planner=\"" << planner_id_ << "\"/>\n";
  return xml.str();
}

bool SetPlannerMotion::fromYaml(const YAML::Node& node)
{
  if (node["pipeline"]) {
    pipeline_ = node["pipeline"].as<std::string>();
  }
  if (node["planner_id"]) {
    planner_id_ = node["planner_id"].as<std::string>();
  } else if (node["planner"]) {
    planner_id_ = node["planner"].as<std::string>();
  }
  return true;
}

}  // namespace ar_projects
