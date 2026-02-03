#include "ar_projects/yaml_parser.h"
#include "ar_projects/motion_types/move_joint.h"
#include <fstream>
#include <sstream>
#include <set>

namespace ar_projects
{

YamlParser::YamlParser()
{
}

ProjectConfig YamlParser::parse(const std::string& filepath)
{
  ProjectConfig config;
  error_.clear();

  try {
    YAML::Node root = YAML::LoadFile(filepath);
    
    // Parse sections
    if (root["project"]) {
      parseProject(root["project"], config);
    }
    if (root["defaults"]) {
      parseDefaults(root["defaults"], config);
    }
    if (root["waypoints"]) {
      parseWaypoints(root["waypoints"], config);
    }
    if (root["sequence"]) {
      parseSequence(root["sequence"], config);
    }
    
    // Resolve waypoint references
    resolveWaypoints(config);
    
  } catch (const YAML::Exception& e) {
    error_ = "YAML parse error: " + std::string(e.what());
  } catch (const std::exception& e) {
    error_ = "Error: " + std::string(e.what());
  }

  return config;
}

ProjectConfig YamlParser::parseString(const std::string& yaml_content)
{
  ProjectConfig config;
  error_.clear();

  try {
    YAML::Node root = YAML::Load(yaml_content);
    
    if (root["project"]) {
      parseProject(root["project"], config);
    }
    if (root["defaults"]) {
      parseDefaults(root["defaults"], config);
    }
    if (root["waypoints"]) {
      parseWaypoints(root["waypoints"], config);
    }
    if (root["sequence"]) {
      parseSequence(root["sequence"], config);
    }
    
    resolveWaypoints(config);
    
  } catch (const YAML::Exception& e) {
    error_ = "YAML parse error: " + std::string(e.what());
  }

  return config;
}

std::vector<std::string> YamlParser::validate(const std::string& filepath)
{
  std::vector<std::string> errors;

  try {
    YAML::Node root = YAML::LoadFile(filepath);

    // Check required sections
    if (!root["project"]) {
      errors.push_back("Missing 'project' section");
    } else {
      if (!root["project"]["name"]) {
        errors.push_back("Missing 'project.name'");
      }
    }

    if (!root["sequence"]) {
      errors.push_back("Missing 'sequence' section");
    } else if (!root["sequence"].IsSequence()) {
      errors.push_back("'sequence' must be a list");
    } else {
      // Validate each motion in sequence
      int idx = 0;
      for (const auto& motion : root["sequence"]) {
        if (!motion["type"]) {
          errors.push_back("Motion #" + std::to_string(idx) + ": missing 'type'");
        } else {
          std::string type = motion["type"].as<std::string>();
          if (!MotionRegistry::instance().create(type)) {
            errors.push_back("Motion #" + std::to_string(idx) + ": unknown type '" + type + "'");
          }
        }
        idx++;
      }
    }

    // Validate waypoint references
    if (root["sequence"] && root["sequence"].IsSequence()) {
      std::set<std::string> defined_waypoints;
      if (root["waypoints"]) {
        for (const auto& wp : root["waypoints"]) {
          defined_waypoints.insert(wp.first.as<std::string>());
        }
      }

      int idx = 0;
      for (const auto& motion : root["sequence"]) {
        if (motion["waypoint"]) {
          std::string wp = motion["waypoint"].as<std::string>();
          if (defined_waypoints.find(wp) == defined_waypoints.end()) {
            errors.push_back("Motion #" + std::to_string(idx) + ": undefined waypoint '" + wp + "'");
          }
        }
        idx++;
      }
    }

  } catch (const YAML::Exception& e) {
    errors.push_back("YAML parse error: " + std::string(e.what()));
  }

  return errors;
}

void YamlParser::parseProject(const YAML::Node& node, ProjectConfig& config)
{
  if (node["name"]) {
    config.name = node["name"].as<std::string>();
  }
  if (node["description"]) {
    config.description = node["description"].as<std::string>();
  }
  if (node["extends"]) {
    config.extends = node["extends"].as<std::string>();
  }
  
  // Parse code-based task reference
  if (node["task"]) {
    config.task_name = node["task"].as<std::string>();
  }
  if (node["task_params"]) {
    config.task_params = node["task_params"];
  }
}

void YamlParser::parseDefaults(const YAML::Node& node, ProjectConfig& config)
{
  if (node["velocity_scaling"]) {
    config.velocity_scaling = node["velocity_scaling"].as<double>();
  }
  if (node["acceleration_scaling"]) {
    config.acceleration_scaling = node["acceleration_scaling"].as<double>();
  }
  if (node["planner"]) {
    if (node["planner"]["pipeline"]) {
      config.default_pipeline = node["planner"]["pipeline"].as<std::string>();
    }
    if (node["planner"]["planner_id"]) {
      config.default_planner = node["planner"]["planner_id"].as<std::string>();
    }
  }
}

void YamlParser::parseWaypoints(const YAML::Node& node, ProjectConfig& config)
{
  for (const auto& wp : node) {
    std::string name = wp.first.as<std::string>();
    std::vector<double> joints;

    if (wp.second["joints"]) {
      for (const auto& j : wp.second["joints"]) {
        joints.push_back(j.as<double>());
      }
    }

    config.waypoints[name] = joints;
  }
}

void YamlParser::parseSequence(const YAML::Node& node, ProjectConfig& config)
{
  for (const auto& motion_node : node) {
    if (!motion_node["type"]) {
      continue;
    }

    std::string type = motion_node["type"].as<std::string>();
    auto motion = MotionRegistry::instance().create(type);
    
    if (motion) {
      if (motion->fromYaml(motion_node)) {
        config.motions.push_back(motion);
      }
    }
  }
}

void YamlParser::resolveWaypoints(ProjectConfig& config)
{
  for (auto& motion : config.motions) {
    // Check if this is a MoveJoint that needs waypoint resolution
    auto move_joint = std::dynamic_pointer_cast<MoveJoint>(motion);
    if (move_joint && !move_joint->getWaypoint().empty()) {
      move_joint->resolveWaypoint(config.waypoints);
    }
  }
}

}  // namespace ar_projects
