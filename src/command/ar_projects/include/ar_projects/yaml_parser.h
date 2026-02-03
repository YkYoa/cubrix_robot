#ifndef AR_PROJECTS__YAML_PARSER_H_
#define AR_PROJECTS__YAML_PARSER_H_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "ar_projects/motion_types/base_motion.h"

namespace ar_projects
{

/**
 * @brief Project configuration parsed from YAML
 */
struct ProjectConfig
{
  std::string name;
  std::string description;
  
  // Default settings
  double velocity_scaling = 0.5;
  double acceleration_scaling = 0.5;
  std::string default_pipeline = "pilz";
  std::string default_planner = "PTP";
  
  // Named waypoints (name -> joint values)
  std::map<std::string, std::vector<double>> waypoints;
  
  // Motion sequence (YAML-defined motions)
  std::vector<std::shared_ptr<BaseMotion>> motions;
  
  // Code-based task (optional)
  std::string task_name;              // Task identifier (e.g., "draw_rectangle")
  YAML::Node task_params;             // Parameters passed to task.configure()
  
  // Optional: extends another project config
  std::string extends;
  
  /**
   * @brief Check if project has a code-based task
   */
  bool hasTask() const { return !task_name.empty(); }
};

/**
 * @brief Parse YAML project configuration files
 */
class YamlParser
{
public:
  YamlParser();
  ~YamlParser() = default;

  /**
   * @brief Parse a project configuration from file
   * @param filepath Path to YAML file
   * @return Parsed project config
   */
  ProjectConfig parse(const std::string& filepath);

  /**
   * @brief Parse project configuration from string
   * @param yaml_content YAML content as string
   * @return Parsed project config
   */
  ProjectConfig parseString(const std::string& yaml_content);

  /**
   * @brief Validate a YAML configuration
   * @param filepath Path to YAML file
   * @return List of validation errors (empty if valid)
   */
  std::vector<std::string> validate(const std::string& filepath);

  /**
   * @brief Get last parse error
   */
  const std::string& getError() const { return error_; }

private:
  /**
   * @brief Parse project metadata
   */
  void parseProject(const YAML::Node& node, ProjectConfig& config);

  /**
   * @brief Parse default settings
   */
  void parseDefaults(const YAML::Node& node, ProjectConfig& config);

  /**
   * @brief Parse waypoint definitions
   */
  void parseWaypoints(const YAML::Node& node, ProjectConfig& config);

  /**
   * @brief Parse motion sequence
   */
  void parseSequence(const YAML::Node& node, ProjectConfig& config);

  /**
   * @brief Resolve waypoint references in motions
   */
  void resolveWaypoints(ProjectConfig& config);

  std::string error_;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__YAML_PARSER_H_
