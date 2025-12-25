#ifndef AR_PROJECTS__PROJECT_MANAGER_H_
#define AR_PROJECTS__PROJECT_MANAGER_H_

#include <string>
#include <vector>
#include <map>
#include <filesystem>
#include "ar_projects/yaml_parser.h"
#include "ar_projects/xml_generator.h"

namespace ar_projects
{

/**
 * @brief Project information for listing
 */
struct ProjectInfo
{
  std::string name;
  std::string description;
  std::string filepath;
};

/**
 * @brief Manage project configurations and XML generation
 */
class ProjectManager
{
public:
  /**
   * @brief Constructor
   * @param projects_dir Directory containing project YAML files
   */
  explicit ProjectManager(const std::string& projects_dir = "");
  ~ProjectManager() = default;

  /**
   * @brief Set projects directory
   */
  void setProjectsDirectory(const std::string& dir);

  /**
   * @brief Get projects directory
   */
  const std::string& getProjectsDirectory() const { return projects_dir_; }

  /**
   * @brief List all available projects
   * @return List of project info
   */
  std::vector<ProjectInfo> listProjects();

  /**
   * @brief Load a project configuration
   * @param project_name Project name (without .yaml extension)
   * @return Project configuration
   */
  ProjectConfig loadProject(const std::string& project_name);

  /**
   * @brief Generate XML for a project
   * @param project_name Project name
   * @return Generated XML string
   */
  std::string generateXml(const std::string& project_name);

  /**
   * @brief Generate and save XML for a project
   * @param project_name Project name
   * @param output_path Output file path (optional, defaults to trees/[project_name].xml)
   * @return Output file path if successful, empty string on failure
   */
  std::string generateAndSave(const std::string& project_name, 
                               const std::string& output_path = "");

  /**
   * @brief Get the default output directory for generated XML
   */
  std::string getDefaultOutputDirectory() const;

  /**
   * @brief Get last error message
   */
  const std::string& getError() const { return error_; }

private:
  /**
   * @brief Find project file by name
   */
  std::string findProjectFile(const std::string& project_name);

  std::string projects_dir_;
  std::string error_;
  YamlParser parser_;
  XmlGenerator generator_;
};

}  // namespace ar_projects

#endif  // AR_PROJECTS__PROJECT_MANAGER_H_
