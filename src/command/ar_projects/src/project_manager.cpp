#include "ar_projects/project_manager.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

namespace ar_projects
{

ProjectManager::ProjectManager(const std::string& projects_dir)
  : projects_dir_(projects_dir)
{
  if (projects_dir_.empty()) {
    // Default to package share directory
    try {
      projects_dir_ = ament_index_cpp::get_package_share_directory("ar_projects") + "/projects";
    } catch (const std::exception&) {
      // Package not found, will be set later
    }
  }
}

void ProjectManager::setProjectsDirectory(const std::string& dir)
{
  projects_dir_ = dir;
}

std::vector<ProjectInfo> ProjectManager::listProjects()
{
  std::vector<ProjectInfo> projects;
  error_.clear();

  if (projects_dir_.empty() || !std::filesystem::exists(projects_dir_)) {
    error_ = "Projects directory not found: " + projects_dir_;
    return projects;
  }

  for (const auto& entry : std::filesystem::directory_iterator(projects_dir_)) {
    ProjectInfo info;
    std::string config_path;

    // Check if it's a directory with config.yaml inside (new structure)
    if (entry.is_directory()) {
      std::string dir_config = entry.path().string() + "/config.yaml";
      if (std::filesystem::exists(dir_config)) {
        info.filepath = dir_config;
        info.name = entry.path().filename().string();
        config_path = dir_config;
      } else {
        // Try config.yml
        dir_config = entry.path().string() + "/config.yml";
        if (std::filesystem::exists(dir_config)) {
          info.filepath = dir_config;
          info.name = entry.path().filename().string();
          config_path = dir_config;
        } else {
          continue; // Not a valid project directory
        }
      }
    }
    // Check if it's a yaml file directly (old structure)
    else if (entry.path().extension() == ".yaml" || entry.path().extension() == ".yml") {
      info.filepath = entry.path().string();
      info.name = entry.path().stem().string();
      config_path = entry.path().string();
    } else {
      continue;
    }

    // Try to read description from config file
    try {
      YAML::Node root = YAML::LoadFile(config_path);
      if (root["project"] && root["project"]["description"]) {
        info.description = root["project"]["description"].as<std::string>();
      }
    } catch (...) {
      // Ignore parse errors for listing
    }

    projects.push_back(info);
  }

  return projects;
}

ProjectConfig ProjectManager::loadProject(const std::string& project_name)
{
  error_.clear();
  
  std::string filepath = findProjectFile(project_name);
  if (filepath.empty()) {
    error_ = "Project not found: " + project_name;
    return ProjectConfig();
  }

  auto config = parser_.parse(filepath);
  if (!parser_.getError().empty()) {
    error_ = parser_.getError();
  }

  return config;
}

std::string ProjectManager::generateXml(const std::string& project_name)
{
  error_.clear();

  auto config = loadProject(project_name);
  if (!error_.empty()) {
    return "";
  }

  return generator_.generate(config);
}

std::string ProjectManager::generateAndSave(const std::string& project_name, 
                                            const std::string& output_path)
{
  error_.clear();

  auto config = loadProject(project_name);
  if (!error_.empty()) {
    return "";
  }

  std::string out_file = output_path;
  if (out_file.empty()) {
    out_file = getDefaultOutputDirectory() + "/" + project_name + ".xml";
  }

  // Create output directory if it doesn't exist
  std::filesystem::path out_path(out_file);
  if (!std::filesystem::exists(out_path.parent_path())) {
    std::filesystem::create_directories(out_path.parent_path());
  }

  if (generator_.writeToFile(config, out_file)) {
    return out_file;
  } else {
    error_ = generator_.getError();
    return "";
  }
}

std::string ProjectManager::getDefaultOutputDirectory() const
{
  try {
    return ament_index_cpp::get_package_share_directory("ar_bt") + "/trees";
  } catch (const std::exception&) {
    return "/tmp/ar_trees";
  }
}

std::string ProjectManager::findProjectFile(const std::string& project_name)
{
  if (projects_dir_.empty()) {
    return "";
  }

  // Check for directory-based project (new structure: projects/project_name/config.yaml)
  std::string dir_path = projects_dir_ + "/" + project_name;
  if (std::filesystem::is_directory(dir_path)) {
    std::string config_yaml = dir_path + "/config.yaml";
    if (std::filesystem::exists(config_yaml)) {
      return config_yaml;
    }
    std::string config_yml = dir_path + "/config.yml";
    if (std::filesystem::exists(config_yml)) {
      return config_yml;
    }
  }

  // Check with .yaml extension (old structure)
  std::string yaml_path = projects_dir_ + "/" + project_name + ".yaml";
  if (std::filesystem::exists(yaml_path)) {
    return yaml_path;
  }

  // Check with .yml extension
  std::string yml_path = projects_dir_ + "/" + project_name + ".yml";
  if (std::filesystem::exists(yml_path)) {
    return yml_path;
  }

  // Check if it's already a full path
  if (std::filesystem::exists(project_name)) {
    return project_name;
  }

  return "";
}

}  // namespace ar_projects
