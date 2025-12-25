#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>

#include "ar_projects/project_manager.h"
#include "ar_projects/yaml_parser.h"
#include "ar_projects/xml_generator.h"

// Include all motion types to ensure registration
#include "ar_projects/motion_types/move_joint.h"
#include "ar_projects/motion_types/delay_motion.h"
#include "ar_projects/motion_types/set_planner.h"
#include "ar_projects/motion_types/set_velocity.h"
#include "ar_projects/motion_types/circle_motion.h"

/**
 * @brief Project runner node
 * 
 * Usage:
 *   ros2 run ar_projects project_runner --ros-args -p project:=pick_place
 *   ros2 run ar_projects project_runner --ros-args -p config:=/path/to/config.yaml
 * 
 * This node:
 * 1. Loads the project YAML configuration
 * 2. Generates BehaviorTree XML
 * 3. Writes XML to a temporary file
 * 4. Launches the ar_bt executor with the generated tree
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ar_project_runner");

  // Declare parameters
  node->declare_parameter<std::string>("project", "");
  node->declare_parameter<std::string>("config", "");
  node->declare_parameter<std::string>("output", "");
  node->declare_parameter<bool>("generate_only", false);
  node->declare_parameter<bool>("list_projects", false);
  node->declare_parameter<bool>("list_types", false);
  node->declare_parameter<std::string>("planning_group", "Arm");

  // Get parameters
  std::string project_name = node->get_parameter("project").as_string();
  std::string config_path = node->get_parameter("config").as_string();
  std::string output_path = node->get_parameter("output").as_string();
  bool generate_only = node->get_parameter("generate_only").as_bool();
  bool list_projects = node->get_parameter("list_projects").as_bool();
  bool list_types = node->get_parameter("list_types").as_bool();
  std::string planning_group = node->get_parameter("planning_group").as_string();

  // Create project manager
  ar_projects::ProjectManager manager;

  // List motion types
  if (list_types) {
    RCLCPP_INFO(node->get_logger(), "Available motion types:");
    for (const auto& type : ar_projects::MotionRegistry::instance().getTypes()) {
      RCLCPP_INFO(node->get_logger(), "  - %s", type.c_str());
    }
    rclcpp::shutdown();
    return 0;
  }

  // List projects
  if (list_projects) {
    RCLCPP_INFO(node->get_logger(), "Available projects in %s:", manager.getProjectsDirectory().c_str());
    for (const auto& info : manager.listProjects()) {
      RCLCPP_INFO(node->get_logger(), "  - %s: %s", info.name.c_str(), info.description.c_str());
    }
    rclcpp::shutdown();
    return 0;
  }

  // Determine which config to use
  std::string yaml_file;
  if (!config_path.empty()) {
    yaml_file = config_path;
  } else if (!project_name.empty()) {
    yaml_file = manager.getProjectsDirectory() + "/" + project_name + ".yaml";
  } else {
    RCLCPP_ERROR(node->get_logger(), "Either 'project' or 'config' parameter is required!");
    RCLCPP_INFO(node->get_logger(), "Usage:");
    RCLCPP_INFO(node->get_logger(), "  ros2 run ar_projects project_runner --ros-args -p project:=<name>");
    RCLCPP_INFO(node->get_logger(), "  ros2 run ar_projects project_runner --ros-args -p config:=/path/to/config.yaml");
    RCLCPP_INFO(node->get_logger(), "  ros2 run ar_projects project_runner --ros-args -p list_projects:=true");
    RCLCPP_INFO(node->get_logger(), "  ros2 run ar_projects project_runner --ros-args -p list_types:=true");
    rclcpp::shutdown();
    return 1;
  }

  // Check if config file exists
  if (!std::filesystem::exists(yaml_file)) {
    RCLCPP_ERROR(node->get_logger(), "Config file not found: %s", yaml_file.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Loading project from: %s", yaml_file.c_str());

  // Parse YAML and generate XML
  ar_projects::YamlParser parser;
  ar_projects::XmlGenerator generator;

  auto config = parser.parse(yaml_file);
  if (!parser.getError().empty()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse config: %s", parser.getError().c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Project: %s", config.name.c_str());
  RCLCPP_INFO(node->get_logger(), "Description: %s", config.description.c_str());
  RCLCPP_INFO(node->get_logger(), "Motions: %zu", config.motions.size());

  // Generate XML
  std::string xml = generator.generate(config);
  if (xml.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to generate XML");
    rclcpp::shutdown();
    return 1;
  }

  // Determine output file
  std::string xml_file;
  if (!output_path.empty()) {
    xml_file = output_path;
  } else {
    // Create temp file
    std::string temp_dir;
    try {
      temp_dir = ament_index_cpp::get_package_share_directory("ar_bt") + "/trees";
    } catch (const std::exception&) {
      temp_dir = "/tmp";
    }
    xml_file = temp_dir + "/" + config.name + "_generated.xml";
  }

  // Write XML to file
  std::ofstream file(xml_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Could not write to: %s", xml_file.c_str());
    rclcpp::shutdown();
    return 1;
  }
  file << xml;
  file.close();

  RCLCPP_INFO(node->get_logger(), "Generated XML saved to: %s", xml_file.c_str());

  if (generate_only) {
    RCLCPP_INFO(node->get_logger(), "Generate only mode - XML written, not executing");
    RCLCPP_INFO(node->get_logger(), "\nGenerated XML:\n%s", xml.c_str());
    rclcpp::shutdown();
    return 0;
  }

  // Execute the behavior tree
  RCLCPP_INFO(node->get_logger(), "============================================");
  RCLCPP_INFO(node->get_logger(), "To execute this tree, run:");
  RCLCPP_INFO(node->get_logger(), "  ros2 run ar_bt bt_executor_node --ros-args -p tree_file:=%s -p planning_group:=%s", 
              xml_file.c_str(), planning_group.c_str());
  RCLCPP_INFO(node->get_logger(), "============================================");

  // Print the generated XML for debugging
  RCLCPP_INFO(node->get_logger(), "\nGenerated BehaviorTree XML:");
  RCLCPP_INFO(node->get_logger(), "--------------------------------------------");
  std::cout << xml << std::endl;
  RCLCPP_INFO(node->get_logger(), "--------------------------------------------");

  rclcpp::shutdown();
  return 0;
}
