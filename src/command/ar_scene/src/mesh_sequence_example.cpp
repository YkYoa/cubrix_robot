#include <rclcpp/rclcpp.hpp>
#include "ar_scene/ar_scene_manager.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief Example node demonstrating how to use SceneManager to add meshes
 * 
 * This example:
 * 1. Initializes the static SceneManager singleton
 * 2. Loads all meshes from ar_moveit_config/meshes/obj/
 * 3. Adds each mesh to the planning scene at different positions
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mesh_sequence_example");
  
  RCLCPP_INFO(node->get_logger(), "Starting mesh sequence example...");
  
  // Initialize the static SceneManager singleton
  ar_scene::SceneManager::initialize(node);
  auto& scene = ar_scene::SceneManager::getInstance();
  
  // Load all meshes from config package
  if (!scene.loadMeshesFromConfig()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load meshes from config");
    rclcpp::shutdown();
    return 1;
  }
  
  // Get list of available meshes
  auto mesh_names = scene.getRegisteredMeshes();
  RCLCPP_INFO(node->get_logger(), "Found %zu meshes:", mesh_names.size());
  for (const auto& name : mesh_names) {
    RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
  }
  
  // Create poses for each mesh - arranged in a row
  double x_offset = 0.0;
  const double spacing = 0.15;  // 15cm between objects
  
  for (const auto& mesh_name : mesh_names) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = node->get_clock()->now();
    
    // Position objects in a line along X axis
    pose.pose.position.x = 0.5 + x_offset;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.1;  // 10cm above ground
    pose.pose.orientation.w = 1.0;
    
    // Add to scene with name like "obj_THREE_D_B"
    std::string object_name = "obj_" + mesh_name;
    
    if (scene.addMeshObj(object_name, mesh_name, pose)) {
      RCLCPP_INFO(node->get_logger(), "Added '%s' at x=%.2f", object_name.c_str(), pose.pose.position.x);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to add '%s'", object_name.c_str());
    }
    
    x_offset += spacing;
    
    // Small delay between additions
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  
  RCLCPP_INFO(node->get_logger(), "Mesh sequence complete! Added %zu objects to scene.", mesh_names.size());
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit and remove objects...");
  
  // Spin until shutdown
  rclcpp::spin(node);
  
  // Cleanup - remove all added objects
  RCLCPP_INFO(node->get_logger(), "Cleaning up...");
  for (const auto& mesh_name : mesh_names) {
    std::string object_name = "obj_" + mesh_name;
    scene.removeMeshObj(object_name);
  }
  
  rclcpp::shutdown();
  return 0;
}
