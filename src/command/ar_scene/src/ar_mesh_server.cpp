#include "ar_scene/ar_mesh_server.h"
#include <filesystem>
#include <fstream>

namespace ar_scene
{

MeshServer::MeshServer()
: logger_(rclcpp::get_logger("ar_mesh_server"))
{
}

MeshServer::~MeshServer()
{
}

bool MeshServer::registerMesh(const std::string & name, const std::string & path)
{
  if (mesh_registry_.find(name) != mesh_registry_.end()) {
    RCLCPP_WARN(logger_, "Mesh '%s' already registered. Overwriting.", name.c_str());
  }
  mesh_registry_[name] = path;
  RCLCPP_INFO(logger_, "Registered mesh '%s': %s", name.c_str(), path.c_str());
  return true;
}

std::string MeshServer::getMeshPath(const std::string & name) const
{
  auto it = mesh_registry_.find(name);
  if (it != mesh_registry_.end()) {
    return it->second;
  }
  RCLCPP_ERROR(logger_, "Mesh '%s' not found in registry.", name.c_str());
  return "";
}

bool MeshServer::loadMesh(const std::string & path)
{
  // Basic check if file exists
  std::string actual_path = path;
  
  // Handle package:// prefix if necessary - though typical ROS mesh loaders handle this.
  // We can use ament_index_cpp if we need to resolve package paths strictly, 
  // but often Assimp or MoveIt loaders handle URL-style paths.
  // For this simple check, we might just rely on filesystem if it's an absolute path.
  
  if (path.find("package://") == 0) {
    // Naive resolution for existence check
    std::string package_prefix = "package://";
    std::string rest = path.substr(package_prefix.length());
    size_t split = rest.find('/');
    if (split != std::string::npos) {
      std::string pkg_name = rest.substr(0, split);
      std::string rel_path = rest.substr(split + 1);
      try {
        std::string pkg_dir = ament_index_cpp::get_package_share_directory(pkg_name);
        actual_path = pkg_dir + "/" + rel_path;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Failed to resolve package path for '%s': %s", path.c_str(), e.what());
        return false;
      }
    }
  }

  if (std::filesystem::exists(actual_path)) {
    return true;
  }

  RCLCPP_ERROR(logger_, "Mesh file not found: %s", actual_path.c_str());
  return false;
}

} // namespace ar_scene
