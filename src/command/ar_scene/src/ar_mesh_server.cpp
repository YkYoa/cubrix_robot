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

int MeshServer::registerMeshDirectory(const std::string & package_name, const std::string & relative_path)
{
  int count = 0;
  
  try {
    std::string pkg_dir = ament_index_cpp::get_package_share_directory(package_name);
    std::string full_path = pkg_dir + "/" + relative_path;
    
    if (!std::filesystem::exists(full_path)) {
      RCLCPP_ERROR(logger_, "Directory does not exist: %s", full_path.c_str());
      return 0;
    }
    
    for (const auto& entry : std::filesystem::directory_iterator(full_path)) {
      if (entry.is_regular_file()) {
        std::string ext = entry.path().extension().string();
        // Support STL and OBJ mesh formats
        if (ext == ".stl" || ext == ".STL" || ext == ".obj" || ext == ".OBJ" || ext == ".dae" || ext == ".DAE") {
          std::string mesh_name = entry.path().stem().string();
          std::string mesh_path = entry.path().string();
          registerMesh(mesh_name, mesh_path);
          count++;
        }
      }
    }
    
    RCLCPP_INFO(logger_, "Registered %d meshes from %s", count, full_path.c_str());
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to scan mesh directory '%s/%s': %s", 
                 package_name.c_str(), relative_path.c_str(), e.what());
    return 0;
  }
  
  return count;
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

std::vector<std::string> MeshServer::getRegisteredMeshNames() const
{
  std::vector<std::string> names;
  names.reserve(mesh_registry_.size());
  for (const auto& pair : mesh_registry_) {
    names.push_back(pair.first);
  }
  return names;
}

bool MeshServer::loadMesh(const std::string & path)
{
  // Basic check if file exists
  std::string actual_path = path;
  
  // Handle package:// prefix if necessary
  if (path.find("package://") == 0) {
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

