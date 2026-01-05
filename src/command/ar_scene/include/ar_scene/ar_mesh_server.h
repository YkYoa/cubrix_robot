#ifndef AR_SCENE__AR_MESH_SERVER_H_
#define AR_SCENE__AR_MESH_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>
#include <vector>
#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ar_scene
{

class MeshServer
{
public:
  using SharedPtr = std::shared_ptr<MeshServer>;

  MeshServer();
  ~MeshServer();

  /**
   * @brief Register a mesh with a name and a relative path package://... or absolute path
   * @param name Name of the mesh
   * @param path Path to the mesh file
   * @return true if successful
   */
  bool registerMesh(const std::string & name, const std::string & path);

  /**
   * @brief Register all mesh files from a directory in a package
   * @param package_name ROS package name
   * @param relative_path Path relative to package share directory
   * @return Number of meshes registered
   */
  int registerMeshDirectory(const std::string & package_name, const std::string & relative_path);

  /**
   * @brief Get the full path to a mesh
   * @param name Name of the mesh
   * @return std::string Full path, or empty if not found
   */
  std::string getMeshPath(const std::string & name) const;

  /**
   * @brief Get list of all registered mesh names
   * @return Vector of mesh names
   */
  std::vector<std::string> getRegisteredMeshNames() const;

  /**
   * @brief Check if a mesh file exists and is accessible
   * @param path Path to check
   * @return true if accessible
   */
  bool loadMesh(const std::string & path);

private:
  std::map<std::string, std::string> mesh_registry_;
  rclcpp::Logger logger_;
};

} // namespace ar_scene

#endif // AR_SCENE__AR_MESH_SERVER_H_

