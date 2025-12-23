#ifndef AR_SCENE__AR_SCENE_MANAGER_H_
#define AR_SCENE__AR_SCENE_MANAGER_H_

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>
#include <memory>
#include "ar_scene/ar_mesh_server.h"

namespace ar_scene
{

class SceneManager
{
public:
  using SharedPtr = std::shared_ptr<SceneManager>;

  explicit SceneManager(const rclcpp::Node::SharedPtr& node);
  ~SceneManager();

  /**
   * @brief Add a mesh object to the planning scene
   * @param name Name of the object
   * @param mesh_name Name of the mesh registered in MeshServer
   * @param pose_stamped Pose of the object
   * @return true if successful
   */
  bool addMeshObj(const std::string & name, const std::string & mesh_name, const geometry_msgs::msg::PoseStamped & pose_stamped);

  /**
   * @brief Remove an object from the planning scene
   * @param name Name of the object
   * @return true if successful
   */
  bool removeMeshObj(const std::string & name);

  /**
   * @brief Allow collision between two objects or an object and the robot
   * @param name Name of the object
   * @param allow True to allow collision, false to disallow (restore default)
   */
  bool allowCollision(const std::string & name, bool allow);

  /**
   * @brief Get the internal MeshServer
   */
  std::shared_ptr<MeshServer> getMeshServer() const { return mesh_server_; }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::shared_ptr<MeshServer> mesh_server_;
  rclcpp::Logger logger_;
};

} // namespace ar_scene

#endif // AR_SCENE__AR_SCENE_MANAGER_H_
