#include "ar_scene/ar_scene_manager.h"
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shape_operations.h>

namespace ar_scene
{

SceneManager::SceneManager(const rclcpp::Node::SharedPtr& node)
: node_(node),
  logger_(rclcpp::get_logger("ar_scene_manager"))
{
  mesh_server_ = std::make_shared<MeshServer>();
}

SceneManager::~SceneManager()
{
}

bool SceneManager::addMeshObj(const std::string & name, const std::string & mesh_name, const geometry_msgs::msg::PoseStamped & pose_stamped)
{
  std::string mesh_path = mesh_server_->getMeshPath(mesh_name);
  if (mesh_path.empty()) {
    RCLCPP_ERROR(logger_, "Cannot add object '%s': Mesh '%s' not found.", name.c_str(), mesh_name.c_str());
    return false;
  }

  // Load mesh using geometric_shapes or directly as resource
  // Since we are using PlanningSceneInterface, we typically use collision objects with mesh resources.
  // We can construct a CollisionObject.
  
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = pose_stamped.header.frame_id;
  collision_object.id = name;
  
  // Define the mesh
  // Note: PlanningSceneInterface often likes "package://" URIs.
  // If mesh_path is absolute, we might need to convert it to a file:// URI.
  std::string resource_path = "file://" + mesh_path; 
  if (mesh_path.find("package://") == 0) {
      resource_path = mesh_path;
  }

  // We actually need to load the mesh to get its dimensions if we want to use 'addMesh' helper from visual tools,
  // but PlanningSceneInterface.applyCollisionObject takes a raw CollisionObject msg.
  // Or we can use shapes::createMeshFromResource to ensure validity.
  
  shapes::Mesh* m = shapes::createMeshFromResource(resource_path);
  if (!m) {
     RCLCPP_ERROR(logger_, "Failed to load mesh resource: %s", resource_path.c_str());
     return false;
  }
  
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  delete m; // Clean up

  collision_object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
  collision_object.mesh_poses.push_back(pose_stamped.pose);
  collision_object.operation = collision_object.ADD;

  planning_scene_interface_.applyCollisionObject(collision_object);
  RCLCPP_INFO(logger_, "Added collision object '%s' with mesh '%s'", name.c_str(), mesh_name.c_str());
  
  return true;
}

bool SceneManager::removeMeshObj(const std::string & name)
{
  std::vector<std::string> object_ids;
  object_ids.push_back(name);
  planning_scene_interface_.removeCollisionObjects(object_ids);
  RCLCPP_INFO(logger_, "Removed collision object '%s'", name.c_str());
  return true;
}

bool SceneManager::allowCollision(const std::string & name, bool allow)
{
  // This typically involves interacting with the AllowedCollisionMatrix (ACM).
  // PlanningSceneInterface doesn't expose ACM modification directly easily without getting/setting full scene.
  // However, we can use PlanningSceneMonitor if we had one, but here we only have Interface.
  // Actually, 'allowCollision' usually implies updating the ACM.
  // An alternative is adding the object with specific collision operations, but that's for the object itself.
  
  // To modify ACM properly, we might need to use a PlanningSceneMonitor linked to the scene, 
  // or call a service like /get_planning_scene and /apply_planning_scene.
  
  // For now, I'll log a warning that this might need more robust implementation (e.g. via service)
  // or simple ACM update via the planning scene diff.
  
  RCLCPP_WARN(logger_, "allowCollision not fully implemented via simple PlanningSceneInterface. Requires ACM update.");
  (void)name;
  (void)allow;
  return false;
}

} // namespace ar_scene
