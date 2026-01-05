#include "ar_scene/ar_scene_manager.h"
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shape_operations.h>
#include <stdexcept>

namespace ar_scene
{

// Static member initialization
std::unique_ptr<SceneManager> SceneManager::instance_ = nullptr;
std::mutex SceneManager::mutex_;

void SceneManager::initialize(const rclcpp::Node::SharedPtr& node)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (instance_ == nullptr) {
    instance_ = std::unique_ptr<SceneManager>(new SceneManager(node));
  }
}

SceneManager& SceneManager::getInstance()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (instance_ == nullptr) {
    throw std::runtime_error("SceneManager not initialized. Call SceneManager::initialize() first.");
  }
  return *instance_;
}

bool SceneManager::isInitialized()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return instance_ != nullptr;
}

SceneManager::SceneManager(const rclcpp::Node::SharedPtr& node)
: node_(node),
  logger_(rclcpp::get_logger("ar_scene_manager"))
{
  mesh_server_ = std::make_shared<MeshServer>();
  RCLCPP_INFO(logger_, "SceneManager initialized");
}

SceneManager::~SceneManager()
{
}

bool SceneManager::loadMeshesFromConfig()
{
  int count = mesh_server_->registerMeshDirectory("ar_moveit_config", "meshes/obj");
  if (count > 0) {
    RCLCPP_INFO(logger_, "Loaded %d meshes from ar_moveit_config/meshes/obj", count);
    return true;
  }
  RCLCPP_WARN(logger_, "No meshes loaded from ar_moveit_config/meshes/obj");
  return false;
}

std::vector<std::string> SceneManager::getRegisteredMeshes() const
{
  return mesh_server_->getRegisteredMeshNames();
}

bool SceneManager::addMeshObj(const std::string & name, const std::string & mesh_name, const geometry_msgs::msg::PoseStamped & pose_stamped)
{
  std::string mesh_path = mesh_server_->getMeshPath(mesh_name);
  if (mesh_path.empty()) {
    RCLCPP_ERROR(logger_, "Cannot add object '%s': Mesh '%s' not found.", name.c_str(), mesh_name.c_str());
    return false;
  }

  // Load mesh using geometric_shapes
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = pose_stamped.header.frame_id;
  collision_object.id = name;
  
  // Use file:// prefix for absolute paths
  std::string resource_path = "file://" + mesh_path; 
  if (mesh_path.find("package://") == 0) {
    resource_path = mesh_path;
  }

  shapes::Mesh* m = shapes::createMeshFromResource(resource_path);
  if (!m) {
    RCLCPP_ERROR(logger_, "Failed to load mesh resource: %s", resource_path.c_str());
    return false;
  }
  
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  delete m;

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
  // PlanningSceneInterface doesn't expose ACM modification directly without getting/setting full scene.
  RCLCPP_WARN(logger_, "allowCollision not fully implemented via simple PlanningSceneInterface. Requires ACM update.");
  (void)name;
  (void)allow;
  return false;
}

} // namespace ar_scene
