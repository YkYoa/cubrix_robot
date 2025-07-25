# ar_scene

## Overview

`ar_scene` is a ROS 2 C++ package designed to manage 3D objects and meshes in a MoveIt planning scene. It provides high-level APIs for adding, removing, updating, and attaching objects, as well as managing mesh resources and collision properties. This package is intended for use in robotic applications where dynamic scene management and collision handling are required.

## Features
- **Scene Management:** Add, remove, update, and attach objects in the MoveIt planning scene.
- **Mesh Resource Management:** Register, lookup, and lazily load 3D mesh resources by name or ID.
- **Collision Control:** Set collision properties between robot links or planning groups.
- **Integration:** Built on top of MoveIt, MoveIt Visual Tools, and ROS 2.

## Main Components

### 1. SceneManager
- **Purpose:** High-level interface for managing objects in the planning scene.
- **Key Methods:**
  - `addObjectToScene(...)`: Add an object by mesh name or ID.
  - `deleteObjectFromScene(...)`: Remove an object.
  - `updateObjectPose(...)`: Update the pose of an object.
  - `attachObjectToLink(...)`: Attach an object to a robot link.
  - `detachObject(...)`: Detach an object from a link.
  - `setAllowCollisions(...)`: Set collision permissions between links or groups.
  - `resetScene()`: Clear all objects and reset the scene.
- **Dependencies:** Uses `SceneMeshServer` for mesh management and MoveIt interfaces for scene manipulation.

### 2. SceneMeshServer
- **Purpose:** Manages 3D mesh resources for use in the planning scene.
- **Key Methods:**
  - `getMeshPath(mesh_id)`: Get the file path for a mesh.
  - `getMeshName(mesh_id)`: Get the name of a mesh by ID.
  - `getMeshIdByName(mesh_name)`: Get the ID of a mesh by name.
  - `getAllMeshNames()`: List all registered mesh names.
  - `loadMesh(mesh_id)`: Load and return a mesh object.
- **Features:** Scans mesh directories, registers mesh metadata, and loads meshes on demand.

### 3. scene_definition.h
- Contains shared definitions and macros, such as the robot description package name.

## File Structure
- `include/ar_scene/scene_manager.h` — Main scene management class (API)
- `include/ar_scene/scene_mesh_server.h` — Mesh resource manager (API)
- `include/ar_scene/scene_definition.h` — Shared definitions
- `src/scene_manager.cpp` — Implementation of SceneManager
- `src/scene_mesh_server.cpp` — Implementation of SceneMeshServer
- `test/` — Example and test code

## Example Usage

Below is a conceptual example of how you might use the `SceneManager` in a ROS 2 node:

```cpp
#include <rclcpp/rclcpp.hpp>
#include "ar_scene/scene_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scene_manager_example");
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(...); // Initialize as needed
    ar::SceneManager scene_manager(node, "your_mesh_package_name", planning_scene_monitor);

    // Add an object
    arPose pose = ...; // Define pose
    scene_manager.addObjectToScene(pose, "object_label", "mesh_name");

    // Attach to link
    scene_manager.attachObjectToLink("object_label", "link_name");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Dependencies
- ROS 2
- MoveIt
- moveit_visual_tools
- ar_utils
- ar_computation

See `CMakeLists.txt` and `package.xml` for the full list of dependencies.

## Notes
- Mesh files should be placed in the `meshes/obj` directory of the specified mesh package.
- Object labels must be unique within the planning scene.
- The package is intended for use with MoveIt-based robotic applications.

## License
See `package.xml` for license information (to be updated).

## Maintainer
See `package.xml` for maintainer information (to be updated).

## CMakeLists.txt Example

Below is an example of how your `CMakeLists.txt` should look to include all dependencies required for advanced scene management and visualization:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ar_scene)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)

# Add your sources and headers here
# add_library(${PROJECT_NAME} src/ar_scene_manager.cpp src/ar_mesh_server.cpp)
# ament_target_dependencies(${PROJECT_NAME} ...)

ament_package()
```

**Notes:**
- Make sure to add your source and header files to the `add_library` or `add_executable` commands as you implement them.
- Use `ament_target_dependencies` to link all required dependencies to your targets.

## Extended Example Usage

Here are more detailed usage examples for your intern to follow. These assume the API will be implemented as described in the overview:

### Adding and Attaching Objects

```cpp
#include <rclcpp/rclcpp.hpp>
#include "ar_scene/scene_manager.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scene_manager_example");
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    ar::SceneManager scene_manager(node, "your_mesh_package_name", planning_scene_monitor);

    // Define the pose for the object
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = 0.0;
    pose.position.z = 0.5;
    pose.orientation.w = 1.0;

    // Add an object to the scene
    scene_manager.addObjectToScene(pose, "box1", "box_mesh");

    // Attach the object to a robot link
    scene_manager.attachObjectToLink("box1", "wrist_link");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Updating Object Pose

```cpp
// ... after adding the object as above ...
geometry_msgs::msg::Pose new_pose;
new_pose.position.x = 0.6;
new_pose.position.y = 0.1;
new_pose.position.z = 0.5;
new_pose.orientation.w = 1.0;
scene_manager.updateObjectPose("box1", new_pose);
```

### Removing Objects

```cpp
scene_manager.deleteObjectFromScene("box1");
```

### Setting Collision Permissions

```cpp
scene_manager.setAllowCollisions("box1", "robot_link", true); // Allow collision
scene_manager.setAllowCollisions("box1", "robot_link", false); // Disallow collision
```

## Guidance for Extending the Package
- Implement the methods described in the overview in your header and source files.
- Use MoveIt and MoveIt Visual Tools APIs for scene and visualization operations.
- Register and manage mesh resources using the `SceneMeshServer`.
- Add tests and example nodes in the `test/` directory to demonstrate usage.