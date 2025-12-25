#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <ar_planning_interface/ar_planning_interface.h>
#include "ar_bt/bt_action_nodes.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ar_bt_executor");

  // Declare parameters
  node->declare_parameter<std::string>("tree_file", "");
  node->declare_parameter<std::string>("planning_group", "Arm");


  std::string tree_file = node->get_parameter("tree_file").as_string();
  std::string planning_group = node->get_parameter("planning_group").as_string();

  if (tree_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'tree_file' is required!");
    RCLCPP_INFO(node->get_logger(), "Usage: ros2 run ar_bt bt_executor_node --ros-args -p tree_file:=path/to/tree.xml");
    rclcpp::shutdown();
    return 1;
  }

  // Resolve tree file path
  std::filesystem::path tree_path(tree_file);
  if (!tree_path.is_absolute()) {
    // Try to find in package share directory
    try {
      auto share_dir = ament_index_cpp::get_package_share_directory("ar_bt");
      tree_path = std::filesystem::path(share_dir) / "trees" / tree_file;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to find ar_bt package: %s", e.what());
      rclcpp::shutdown();
      return 1;
    }
  }

  if (!std::filesystem::exists(tree_path)) {
    RCLCPP_ERROR(node->get_logger(), "Tree file does not exist: %s", tree_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", tree_path.c_str());

  rclcpp::sleep_for(std::chrono::seconds(2));

  auto planning_interface = std::make_shared<ar_planning_interface::ArPlanningInterface>(
    node, planning_group);

  RCLCPP_INFO(node->get_logger(), "Planning interface ready for group: %s", planning_group.c_str());

  BT::BehaviorTreeFactory factory;
  ar_bt::registerARBTNodes(factory, node, planning_interface);

  RCLCPP_INFO(node->get_logger(), "Registered AR BT nodes");

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(tree_path.string());
    RCLCPP_INFO(node->get_logger(), "Behavior tree loaded successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load tree: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  BT::StdCoutLogger logger_cout(tree);

  RCLCPP_INFO(node->get_logger(), "Executing behavior tree...");
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Behavior tree execution SUCCEEDED");
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "Behavior tree execution FAILED");
  } else {
    RCLCPP_WARN(node->get_logger(), "Behavior tree execution HALTED");
  }

  rclcpp::shutdown();
  return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
