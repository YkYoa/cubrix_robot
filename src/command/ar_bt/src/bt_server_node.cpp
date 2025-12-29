#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <ar_planning_interface/ar_planning_interface.h>
#include "ar_bt/bt_action_nodes.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <filesystem>
#include <thread>
#include <atomic>
#include <mutex>

/**
 * @brief BT Server Node - Continuously running behavior tree executor
 * 
 * Subscribes to /ar_bt/execute_command to receive tree file paths
 * Publishes status to /ar_bt/execution_status
 */
class BTServerNode : public rclcpp::Node
{
public:
  BTServerNode()
    : Node("ar_bt_server")
    , is_executing_(false)
    , stop_requested_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("planning_group", "Arm");
    planning_group_ = this->get_parameter("planning_group").as_string();

    // Create publisher and subscriber
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/ar_bt/execution_status", 10);

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/ar_bt/execute_command", 10,
      std::bind(&BTServerNode::commandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "BT Server initialized, waiting for commands...");
    RCLCPP_INFO(this->get_logger(), "  Subscribe to: /ar_bt/execute_command");
    RCLCPP_INFO(this->get_logger(), "  Publish to: /ar_bt/execution_status");

    // Initialize planning interface
    planning_interface_ = std::make_shared<ar_planning_interface::ArPlanningInterface>(
      shared_from_this(), planning_group_);

    // Initialize BT factory and register nodes
    ar_bt::registerARBTNodes(factory_, shared_from_this(), planning_interface_);
    RCLCPP_INFO(this->get_logger(), "Registered AR BT nodes for group: %s", planning_group_.c_str());
  }

  void spin()
  {
    rclcpp::spin(shared_from_this());
  }

private:
  void commandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string command = msg->data;
    
    if (command == "STOP") {
      if (is_executing_) {
        RCLCPP_INFO(this->get_logger(), "Stop requested");
        stop_requested_ = true;
        publishStatus("STOPPED: Execution interrupted");
      }
      return;
    }

    // Treat as tree file path
    if (is_executing_) {
      RCLCPP_WARN(this->get_logger(), "Already executing a tree, ignoring new request");
      publishStatus("BUSY: Already executing a tree");
      return;
    }

    // Start execution in a separate thread
    std::thread(&BTServerNode::executeTree, this, command).detach();
  }

  void executeTree(std::string tree_file)
  {
    is_executing_ = true;
    stop_requested_ = false;

    RCLCPP_INFO(this->get_logger(), "Executing tree: %s", tree_file.c_str());
    publishStatus("RUNNING: Starting execution...");

    // Resolve tree file path
    std::filesystem::path tree_path(tree_file);
    if (!tree_path.is_absolute()) {
      try {
        auto share_dir = ament_index_cpp::get_package_share_directory("ar_bt");
        tree_path = std::filesystem::path(share_dir) / "trees" / tree_file;
      } catch (const std::exception& e) {
        publishStatus("FAILURE: Failed to find ar_bt package");
        is_executing_ = false;
        return;
      }
    }

    if (!std::filesystem::exists(tree_path)) {
      publishStatus("FAILURE: Tree file not found: " + tree_path.string());
      RCLCPP_ERROR(this->get_logger(), "Tree file not found: %s", tree_path.c_str());
      is_executing_ = false;
      return;
    }

    // Load and execute tree
    try {
      BT::Tree tree = factory_.createTreeFromFile(tree_path.string());
      publishStatus("RUNNING: Tree loaded, executing...");
      RCLCPP_INFO(this->get_logger(), "Tree loaded successfully");

      BT::NodeStatus status = BT::NodeStatus::RUNNING;

      while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && !stop_requested_) {
        status = tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if (stop_requested_) {
        tree.haltTree();
        publishStatus("STOPPED: Execution halted by user");
        RCLCPP_INFO(this->get_logger(), "Execution stopped by user");
      } else if (status == BT::NodeStatus::SUCCESS) {
        publishStatus("SUCCESS: Execution completed");
        RCLCPP_INFO(this->get_logger(), "Execution SUCCEEDED");
      } else if (status == BT::NodeStatus::FAILURE) {
        publishStatus("FAILURE: Execution failed");
        RCLCPP_ERROR(this->get_logger(), "Execution FAILED");
      }

    } catch (const std::exception& e) {
      publishStatus("FAILURE: " + std::string(e.what()));
      RCLCPP_ERROR(this->get_logger(), "Execution error: %s", e.what());
    }

    is_executing_ = false;
    stop_requested_ = false;
  }

  void publishStatus(const std::string& status)
  {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
  }

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  // Planning
  std::string planning_group_;
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;

  // BehaviorTree
  BT::BehaviorTreeFactory factory_;

  // State
  std::atomic<bool> is_executing_;
  std::atomic<bool> stop_requested_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Need to create node first, then call shared_from_this in constructor
  // This requires a different approach
  auto node = std::make_shared<rclcpp::Node>("ar_bt_server");
  
  // Declare parameters
  node->declare_parameter<std::string>("planning_group", "Arm");
  std::string planning_group = node->get_parameter("planning_group").as_string();

  RCLCPP_INFO(node->get_logger(), "BT Server started");
  RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());

  // Wait for other nodes to be ready
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Initialize planning interface
  auto planning_interface = std::make_shared<ar_planning_interface::ArPlanningInterface>(
    node, planning_group);

  // Initialize BT factory
  BT::BehaviorTreeFactory factory;
  ar_bt::registerARBTNodes(factory, node, planning_interface);
  RCLCPP_INFO(node->get_logger(), "Registered AR BT nodes");

  // State
  std::atomic<bool> is_executing{false};
  std::atomic<bool> stop_requested{false};
  std::mutex execution_mutex;

  // Publisher with transient local QoS (keeps last message for late subscribers)
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  auto status_pub = node->create_publisher<std_msgs::msg::String>(
    "/ar_bt/execution_status", status_qos);

  auto publish_status = [&](const std::string& status) {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Status: %s", status.c_str());
  };

  // Subscriber
  auto command_sub = node->create_subscription<std_msgs::msg::String>(
    "/ar_bt/execute_command", 10,
    [&](const std_msgs::msg::String::SharedPtr msg) {
      std::string command = msg->data;
      
      if (command == "STOP") {
        if (is_executing) {
          RCLCPP_INFO(node->get_logger(), "Stop requested");
          stop_requested = true;
        }
        return;
      }

      // Start tree execution
      if (is_executing) {
        publish_status("BUSY: Already executing");
        return;
      }

      // Execute in thread
      std::thread([&, command]() {
        std::lock_guard<std::mutex> lock(execution_mutex);
        is_executing = true;
        stop_requested = false;

        std::string tree_file = command;
        RCLCPP_INFO(node->get_logger(), "Executing: %s", tree_file.c_str());
        publish_status("RUNNING: Loading tree...");

        // Resolve path
        std::filesystem::path tree_path(tree_file);
        if (!tree_path.is_absolute()) {
          try {
            auto share_dir = ament_index_cpp::get_package_share_directory("ar_bt");
            tree_path = std::filesystem::path(share_dir) / "trees" / tree_file;
          } catch (...) {
            publish_status("FAILURE: Package not found");
            is_executing = false;
            return;
          }
        }

        if (!std::filesystem::exists(tree_path)) {
          publish_status("FAILURE: File not found: " + tree_path.string());
          is_executing = false;
          return;
        }

        try {
          BT::Tree tree = factory.createTreeFromFile(tree_path.string());
          publish_status("RUNNING: Executing...");
          
          // Enable state transition logging (IDLE -> RUNNING, etc.)
          BT::StdCoutLogger logger_cout(tree);

          BT::NodeStatus status = BT::NodeStatus::RUNNING;
          while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && !stop_requested) {
            status = tree.tickRoot();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }

          if (stop_requested) {
            tree.haltTree();
            publish_status("STOPPED: Halted by user");
          } else if (status == BT::NodeStatus::SUCCESS) {
            publish_status("SUCCESS: Completed");
          } else {
            publish_status("FAILURE: Execution failed");
          }
        } catch (const std::exception& e) {
          publish_status("FAILURE: " + std::string(e.what()));
        }

        is_executing = false;
        stop_requested = false;
      }).detach();
    });

  RCLCPP_INFO(node->get_logger(), "BT Server ready - listening on /ar_bt/execute_command");
  publish_status("READY: Waiting for commands");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
