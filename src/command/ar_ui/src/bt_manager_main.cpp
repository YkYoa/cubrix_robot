#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "ar_ui/bt_manager_window.h"

int main(int argc, char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  // Create Qt application
  QApplication app(argc, argv);
  app.setApplicationName("BehaviorTree Manager");
  app.setOrganizationName("AR Robotics");
  
  // Create ROS node
  auto node = std::make_shared<rclcpp::Node>("bt_manager");
  
  // Create and show window
  ar_ui::BTManagerWindow window;
  window.setRosNode(node);
  window.show();
  
  // Run Qt event loop
  int result = app.exec();
  
  // Cleanup
  rclcpp::shutdown();
  return result;
}
