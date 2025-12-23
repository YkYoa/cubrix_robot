#include <rclcpp/rclcpp.hpp>
#include <ar_planning_interface/ar_planning_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

class PlanningInterfaceTest
{
public:
  PlanningInterfaceTest(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    RCLCPP_INFO(node_->get_logger(), "Initializing Planning Interface Test...");
    
    // Declare parameter for planning group
    node_->declare_parameter<std::string>("planning_group", "Arm");
    std::string group_name = node_->get_parameter("planning_group").as_string();
    
    RCLCPP_INFO(node_->get_logger(), "Using planning group: %s", group_name.c_str());
    planning_interface_ = getMoveGroup(group_name);
  }

  std::shared_ptr<ar_planning_interface::ArPlanningInterface> getMoveGroup(const std::string& group_name)
  {
    RCLCPP_INFO(node_->get_logger(), "Creating MoveGroup for: %s", group_name.c_str());
    return std::make_shared<ar_planning_interface::ArPlanningInterface>(node_, group_name);
  }

  void configurePlanner()
  {
    std::cout << "\n=== Configure Planner ===" << std::endl;
    std::cout << "Select Planning Pipeline:" << std::endl;
    std::cout << "1. OMPL" << std::endl;
    std::cout << "2. PILZ Industrial Motion Planner" << std::endl;
    std::cout << "Enter choice (1-2): ";
    
    int choice;
    std::cin >> choice;

    if (choice == 1) {
      planning_interface_->setPlanningPipelineId("ompl");
      
      std::cout << "\nSelect OMPL Planner:" << std::endl;
      std::cout << "1. RRTConnect" << std::endl;
      std::cout << "2. RRT" << std::endl;
      std::cout << "3. PRM" << std::endl;
      std::cout << "4. BKPIECE" << std::endl;
      std::cout << "Enter choice (1-4): ";
      
      int planner_choice;
      std::cin >> planner_choice;
      
      switch (planner_choice) {
        case 1: planning_interface_->setPlannerId("RRTConnect"); break;
        case 2: planning_interface_->setPlannerId("RRT"); break;
        case 3: planning_interface_->setPlannerId("PRM"); break;
        case 4: planning_interface_->setPlannerId("BKPIECE"); break;
        default: planning_interface_->setPlannerId("RRTConnect");
      }
    } else {
      planning_interface_->setPlanningPipelineId("pilz");  // Must match launch file config
      
      std::cout << "\nSelect PILZ Planner:" << std::endl;
      std::cout << "1. PTP (Point-to-Point)" << std::endl;
      std::cout << "2. LIN (Linear)" << std::endl;
      std::cout << "3. CIRC (Circular)" << std::endl;
      std::cout << "Enter choice (1-3): ";
      
      int planner_choice;
      std::cin >> planner_choice;
      
      switch (planner_choice) {
        case 1: planning_interface_->setPlannerId("PTP"); break;
        case 2: planning_interface_->setPlannerId("LIN"); break;
        case 3: planning_interface_->setPlannerId("CIRC"); break;
        default: planning_interface_->setPlannerId("PTP");
      }
    }

    // Configure velocity and acceleration
    std::cout << "\nEnter velocity scaling factor (0.0-1.0): ";
    double velocity_scaling;
    std::cin >> velocity_scaling;
    planning_interface_->setVelocityScaling(std::clamp(velocity_scaling, 0.0, 1.0));

    std::cout << "Enter acceleration scaling factor (0.0-1.0): ";
    double accel_scaling;
    std::cin >> accel_scaling;
    planning_interface_->setAccelerationScaling(std::clamp(accel_scaling, 0.0, 1.0));

    // Configure blend radius for PILZ
    if (choice == 2) {  // PILZ selected
      std::cout << "\nEnter blend radius (meters, 0.0 = no blending): ";
      double blend_radius;
      std::cin >> blend_radius;
      planning_interface_->setBlendRadius(std::max(0.0, blend_radius));
    }

    RCLCPP_INFO(node_->get_logger(), "Planner configured:");
    RCLCPP_INFO(node_->get_logger(), "  Pipeline: %s", planning_interface_->getPlanningPipelineId().c_str());
    RCLCPP_INFO(node_->get_logger(), "  Planner: %s", planning_interface_->getPlannerId().c_str());
    if (choice == 2) {
      RCLCPP_INFO(node_->get_logger(), "  Blend Radius: %.3f m", planning_interface_->getBlendRadius());
    }
  }

  void setArmGoal()
  {
    std::cout << "\n=== Set Arm Goal ===" << std::endl;
    std::cout << "Select goal type:" << std::endl;
    std::cout << "1. Joint target" << std::endl;
    std::cout << "2. Pose target" << std::endl;
    std::cout << "3. Named target" << std::endl;
    std::cout << "Enter choice (1-3): ";
    
    int choice;
    std::cin >> choice;

    if (choice == 1) {
      // Joint target
      std::vector<double> joint_values(6);
      std::cout << "\nEnter 6 joint values (radians):" << std::endl;
      for (int i = 0; i < 6; i++) {
        std::cout << "Joint " << (i+1) << ": ";
        std::cin >> joint_values[i];
      }
      planning_interface_->setTargetJoints(joint_values);
      RCLCPP_INFO(node_->get_logger(), "Joint target set");
    } 
    else if (choice == 2) {
      // Pose target
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "Base";
      pose.header.stamp = node_->now();
      
      std::cout << "\nEnter pose (position in meters, orientation in quaternion):" << std::endl;
      std::cout << "Position X: "; std::cin >> pose.pose.position.x;
      std::cout << "Position Y: "; std::cin >> pose.pose.position.y;
      std::cout << "Position Z: "; std::cin >> pose.pose.position.z;
      std::cout << "Orientation X: "; std::cin >> pose.pose.orientation.x;
      std::cout << "Orientation Y: "; std::cin >> pose.pose.orientation.y;
      std::cout << "Orientation Z: "; std::cin >> pose.pose.orientation.z;
      std::cout << "Orientation W: "; std::cin >> pose.pose.orientation.w;
      
      planning_interface_->setTargetPose(pose);
      RCLCPP_INFO(node_->get_logger(), "Pose target set");
    }
    else if (choice == 3) {
      // Named target
      std::cin.ignore(); // Clear newline
      std::string target_name;
      std::cout << "\nEnter named target (e.g., 'home', 'ready'): ";
      std::getline(std::cin, target_name);
      
      if (planning_interface_->setNamedTarget(target_name)) {
        RCLCPP_INFO(node_->get_logger(), "Named target '%s' set", target_name.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set named target '%s'", target_name.c_str());
      }
    }
  }

  void planAndExecute()
  {
    std::cout << "\n=== Planning ===" << std::endl;
    
    // Check if we can get current state first
    auto current_joints = planning_interface_->getCurrentJoints();
    if (current_joints.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot get current robot state. Is the robot/simulation running?");
      std::cout << "ERROR: Cannot plan - no current robot state available" << std::endl;
      return;
    }
    
    std::cout << "Current joint values: [";
    for (size_t i = 0; i < current_joints.size(); i++) {
      std::cout << current_joints[i];
      if (i < current_joints.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node_->get_logger(), "Starting planning...");
    
    if (planning_interface_->plan(plan)) {
      std::cout << "\n Planning succeeded!" << std::endl;
      std::cout << "Trajectory has " << plan.trajectory_.joint_trajectory.points.size() << " waypoints" << std::endl;
      
      // Visualize waypoints in RViz
      planning_interface_->visualizeTrajectory(plan);
      
      std::cout << "Execute? (y/n): ";
      char execute;
      std::cin >> execute;
      
      if (execute == 'y' || execute == 'Y') {
        RCLCPP_INFO(node_->get_logger(), "Executing trajectory...");
        if (planning_interface_->execute(plan)) {
          RCLCPP_INFO(node_->get_logger(), "Execution completed successfully");
          std::cout << "Motion completed!" << std::endl;
          
          // Wait for state to propagate
          std::cout << "Waiting for state update..." << std::endl;
          rclcpp::sleep_for(std::chrono::milliseconds(500));
          
          // Wait for execution to finish and state to update
          rclcpp::sleep_for(std::chrono::seconds(1));

          
          // Display updated state
          auto updated_joints = planning_interface_->getCurrentJoints();
          std::cout << "\nUpdated joint values: [";
          for (size_t i = 0; i < updated_joints.size(); i++) {
            std::cout << updated_joints[i];
            if (i < updated_joints.size() - 1) std::cout << ", ";
          }
          std::cout << "]" << std::endl;
          
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Execution failed");
          std::cout << " Execution failed!" << std::endl;
        }
      }
    }
  }

  void displayCurrentState()
  {
    // Refresh state by spinning a few times
    std::cout << "Refreshing state..." << std::endl;
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    
    auto current_pose = planning_interface_->getCurrentPose();
    auto current_joints = planning_interface_->getCurrentJoints();
    
    std::cout << "\n=== Current State ===" << std::endl;
    std::cout << "Position: [" 
              << current_pose.pose.position.x << ", "
              << current_pose.pose.position.y << ", "
              << current_pose.pose.position.z << "], " 
              << "Orientation: ["
              << current_pose.pose.orientation.x << ", " 
              << current_pose.pose.orientation.y << ", " 
              << current_pose.pose.orientation.z << ", " 
              << current_pose.pose.orientation.w << "] " 
              << std::endl;
    
    std::cout << "Joint Values: [";
    for (size_t i = 0; i < current_joints.size(); i++) {
      std::cout << current_joints[i];
      if (i < current_joints.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }

  void run()
  {
    while (rclcpp::ok()) {
      std::cout << "\n======================================" << std::endl;
      std::cout << "AR Planning Interface Test Menu" << std::endl;
      std::cout << "======================================" << std::endl;
      std::cout << "1. Configure Planner" << std::endl;
      std::cout << "2. Set Arm Goal" << std::endl;
      std::cout << "3. Plan and Execute" << std::endl;
      std::cout << "4. Display Current State" << std::endl;
      std::cout << "5. Exit" << std::endl;
      std::cout << "Enter choice: ";
      
      int choice;
      std::cin >> choice;
      
      switch (choice) {
        case 1:
          configurePlanner();
          break;
        case 2:
          setArmGoal();
          break;
        case 3:
          planAndExecute();
          break;
        case 4:
          displayCurrentState();
          break;
        case 5:
          return;
        default:
          std::cout << "Invalid choice" << std::endl;
      }
      

    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("ar_planning_interface_test");
  
  std::thread([&node]() {
    rclcpp::spin(node);
  }).detach();
  
  // Small delay to ensure MoveIt is ready
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  PlanningInterfaceTest test(node);
  test.run();
  
  rclcpp::shutdown();
  return 0;
}
