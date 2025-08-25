#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include "ar_motion_planner/ar_motion_planner.h"

class ArMotionPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("ar_motion_planner_test_node");

    // Load robot model (dummy for testing)
    // In a real scenario, you'd load from parameter server or URDF directly
    // For this test, we'll create a minimal robot model just to satisfy the constructor
    // A more complete test would set up a mock robot model or load a real one.
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
    robot_model_ = robot_model_loader_->getModel();
    ASSERT_TRUE(robot_model_) << "Failed to load robot model";

    // Load the ArMotionPlanner plugin
    try
    {
      planner_loader_ = std::make_shared<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
          "moveit_core", "moveit::planning_interface::PlannerManager");
      planner_ = planner_loader_->createUniqueInstance("ar_motion_planner/ArMotionPlanner");
      ASSERT_TRUE(planner_ != nullptr) << "Failed to load ArMotionPlanner plugin";
    }
    catch (pluginlib::PluginlibException& ex)
    {
      FAIL() << "Exception while loading ArMotionPlanner plugin: " << ex.what();
    }

    // Initialize the planner
    ASSERT_TRUE(planner_->initialize(robot_model_, "")) << "Failed to initialize ArMotionPlanner";

    // Create a dummy planning scene for context
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    ASSERT_TRUE(planning_scene_ != nullptr) << "Failed to create planning scene";
  }

  void TearDown() override
  {
    planner_.reset();
    planner_loader_.reset();
    planning_scene_.reset();
    robot_model_.reset();
    robot_model_loader_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr robot_model_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_loader_;
  planning_interface::PlannerManagerPtr planner_;
};

TEST_F(ArMotionPlannerTest, BasicPlanningTest)
{
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = "arm"; // Assuming a group named 'arm' for testing
  req.num_planning_attempts = 1;
  req.allowed_planning_time = 1.0;

  // Set a dummy start state (e.g., all zeros)
  moveit::core::RobotState start_state(robot_model_);
  start_state.setToDefaultValues();
  planning_scene_->setCurrentState(start_state);
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  // Set a dummy goal constraint (e.g., current state as goal)
  moveit_msgs::msg::JointConstraint jc;
  jc.joint_name = "joint1";
  jc.position = 0.0;
  jc.tolerance_above = 0.01;
  jc.tolerance_below = 0.01;
  jc.weight = 1.0;
  req.goal_constraints.resize(1);
  req.goal_constraints[0].joint_constraints.push_back(jc);

  planning_interface::PlanningContextPtr context = planner_->getPlanningContext(planning_scene_, req);
  ASSERT_TRUE(context != nullptr) << "Failed to get planning context";

  planning_interface::MotionPlanResponse res;
  context->solve(res);

  // As our current solve returns false, we expect a planning failure
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
  EXPECT_FALSE(res.trajectory_);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
