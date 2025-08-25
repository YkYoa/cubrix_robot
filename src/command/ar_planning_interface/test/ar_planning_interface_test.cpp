#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include "ar_planning_interface/ar_planning_interface.h"

class ArPlanningInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("ar_planning_interface_test_node");

    // Create ArPlanningInterface instance
    planning_interface_ = std::make_shared<ar_planning_interface::ArPlanningInterface>(node_);
    ASSERT_TRUE(planning_interface_ != nullptr) << "Failed to create ArPlanningInterface";

    // Initialize the planning interface
    // Note: This test assumes a robot description is available on the parameter server
    // For a unit test, you might mock the robot_model_loader or provide a minimal URDF
    // For now, we rely on the robot_description being present for init() to succeed
    ASSERT_TRUE(planning_interface_->init()) << "Failed to initialize ArPlanningInterface";
  }

  void TearDown() override
  {
    planning_interface_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ar_planning_interface::ArPlanningInterface> planning_interface_;
};

TEST_F(ArPlanningInterfaceTest, BasicPlanningRequestTest)
{
  moveit_msgs::msg::MotionPlanRequest req;
  req.group_name = "arm"; // Assuming a group named 'arm' for testing
  req.num_planning_attempts = 1;
  req.allowed_planning_time = 1.0;

  // For simplicity, we are not setting start/goal states here.
  // In a real test, these would be properly defined.
  // Given our dummy planner, we expect a planning failure.

  moveit_msgs::msg::MotionPlanResponse res;
  bool success = planning_interface_->plan(req, res);

  // Our dummy motion planner in ar_motion_planner currently always returns PLANNING_FAILED
  // and an empty trajectory, so we expect the ArPlanningInterface to reflect that.
  EXPECT_FALSE(success);
  EXPECT_EQ(res.error_code_.val, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
