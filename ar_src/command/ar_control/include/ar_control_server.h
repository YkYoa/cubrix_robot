// Implemented PMas master based remote control; Udupa; Apr'23

#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <functional>
#include <memory>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "ar_action/action/ar_control.hpp"

#include <ar_utils.h>
#include <control_def.h>

namespace ar_control
{
    class ArHardwareInterface;

    class ArControlServer : public rclcpp::Node
    {
    public:
        using ArControlAction = ar_action::action::ArControl;
        using ArControlServerGoalHandle = rclcpp_action::ServerGoalHandle<ArControlAction>;

        explicit ArControlServer(ArHardwareInterface* robotArg = nullptr, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ArControlServer();
    	void startActionServer();

	private:
        rclcpp_action::Server<ArControlAction>::SharedPtr action_server;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                                std::shared_ptr<const ArControlAction::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<ArControlServerGoalHandle> goal_handle);

        void handle_accepted(const std::shared_ptr<ArControlServerGoalHandle> goal_handle);

        void execute(const std::shared_ptr<ArControlServerGoalHandle> goal_handle);

	private:
		// boost::mutex& control_mutex_;
		ArHardwareInterface* robot;
		rclcpp::CallbackGroup::SharedPtr server_cb_group;

		rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_topic_sub;
		rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_servo_sub;
		bool planning_state;
	};
}