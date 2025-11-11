// Implemented PMas master based remote control; Udupa; Apr'23

#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ar_action/action/ar_control.hpp"

#include <control_def.h>
#include <ar_utils.h>


namespace ar_control
{
	class ArControlClient : public rclcpp::Node
	{
	public:
		using ArControlAction = ar_action::action::ArControl;
		using ArControlClientGoalHandle = rclcpp_action::ClientGoalHandle<ArControlAction>;

		explicit ArControlClient(const std::string& serverName	  = "ar_control_server",
								   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
		~ArControlClient();

		bool waitForServer();

		bool sendGoal(const ArControlAction::Goal& goal, bool waitServer = false);
		int actionCommand(ControlCommandType commandType, int timeoutSec, const ArControlAction::Goal& goal);

		int setPlanningState(const std::string& planningGroup = "", ControlCommandType commandType = ar_control::CTR_CMD_SEND_WAIT,
							 int timeoutSec = 5);
		int setMotorStates(const std::vector<bool>& motorCmds, ControlCommandType commandType, int timeoutSec);

	protected:
		rclcpp_action::Client<ArControlAction>::SharedPtr client_ptr_;

		bool action_in_progress;
		std::shared_future<ArControlClientGoalHandle::SharedPtr> goal_handle_future;

		void goal_response_callback(std::shared_future<ArControlClientGoalHandle::SharedPtr> future);
		void feedback_callback(ArControlClientGoalHandle::SharedPtr, const std::shared_ptr<const ArControlAction::Feedback> feedback);
		void result_callback(const ArControlClientGoalHandle::WrappedResult& result);
		rclcpp::CallbackGroup::SharedPtr client_cb_group;
		typename rclcpp_action::Client<ArControlAction>::SendGoalOptions send_goal_options;
	};
}  // namespace ar_control

typedef std::shared_ptr<ar_control::ArControlClient> tControlClientPtr;