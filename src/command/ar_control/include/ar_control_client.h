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
	/**
	 * @brief ROS2 action client for robot control commands
	 * 
	 * Provides interface to send control commands to the robot control server
	 * via ROS2 actions. Supports planning state changes and motor control.
	 */
	class ArControlClient : public rclcpp::Node
	{
	public:
		using ArControlAction = ar_action::action::ArControl;
		using ArControlClientGoalHandle = rclcpp_action::ClientGoalHandle<ArControlAction>;

		/**
		 * @brief Constructor
		 * @param serverName Name of the control server node
		 * @param options ROS2 node options
		 */
		explicit ArControlClient(const std::string &serverName = "ar_control_server",
								 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
		
		/**
		 * @brief Destructor
		 */
		~ArControlClient();

		/**
		 * @brief Wait for the control server to become available
		 * @return true if server is available, false on timeout
		 */
		bool waitForServer();

		/**
		 * @brief Send a goal to the control server
		 * @param goal The control goal to send
		 * @param waitServer If true, wait for server before sending
		 * @return true if goal was sent successfully
		 */
		bool sendGoal(const ArControlAction::Goal &goal, bool waitServer = false);
		
		/**
		 * @brief Execute an action command
		 * @param commandType Type of command to execute
		 * @param timeoutSec Timeout in seconds
		 * @param goal The control goal
		 * @return Command result code
		 */
		int actionCommand(ControlCommandType commandType, int timeoutSec, const ArControlAction::Goal &goal);

		/**
		 * @brief Set the planning state
		 * @param planningGroup Name of the planning group (default: empty)
		 * @param commandType Command type (default: CTR_CMD_SEND_WAIT)
		 * @param timeoutSec Timeout in seconds (default: 5)
		 * @return Result code
		 */
		int setPlanningState(const std::string &planningGroup = "", ControlCommandType commandType = ar_control::CTR_CMD_SEND_WAIT,
							 int timeoutSec = 5);
		
		/**
		 * @brief Set motor states (enable/disable)
		 * @param motorCmds Vector of motor enable commands (true = enable, false = disable)
		 * @param commandType Command type
		 * @param timeoutSec Timeout in seconds
		 * @return Result code
		 */
		int setMotorStates(const std::vector<bool> &motorCmds, ControlCommandType commandType, int timeoutSec);

	protected:
		rclcpp_action::Client<ArControlAction>::SharedPtr client_ptr_; ///< Action client pointer

		bool action_in_progress; ///< Flag indicating if an action is currently in progress
		std::shared_future<ArControlClientGoalHandle::SharedPtr> goal_handle_future; ///< Future for goal handle

		/**
		 * @brief Callback for goal response
		 * @param future Future containing the goal handle
		 */
		void goal_response_callback(std::shared_future<ArControlClientGoalHandle::SharedPtr> future);
		
		/**
		 * @brief Callback for action feedback
		 * @param goal_handle Handle to the goal
		 * @param feedback Feedback message
		 */
		void feedback_callback(ArControlClientGoalHandle::SharedPtr, const std::shared_ptr<const ArControlAction::Feedback> feedback);
		
		/**
		 * @brief Callback for action result
		 * @param result Wrapped result containing status and result
		 */
		void result_callback(const ArControlClientGoalHandle::WrappedResult &result);
		rclcpp::CallbackGroup::SharedPtr client_cb_group; ///< Callback group for the client
		typename rclcpp_action::Client<ArControlAction>::SendGoalOptions send_goal_options; ///< Options for sending goals
	};
} // namespace ar_control

typedef std::shared_ptr<ar_control::ArControlClient> tControlClientPtr;