#include "ar_control_client.h"

namespace ar_control
{
	auto CLIENT_LOG = rclcpp::get_logger("client");
	using namespace std::placeholders;
	ArControlClient::ArControlClient(const std::string& serverName, const rclcpp::NodeOptions& options)
		: Node("ar_control_client", options)
	{
		RCLCPP_INFO(CLIENT_LOG, "Creating 'ar_control_client'");
		client_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		client_ptr_		= rclcpp_action::create_client<ArControlAction>(this, serverName, client_cb_group);
		RCLCPP_INFO(CLIENT_LOG, "'ar_control_client' created");

		send_goal_options.goal_response_callback =
			[this](const rclcpp_action::Client<ArControlAction>::GoalHandle::SharedPtr& goal_handle) {
				if(!goal_handle) {
					RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: Goal was rejected by server");
				}
				else {
					printf(COLOR_GRAY "%s ArControlClient: Goal accepted by server, waiting for result\n" COLOR_RESET,
						   ar_utils::getCurrentTime().c_str());
					fflush(stdout);
				}
			};
		send_goal_options.feedback_callback		 = std::bind(&ArControlClient::feedback_callback, this, _1, _2);
		send_goal_options.result_callback		 = std::bind(&ArControlClient::result_callback, this, _1);
	}

	ArControlClient::~ArControlClient()
	{
	}

	void ArControlClient::goal_response_callback(std::shared_future<ArControlClientGoalHandle::SharedPtr> future)
	{
		auto goal_handle = future.get();
		if(!goal_handle) {
			RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: Goal was rejected by server");
		}
		else {
			printf(COLOR_GRAY "%s ArControlClient: Goal accepted by server, waiting for result\n" COLOR_RESET,
				   ar_utils::getCurrentTime().c_str());
			fflush(stdout);
		}
	}

	void ArControlClient::feedback_callback(ArControlClientGoalHandle::SharedPtr,
											  const std::shared_ptr<const ArControlAction::Feedback> feedback)
	{
		(void) feedback;  // Mark feedback as unused
		RCLCPP_INFO(CLIENT_LOG, "ArControlClient: Feedback is not implemented yet");
	}

	void ArControlClient::result_callback(const ArControlClientGoalHandle::WrappedResult& result)
	{
		switch(result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: Goal success with code %d", result.result->status);
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: Goal was aborted");
			return;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: Goal was canceled");
			return;
		default:
			RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: Unknown result code");
			return;
		}
	}

	bool ArControlClient::waitForServer()
	{
		if(!client_ptr_->wait_for_action_server()) {
			RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: Action server not available after waiting");
			return false;
		}

		// RCLCPP_INFO(CLIENT_LOG, "ArControlClient: Connected to action server");
		return true;
	}

	bool ArControlClient::sendGoal(const ArControlAction::Goal& goal, bool waitServer)
	{
		if(waitServer) {
			if(!waitForServer())
				return false;
		}

		printf(COLOR_GRAY "%s ArControlClient: Sending goal: %s\n" COLOR_RESET, ar_utils::getCurrentTime().c_str(),
			   HostCommandToName[goal.action].c_str());

		client_ptr_->async_send_goal(goal, send_goal_options);
		// client_ptr_->async_send_goal(goal);

		return true;
	}

	int ArControlClient::actionCommand(ControlCommandType commandType, int timeoutSec, const ArControlAction::Goal& goal)
	{
		if(!waitForServer())
			return false;

		const char* actionName = HostCommandToName[goal.action].c_str();
		if(commandType != CTR_CMD_WAIT) {
			if(action_in_progress) {
				RCLCPP_WARN(CLIENT_LOG, "ArControlClient: Previous action result discarded");
				/// Need to reset action status here
			}
			goal_handle_future = client_ptr_->async_send_goal(goal);
			// sendGoal(goal);
			action_in_progress = true;
			printf(COLOR_GRAY "%s ArControlClient: '%s' sent\n" COLOR_RESET, ar_utils::getCurrentTime().c_str(), actionName);
			fflush(stdout);
		}

		if(commandType != CTR_CMD_SEND) {
			if(!action_in_progress) {
				RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: '%s': No action result pending; Ignoring the wait", actionName);
				return -1;
			}

			printf(COLOR_GRAY "%s ArControlClient: '%s': Waiting for result\n" COLOR_RESET, ar_utils::getCurrentTime().c_str(),
				   actionName);
			fflush(stdout);

			int status = -1;
			if(goal_handle_future.wait_for(std::chrono::seconds(timeoutSec)) == std::future_status::ready) {

				// auto result = goal_handle_future.get();
				auto result = client_ptr_->async_get_result(goal_handle_future.get());
				status		= (int) result.get().result->status;
				printf(COLOR_GRAY "%s ArControlClient: '%s' completed (result = %d)\n" COLOR_RESET, ar_utils::getCurrentTime().c_str(),
					   actionName, status);
				fflush(stdout);
			}
			else
				RCLCPP_ERROR(CLIENT_LOG, "ArControlClient: '%s' timed out (%d sec)", actionName, timeoutSec);

			action_in_progress = false;
			return status;
		}
		return 0;
	}

	int ArControlClient::setPlanningState(const std::string& planningGroup, ControlCommandType commandType, int timeoutSec)
	{
		auto goal			= ArControlAction::Goal();
		goal.action			= HostCommand::PLANNING;
		goal.planning_group = planningGroup;

		return actionCommand(commandType, timeoutSec, goal);
	}

}  // namespace ar_control
