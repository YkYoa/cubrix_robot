// Implemented PMas master based remote control; Udupa; Apr'23

#include "ar_control_server.h"
#include "ar_hardware_interface.h"

namespace ar_control
{
	using namespace std::placeholders;

	ArControlServer::ArControlServer(ArHardwareInterface* robotArg, const rclcpp::NodeOptions& options)
		: Node("ar_control_server", options)
	{
		robot		   = robotArg;
		planning_state = false;
	}

	void ArControlServer::startActionServer()
	{
		server_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		action_server = rclcpp_action::create_server<ArControlAction>(
			this, "ar_control_server", std::bind(&ArControlServer::handle_goal, this, _1, _2),
			std::bind(&ArControlServer::handle_cancel, this, _1), std::bind(&ArControlServer::handle_accepted, this, _1),
			rcl_action_server_get_default_options(), server_cb_group);
		RCLCPP_INFO(this->get_logger(), "'ar_control_server' created");
	}

	ArControlServer::~ArControlServer()
	{
	}

	rclcpp_action::GoalResponse ArControlServer::handle_goal(const rclcpp_action::GoalUUID& uuid,
															   std::shared_ptr<const ArControlAction::Goal> goal)
	{
		printf(COLOR_GRAY "%s ArControlServer: Received goal request with action %d\n" COLOR_RESET, ar_utils::getCurrentTime().c_str(),
			   goal->action);
		fflush(stdout);
		(void) uuid;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse ArControlServer::handle_cancel(const std::shared_ptr<ArControlServerGoalHandle> goal_handle)
	{
		printf(COLOR_GRAY "%s ArControlServer: Received request to cancel goal\n" COLOR_RESET, ar_utils::getCurrentTime().c_str());
		fflush(stdout);
		(void) goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void ArControlServer::handle_accepted(const std::shared_ptr<ArControlServerGoalHandle> goal_handle)
	{
		printf(COLOR_GRAY "%s ArControlServer: Goal request accepted, executing...\n" COLOR_RESET, ar_utils::getCurrentTime().c_str());
		fflush(stdout);
		// this needs to return quickly to avoid blocking the executor, so spin up a new thread
		std::thread{std::bind(&ArControlServer::execute, this, _1), goal_handle}.detach();
	}

	void ArControlServer::execute(const std::shared_ptr<ArControlServerGoalHandle> goal_handle)
	{
		// Handle control server tasks
		const auto goal = goal_handle->get_goal();
		printf(COLOR_GRAY "%s %s execution started\n" COLOR_RESET, ar_utils::getCurrentTime().c_str(), HostCommandToName[goal->action].c_str());
		fflush(stdout);

		auto result = std::make_shared<ArControlAction::Result>();
		int status	= ControlErrorCode::FAILURE;

		switch(goal->action) {
		case PLANNING: {
			planning_state = !goal->planning_group.empty();
			printf(COLOR_CYAN "%s ArControlServer: Planning %s\n" COLOR_RESET, ar_utils::getCurrentTime().c_str(),
				   planning_state ? ("started (" + goal->planning_group + ")...").c_str() : "finished ********");

			status = ControlErrorCode::SUCCESS;
			break;
		}
		}

		result->status = status;
		goal_handle->succeed(result);
		printf(COLOR_GRAY "%s %s done (status = %d)\n" COLOR_RESET, ar_utils::getCurrentTime().c_str(), HostCommandToName[goal->action].c_str(),
			   status);
		fflush(stdout);
	}

}  // namespace ar_control

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);

	std::shared_ptr<ar_control::ArControlServer> server = std::make_shared<ar_control::ArControlServer>();
	server->startActionServer();

	rclcpp::spin(server);
	return 0;
}
