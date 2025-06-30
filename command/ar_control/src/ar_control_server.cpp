// Implemented PMas master based remote control; Udupa; Apr'23

#include <ar_control/ar_control_server.h>
#include <ar_control/ar_hardware_interface.h>

namespace ar_control
{
	using namespace std::placeholders;

	ArControlServer::ArControlServer(ArHardwareInterface* robotArg, const rclcpp::NodeOptions& options)
		: Node("ar_control_server", options)
	{
		robot		   = robotArg;
		planning_state = false;
	}

	void ArControlServer::createExecutionListener()
	{
		RCLCPP_INFO(rclcpp::get_logger("Ar"), "******** Starting RViz execution listener");
		trajectory_topic_sub = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
			"display_planned_path", 2, std::bind(&ArControlServer::incomingDisplayTrajectory, this, _1));
	}

	void ArControlServer::incomingDisplayTrajectory(const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr msg)
	{
		if(planning_state)
			return;

		printf(COLOR_CYAN "%s Received trajectory from RViz\n" COLOR_RESET, ar_utils::getCurrentTime().c_str());
		fflush(stdout);
		int status = ControlErrorCode::FAILURE;
		robot->move("", msg->trajectory[0].joint_trajectory, status);
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

	rclcpp_action::CancelResponse TomoControlServer::handle_cancel(const std::shared_ptr<TomoControlServerGoalHandle> goal_handle)
	{
		printf(COLOR_GRAY "%s TomoControlServer: Received request to cancel goal\n" COLOR_RESET, tomo_utils::getCurrentTime().c_str());
		fflush(stdout);
		(void) goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void TomoControlServer::handle_accepted(const std::shared_ptr<TomoControlServerGoalHandle> goal_handle)
	{
		printf(COLOR_GRAY "%s TomoControlServer: Goal request accepted, executing...\n" COLOR_RESET, tomo_utils::getCurrentTime().c_str());
		fflush(stdout);
		// this needs to return quickly to avoid blocking the executor, so spin up a new thread
		std::thread{std::bind(&TomoControlServer::execute, this, _1), goal_handle}.detach();
	}

	void TomoControlServer::execute(const std::shared_ptr<TomoControlServerGoalHandle> goal_handle)
	{
		// Handle control server tasks
		const auto goal = goal_handle->get_goal();
		printf(COLOR_GRAY "%s %s execution started\n" COLOR_RESET, tomo_utils::getCurrentTime().c_str(), HostCommandToName[goal->action].c_str());
		fflush(stdout);

		auto result = std::make_shared<TomoControlAction::Result>();
		int status	= ControlErrorCode::FAILURE;

		switch(goal->action) {
		case SERVO_ON_OFF: {
			if(robot) {
				// boost::mutex::scoped_lock lock(control_mutex_);
				tVectorB servo_states = robot->servoOnOff(goal->joint_names, goal->servo_commands);
				result->servo_states  = servo_states;
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}

		case GET_SERVO_STATES: {
			if(robot) {
				// boost::mutex::scoped_lock lock(control_mutex_);
				tVectorB servo_states = robot->getDrivesState(goal->joint_names);
				result->servo_states = servo_states;
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}

		case SET_HALT: {
			if(robot) {
				// boost::mutex::scoped_lock lock(control_mutex_);
				robot->setHalt();
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}
		case RESET_HALT: {
			if(robot) {
				// boost::mutex::scoped_lock lock(control_mutex_);
				robot->resetHalt();
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}

		case UPDATE_POS: {
			if(robot) {
				// boost::mutex::scoped_lock lock(control_mutex_);
				robot->updatePosition();
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}

		case SET_QUICK_STOP: {
			if(robot) {
				// boost::mutex::scoped_lock lock(control_mutex_);
				robot->quickStop();
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}

		case RESET_QUICK_STOP: {
			if(robot) {
				// boost::mutex::scoped_lock lock(control_mutex_);
				robot->resetQuickStop();
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}

		case PLANNING: {
			planning_state = !goal->planning_group.empty();
			printf(COLOR_CYAN "%s TomoControlServer: Planning %s\n" COLOR_RESET, tomo_utils::getCurrentTime().c_str(),
				   planning_state ? ("started (" + goal->planning_group + ")...").c_str() : "finished ********");

			status = ControlErrorCode::SUCCESS;
			break;
		}

		case MOVE_EXECUTE: {
			if(robot) {
				// boost::mutex::scoped_lock lock(control_mutex_);
				robot->move(goal->planning_group, goal->trajectory, status);
			}
			else
				status = ControlErrorCode::SUCCESS;

			break;
		}

		case CHANGE_TOOL: {
			if(robot) {
				robot->changeTool(goal->drive_id);
			}
			else
				status = ControlErrorCode::SUCCESS;

			break;
		}
		case HOME_DKK_DRIVE: {
			if(robot) {
				robot->homeDkkDrives();
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}
		case SET_DKK_PID: {
			if(robot) {
				robot->setDkkPid(goal->client_id, goal->kp, goal->ki, goal->kd);
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}
		case TORQUE_CONTROL: {
			if(robot) {
				robot->torqueDkkControl(goal->target_torque, goal->client_id);
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}
		case POSITION_CONTROL: {
			if(robot) {
				robot->positionDkkControl();
			}
			status = ControlErrorCode::SUCCESS;
			break;
		}
		}

		result->status = status;
		goal_handle->succeed(result);
		printf(COLOR_GRAY "%s %s done (status = %d)\n" COLOR_RESET, tomo_utils::getCurrentTime().c_str(), HostCommandToName[goal->action].c_str(),
			   status);
		fflush(stdout);
	}

	void TomoControlServer::setIheart(int mode, int stripId, const std::string& rgb, const std::string& percentage)
	{
		if(device_client_){
			if(percentage != ""){
				device_client_->setIheartBringup(mode, stripId, rgb, percentage);
			}
			else{
				device_client_->setIheart(stripId, rgb, mode);
			}
		}
	}
}  // namespace tomo_control

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);

	std::shared_ptr<tomo_control::TomoControlServer> server = std::make_shared<tomo_control::TomoControlServer>();
	server->startActionServer();

	rclcpp::spin(server);
	return 0;
}
