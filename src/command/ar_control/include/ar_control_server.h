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
    class ArDriveControl;

    /**
     * @brief ROS2 action server for robot control commands
     * 
     * Handles incoming control action requests, manages planning state,
     * and executes trajectories. Supports both MoveIt trajectory execution
     * and direct servo control.
     */
    class ArControlServer : public rclcpp::Node
    {
    public:
        using ArControlAction = ar_action::action::ArControl;
        using ArControlServerGoalHandle = rclcpp_action::ServerGoalHandle<ArControlAction>;

        /**
         * @brief Constructor
         * @param robotArg Pointer to hardware interface (default: nullptr)
         * @param options ROS2 node options
         */
        explicit ArControlServer(ArHardwareInterface *robotArg = nullptr, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        
        /**
         * @brief Destructor
         */
        ~ArControlServer();
        
        /**
         * @brief Start the action server
         * 
         * Registers the action server and begins accepting goal requests.
         */
        void startActionServer();

    private:
        rclcpp_action::Server<ArControlAction>::SharedPtr action_server; ///< Action server instance

        /**
         * @brief Handle incoming goal requests
         * @param uuid Unique identifier for the goal
         * @param goal Goal message
         * @return Response indicating if goal is accepted
         */
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const ArControlAction::Goal> goal);

        /**
         * @brief Handle goal cancellation requests
         * @param goal_handle Handle to the goal being cancelled
         * @return Response indicating if cancellation is accepted
         */
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<ArControlServerGoalHandle> goal_handle);

        /**
         * @brief Handle accepted goals
         * @param goal_handle Handle to the accepted goal
         * 
         * Starts execution of the goal in a separate thread.
         */
        void handle_accepted(const std::shared_ptr<ArControlServerGoalHandle> goal_handle);

        /**
         * @brief Execute the control action
         * @param goal_handle Handle to the goal being executed
         * 
         * Processes the control command and executes the requested action.
         */
        void execute(const std::shared_ptr<ArControlServerGoalHandle> goal_handle);

    private:
        // boost::mutex& control_mutex_;
        ArHardwareInterface *robot;
        std::map<int, std::shared_ptr<ArDriveControl>> drive_map_;
        rclcpp::CallbackGroup::SharedPtr server_cb_group;

        rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_topic_sub;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_servo_sub;
        bool planning_state;
    };
}