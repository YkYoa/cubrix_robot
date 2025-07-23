// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ar_action:action/ArControl.idl
// generated code does not contain a copyright notice

#ifndef AR_ACTION__ACTION__DETAIL__AR_CONTROL__BUILDER_HPP_
#define AR_ACTION__ACTION__DETAIL__AR_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ar_action/action/detail/ar_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ar_action
{

namespace action
{

namespace builder
{

class Init_ArControl_Goal_client_id
{
public:
  explicit Init_ArControl_Goal_client_id(::ar_action::action::ArControl_Goal & msg)
  : msg_(msg)
  {}
  ::ar_action::action::ArControl_Goal client_id(::ar_action::action::ArControl_Goal::_client_id_type arg)
  {
    msg_.client_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_action::action::ArControl_Goal msg_;
};

class Init_ArControl_Goal_drive_id
{
public:
  explicit Init_ArControl_Goal_drive_id(::ar_action::action::ArControl_Goal & msg)
  : msg_(msg)
  {}
  Init_ArControl_Goal_client_id drive_id(::ar_action::action::ArControl_Goal::_drive_id_type arg)
  {
    msg_.drive_id = std::move(arg);
    return Init_ArControl_Goal_client_id(msg_);
  }

private:
  ::ar_action::action::ArControl_Goal msg_;
};

class Init_ArControl_Goal_joint_names
{
public:
  explicit Init_ArControl_Goal_joint_names(::ar_action::action::ArControl_Goal & msg)
  : msg_(msg)
  {}
  Init_ArControl_Goal_drive_id joint_names(::ar_action::action::ArControl_Goal::_joint_names_type arg)
  {
    msg_.joint_names = std::move(arg);
    return Init_ArControl_Goal_drive_id(msg_);
  }

private:
  ::ar_action::action::ArControl_Goal msg_;
};

class Init_ArControl_Goal_trajectory
{
public:
  explicit Init_ArControl_Goal_trajectory(::ar_action::action::ArControl_Goal & msg)
  : msg_(msg)
  {}
  Init_ArControl_Goal_joint_names trajectory(::ar_action::action::ArControl_Goal::_trajectory_type arg)
  {
    msg_.trajectory = std::move(arg);
    return Init_ArControl_Goal_joint_names(msg_);
  }

private:
  ::ar_action::action::ArControl_Goal msg_;
};

class Init_ArControl_Goal_planning_group
{
public:
  explicit Init_ArControl_Goal_planning_group(::ar_action::action::ArControl_Goal & msg)
  : msg_(msg)
  {}
  Init_ArControl_Goal_trajectory planning_group(::ar_action::action::ArControl_Goal::_planning_group_type arg)
  {
    msg_.planning_group = std::move(arg);
    return Init_ArControl_Goal_trajectory(msg_);
  }

private:
  ::ar_action::action::ArControl_Goal msg_;
};

class Init_ArControl_Goal_action
{
public:
  Init_ArControl_Goal_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArControl_Goal_planning_group action(::ar_action::action::ArControl_Goal::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_ArControl_Goal_planning_group(msg_);
  }

private:
  ::ar_action::action::ArControl_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_action::action::ArControl_Goal>()
{
  return ar_action::action::builder::Init_ArControl_Goal_action();
}

}  // namespace ar_action


namespace ar_action
{

namespace action
{

namespace builder
{

class Init_ArControl_Result_status
{
public:
  Init_ArControl_Result_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ar_action::action::ArControl_Result status(::ar_action::action::ArControl_Result::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_action::action::ArControl_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_action::action::ArControl_Result>()
{
  return ar_action::action::builder::Init_ArControl_Result_status();
}

}  // namespace ar_action


namespace ar_action
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_action::action::ArControl_Feedback>()
{
  return ::ar_action::action::ArControl_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ar_action


namespace ar_action
{

namespace action
{

namespace builder
{

class Init_ArControl_SendGoal_Request_goal
{
public:
  explicit Init_ArControl_SendGoal_Request_goal(::ar_action::action::ArControl_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::ar_action::action::ArControl_SendGoal_Request goal(::ar_action::action::ArControl_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_action::action::ArControl_SendGoal_Request msg_;
};

class Init_ArControl_SendGoal_Request_goal_id
{
public:
  Init_ArControl_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArControl_SendGoal_Request_goal goal_id(::ar_action::action::ArControl_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ArControl_SendGoal_Request_goal(msg_);
  }

private:
  ::ar_action::action::ArControl_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_action::action::ArControl_SendGoal_Request>()
{
  return ar_action::action::builder::Init_ArControl_SendGoal_Request_goal_id();
}

}  // namespace ar_action


namespace ar_action
{

namespace action
{

namespace builder
{

class Init_ArControl_SendGoal_Response_stamp
{
public:
  explicit Init_ArControl_SendGoal_Response_stamp(::ar_action::action::ArControl_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::ar_action::action::ArControl_SendGoal_Response stamp(::ar_action::action::ArControl_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_action::action::ArControl_SendGoal_Response msg_;
};

class Init_ArControl_SendGoal_Response_accepted
{
public:
  Init_ArControl_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArControl_SendGoal_Response_stamp accepted(::ar_action::action::ArControl_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ArControl_SendGoal_Response_stamp(msg_);
  }

private:
  ::ar_action::action::ArControl_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_action::action::ArControl_SendGoal_Response>()
{
  return ar_action::action::builder::Init_ArControl_SendGoal_Response_accepted();
}

}  // namespace ar_action


namespace ar_action
{

namespace action
{

namespace builder
{

class Init_ArControl_GetResult_Request_goal_id
{
public:
  Init_ArControl_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ar_action::action::ArControl_GetResult_Request goal_id(::ar_action::action::ArControl_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_action::action::ArControl_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_action::action::ArControl_GetResult_Request>()
{
  return ar_action::action::builder::Init_ArControl_GetResult_Request_goal_id();
}

}  // namespace ar_action


namespace ar_action
{

namespace action
{

namespace builder
{

class Init_ArControl_GetResult_Response_result
{
public:
  explicit Init_ArControl_GetResult_Response_result(::ar_action::action::ArControl_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::ar_action::action::ArControl_GetResult_Response result(::ar_action::action::ArControl_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_action::action::ArControl_GetResult_Response msg_;
};

class Init_ArControl_GetResult_Response_status
{
public:
  Init_ArControl_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArControl_GetResult_Response_result status(::ar_action::action::ArControl_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ArControl_GetResult_Response_result(msg_);
  }

private:
  ::ar_action::action::ArControl_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_action::action::ArControl_GetResult_Response>()
{
  return ar_action::action::builder::Init_ArControl_GetResult_Response_status();
}

}  // namespace ar_action


namespace ar_action
{

namespace action
{

namespace builder
{

class Init_ArControl_FeedbackMessage_feedback
{
public:
  explicit Init_ArControl_FeedbackMessage_feedback(::ar_action::action::ArControl_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::ar_action::action::ArControl_FeedbackMessage feedback(::ar_action::action::ArControl_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_action::action::ArControl_FeedbackMessage msg_;
};

class Init_ArControl_FeedbackMessage_goal_id
{
public:
  Init_ArControl_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArControl_FeedbackMessage_feedback goal_id(::ar_action::action::ArControl_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ArControl_FeedbackMessage_feedback(msg_);
  }

private:
  ::ar_action::action::ArControl_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_action::action::ArControl_FeedbackMessage>()
{
  return ar_action::action::builder::Init_ArControl_FeedbackMessage_goal_id();
}

}  // namespace ar_action

#endif  // AR_ACTION__ACTION__DETAIL__AR_CONTROL__BUILDER_HPP_
