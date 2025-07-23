// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ar_srv:srv/Device.idl
// generated code does not contain a copyright notice

#ifndef AR_SRV__SRV__DETAIL__DEVICE__BUILDER_HPP_
#define AR_SRV__SRV__DETAIL__DEVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ar_srv/srv/detail/device__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ar_srv
{

namespace srv
{

namespace builder
{

class Init_Device_Request_valve_states
{
public:
  explicit Init_Device_Request_valve_states(::ar_srv::srv::Device_Request & msg)
  : msg_(msg)
  {}
  ::ar_srv::srv::Device_Request valve_states(::ar_srv::srv::Device_Request::_valve_states_type arg)
  {
    msg_.valve_states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_srv::srv::Device_Request msg_;
};

class Init_Device_Request_valve_ids
{
public:
  explicit Init_Device_Request_valve_ids(::ar_srv::srv::Device_Request & msg)
  : msg_(msg)
  {}
  Init_Device_Request_valve_states valve_ids(::ar_srv::srv::Device_Request::_valve_ids_type arg)
  {
    msg_.valve_ids = std::move(arg);
    return Init_Device_Request_valve_states(msg_);
  }

private:
  ::ar_srv::srv::Device_Request msg_;
};

class Init_Device_Request_percentage
{
public:
  explicit Init_Device_Request_percentage(::ar_srv::srv::Device_Request & msg)
  : msg_(msg)
  {}
  Init_Device_Request_valve_ids percentage(::ar_srv::srv::Device_Request::_percentage_type arg)
  {
    msg_.percentage = std::move(arg);
    return Init_Device_Request_valve_ids(msg_);
  }

private:
  ::ar_srv::srv::Device_Request msg_;
};

class Init_Device_Request_mode
{
public:
  explicit Init_Device_Request_mode(::ar_srv::srv::Device_Request & msg)
  : msg_(msg)
  {}
  Init_Device_Request_percentage mode(::ar_srv::srv::Device_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_Device_Request_percentage(msg_);
  }

private:
  ::ar_srv::srv::Device_Request msg_;
};

class Init_Device_Request_rgb
{
public:
  explicit Init_Device_Request_rgb(::ar_srv::srv::Device_Request & msg)
  : msg_(msg)
  {}
  Init_Device_Request_mode rgb(::ar_srv::srv::Device_Request::_rgb_type arg)
  {
    msg_.rgb = std::move(arg);
    return Init_Device_Request_mode(msg_);
  }

private:
  ::ar_srv::srv::Device_Request msg_;
};

class Init_Device_Request_strip_id
{
public:
  explicit Init_Device_Request_strip_id(::ar_srv::srv::Device_Request & msg)
  : msg_(msg)
  {}
  Init_Device_Request_rgb strip_id(::ar_srv::srv::Device_Request::_strip_id_type arg)
  {
    msg_.strip_id = std::move(arg);
    return Init_Device_Request_rgb(msg_);
  }

private:
  ::ar_srv::srv::Device_Request msg_;
};

class Init_Device_Request_port
{
public:
  explicit Init_Device_Request_port(::ar_srv::srv::Device_Request & msg)
  : msg_(msg)
  {}
  Init_Device_Request_strip_id port(::ar_srv::srv::Device_Request::_port_type arg)
  {
    msg_.port = std::move(arg);
    return Init_Device_Request_strip_id(msg_);
  }

private:
  ::ar_srv::srv::Device_Request msg_;
};

class Init_Device_Request_command
{
public:
  Init_Device_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Device_Request_port command(::ar_srv::srv::Device_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_Device_Request_port(msg_);
  }

private:
  ::ar_srv::srv::Device_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_srv::srv::Device_Request>()
{
  return ar_srv::srv::builder::Init_Device_Request_command();
}

}  // namespace ar_srv


namespace ar_srv
{

namespace srv
{

namespace builder
{

class Init_Device_Response_vacuumdata
{
public:
  explicit Init_Device_Response_vacuumdata(::ar_srv::srv::Device_Response & msg)
  : msg_(msg)
  {}
  ::ar_srv::srv::Device_Response vacuumdata(::ar_srv::srv::Device_Response::_vacuumdata_type arg)
  {
    msg_.vacuumdata = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_srv::srv::Device_Response msg_;
};

class Init_Device_Response_iheartdata
{
public:
  explicit Init_Device_Response_iheartdata(::ar_srv::srv::Device_Response & msg)
  : msg_(msg)
  {}
  Init_Device_Response_vacuumdata iheartdata(::ar_srv::srv::Device_Response::_iheartdata_type arg)
  {
    msg_.iheartdata = std::move(arg);
    return Init_Device_Response_vacuumdata(msg_);
  }

private:
  ::ar_srv::srv::Device_Response msg_;
};

class Init_Device_Response_indicator_data
{
public:
  explicit Init_Device_Response_indicator_data(::ar_srv::srv::Device_Response & msg)
  : msg_(msg)
  {}
  Init_Device_Response_iheartdata indicator_data(::ar_srv::srv::Device_Response::_indicator_data_type arg)
  {
    msg_.indicator_data = std::move(arg);
    return Init_Device_Response_iheartdata(msg_);
  }

private:
  ::ar_srv::srv::Device_Response msg_;
};

class Init_Device_Response_success
{
public:
  Init_Device_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Device_Response_indicator_data success(::ar_srv::srv::Device_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Device_Response_indicator_data(msg_);
  }

private:
  ::ar_srv::srv::Device_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_srv::srv::Device_Response>()
{
  return ar_srv::srv::builder::Init_Device_Response_success();
}

}  // namespace ar_srv

#endif  // AR_SRV__SRV__DETAIL__DEVICE__BUILDER_HPP_
