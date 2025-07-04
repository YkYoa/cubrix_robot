// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ar_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

#ifndef AR_MSGS__MSG__DETAIL__JOY__BUILDER_HPP_
#define AR_MSGS__MSG__DETAIL__JOY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ar_msgs/msg/detail/joy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ar_msgs
{

namespace msg
{

namespace builder
{

class Init_Joy_button_pressed
{
public:
  explicit Init_Joy_button_pressed(::ar_msgs::msg::Joy & msg)
  : msg_(msg)
  {}
  ::ar_msgs::msg::Joy button_pressed(::ar_msgs::msg::Joy::_button_pressed_type arg)
  {
    msg_.button_pressed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ar_msgs::msg::Joy msg_;
};

class Init_Joy_y
{
public:
  explicit Init_Joy_y(::ar_msgs::msg::Joy & msg)
  : msg_(msg)
  {}
  Init_Joy_button_pressed y(::ar_msgs::msg::Joy::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Joy_button_pressed(msg_);
  }

private:
  ::ar_msgs::msg::Joy msg_;
};

class Init_Joy_x
{
public:
  Init_Joy_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Joy_y x(::ar_msgs::msg::Joy::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Joy_y(msg_);
  }

private:
  ::ar_msgs::msg::Joy msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ar_msgs::msg::Joy>()
{
  return ar_msgs::msg::builder::Init_Joy_x();
}

}  // namespace ar_msgs

#endif  // AR_MSGS__MSG__DETAIL__JOY__BUILDER_HPP_
