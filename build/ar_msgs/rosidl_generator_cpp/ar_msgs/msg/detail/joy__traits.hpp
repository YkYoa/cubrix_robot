// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ar_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

#ifndef AR_MSGS__MSG__DETAIL__JOY__TRAITS_HPP_
#define AR_MSGS__MSG__DETAIL__JOY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ar_msgs/msg/detail/joy__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Joy & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: button_pressed
  {
    out << "button_pressed: ";
    rosidl_generator_traits::value_to_yaml(msg.button_pressed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Joy & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: button_pressed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_pressed: ";
    rosidl_generator_traits::value_to_yaml(msg.button_pressed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Joy & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ar_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ar_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ar_msgs::msg::Joy & msg,
  std::ostream & out, size_t indentation = 0)
{
  ar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ar_msgs::msg::Joy & msg)
{
  return ar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ar_msgs::msg::Joy>()
{
  return "ar_msgs::msg::Joy";
}

template<>
inline const char * name<ar_msgs::msg::Joy>()
{
  return "ar_msgs/msg/Joy";
}

template<>
struct has_fixed_size<ar_msgs::msg::Joy>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ar_msgs::msg::Joy>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ar_msgs::msg::Joy>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AR_MSGS__MSG__DETAIL__JOY__TRAITS_HPP_
