// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ar_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

#ifndef AR_MSGS__MSG__DETAIL__JOY__STRUCT_HPP_
#define AR_MSGS__MSG__DETAIL__JOY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ar_msgs__msg__Joy __attribute__((deprecated))
#else
# define DEPRECATED__ar_msgs__msg__Joy __declspec(deprecated)
#endif

namespace ar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Joy_
{
  using Type = Joy_<ContainerAllocator>;

  explicit Joy_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->button_pressed = false;
    }
  }

  explicit Joy_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->button_pressed = false;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _button_pressed_type =
    bool;
  _button_pressed_type button_pressed;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__button_pressed(
    const bool & _arg)
  {
    this->button_pressed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ar_msgs::msg::Joy_<ContainerAllocator> *;
  using ConstRawPtr =
    const ar_msgs::msg::Joy_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ar_msgs::msg::Joy_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ar_msgs::msg::Joy_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ar_msgs::msg::Joy_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ar_msgs::msg::Joy_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ar_msgs::msg::Joy_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ar_msgs::msg::Joy_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ar_msgs::msg::Joy_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ar_msgs::msg::Joy_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ar_msgs__msg__Joy
    std::shared_ptr<ar_msgs::msg::Joy_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ar_msgs__msg__Joy
    std::shared_ptr<ar_msgs::msg::Joy_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Joy_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->button_pressed != other.button_pressed) {
      return false;
    }
    return true;
  }
  bool operator!=(const Joy_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Joy_

// alias to use template instance with default allocator
using Joy =
  ar_msgs::msg::Joy_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ar_msgs

#endif  // AR_MSGS__MSG__DETAIL__JOY__STRUCT_HPP_
