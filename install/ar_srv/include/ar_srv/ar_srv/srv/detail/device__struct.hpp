// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ar_srv:srv/Device.idl
// generated code does not contain a copyright notice

#ifndef AR_SRV__SRV__DETAIL__DEVICE__STRUCT_HPP_
#define AR_SRV__SRV__DETAIL__DEVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ar_srv__srv__Device_Request __attribute__((deprecated))
#else
# define DEPRECATED__ar_srv__srv__Device_Request __declspec(deprecated)
#endif

namespace ar_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Device_Request_
{
  using Type = Device_Request_<ContainerAllocator>;

  explicit Device_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->strip_id = 0;
      this->rgb = "";
      this->mode = 0;
      this->percentage = "";
    }
  }

  explicit Device_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc),
    rgb(_alloc),
    percentage(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->strip_id = 0;
      this->rgb = "";
      this->mode = 0;
      this->percentage = "";
    }
  }

  // field types and members
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;
  using _port_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _port_type port;
  using _strip_id_type =
    int8_t;
  _strip_id_type strip_id;
  using _rgb_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _rgb_type rgb;
  using _mode_type =
    int8_t;
  _mode_type mode;
  using _percentage_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _percentage_type percentage;
  using _valve_ids_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _valve_ids_type valve_ids;
  using _valve_states_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _valve_states_type valve_states;

  // setters for named parameter idiom
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__port(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->port = _arg;
    return *this;
  }
  Type & set__strip_id(
    const int8_t & _arg)
  {
    this->strip_id = _arg;
    return *this;
  }
  Type & set__rgb(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->rgb = _arg;
    return *this;
  }
  Type & set__mode(
    const int8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__percentage(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->percentage = _arg;
    return *this;
  }
  Type & set__valve_ids(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->valve_ids = _arg;
    return *this;
  }
  Type & set__valve_states(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->valve_states = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ar_srv::srv::Device_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ar_srv::srv::Device_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ar_srv::srv::Device_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ar_srv::srv::Device_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ar_srv::srv::Device_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ar_srv::srv::Device_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ar_srv::srv::Device_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ar_srv::srv::Device_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ar_srv::srv::Device_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ar_srv::srv::Device_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ar_srv__srv__Device_Request
    std::shared_ptr<ar_srv::srv::Device_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ar_srv__srv__Device_Request
    std::shared_ptr<ar_srv::srv::Device_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Device_Request_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->port != other.port) {
      return false;
    }
    if (this->strip_id != other.strip_id) {
      return false;
    }
    if (this->rgb != other.rgb) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->percentage != other.percentage) {
      return false;
    }
    if (this->valve_ids != other.valve_ids) {
      return false;
    }
    if (this->valve_states != other.valve_states) {
      return false;
    }
    return true;
  }
  bool operator!=(const Device_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Device_Request_

// alias to use template instance with default allocator
using Device_Request =
  ar_srv::srv::Device_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ar_srv


#ifndef _WIN32
# define DEPRECATED__ar_srv__srv__Device_Response __attribute__((deprecated))
#else
# define DEPRECATED__ar_srv__srv__Device_Response __declspec(deprecated)
#endif

namespace ar_srv
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Device_Response_
{
  using Type = Device_Response_<ContainerAllocator>;

  explicit Device_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Device_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _indicator_data_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _indicator_data_type indicator_data;
  using _iheartdata_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _iheartdata_type iheartdata;
  using _vacuumdata_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _vacuumdata_type vacuumdata;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__indicator_data(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->indicator_data = _arg;
    return *this;
  }
  Type & set__iheartdata(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->iheartdata = _arg;
    return *this;
  }
  Type & set__vacuumdata(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->vacuumdata = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ar_srv::srv::Device_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ar_srv::srv::Device_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ar_srv::srv::Device_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ar_srv::srv::Device_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ar_srv::srv::Device_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ar_srv::srv::Device_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ar_srv::srv::Device_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ar_srv::srv::Device_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ar_srv::srv::Device_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ar_srv::srv::Device_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ar_srv__srv__Device_Response
    std::shared_ptr<ar_srv::srv::Device_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ar_srv__srv__Device_Response
    std::shared_ptr<ar_srv::srv::Device_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Device_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->indicator_data != other.indicator_data) {
      return false;
    }
    if (this->iheartdata != other.iheartdata) {
      return false;
    }
    if (this->vacuumdata != other.vacuumdata) {
      return false;
    }
    return true;
  }
  bool operator!=(const Device_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Device_Response_

// alias to use template instance with default allocator
using Device_Response =
  ar_srv::srv::Device_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ar_srv

namespace ar_srv
{

namespace srv
{

struct Device
{
  using Request = ar_srv::srv::Device_Request;
  using Response = ar_srv::srv::Device_Response;
};

}  // namespace srv

}  // namespace ar_srv

#endif  // AR_SRV__SRV__DETAIL__DEVICE__STRUCT_HPP_
