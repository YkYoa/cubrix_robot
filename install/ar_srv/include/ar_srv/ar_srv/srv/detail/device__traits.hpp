// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ar_srv:srv/Device.idl
// generated code does not contain a copyright notice

#ifndef AR_SRV__SRV__DETAIL__DEVICE__TRAITS_HPP_
#define AR_SRV__SRV__DETAIL__DEVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ar_srv/srv/detail/device__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ar_srv
{

namespace srv
{

inline void to_flow_style_yaml(
  const Device_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: port
  {
    if (msg.port.size() == 0) {
      out << "port: []";
    } else {
      out << "port: [";
      size_t pending_items = msg.port.size();
      for (auto item : msg.port) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: strip_id
  {
    out << "strip_id: ";
    rosidl_generator_traits::value_to_yaml(msg.strip_id, out);
    out << ", ";
  }

  // member: rgb
  {
    out << "rgb: ";
    rosidl_generator_traits::value_to_yaml(msg.rgb, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: percentage
  {
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
    out << ", ";
  }

  // member: valve_ids
  {
    if (msg.valve_ids.size() == 0) {
      out << "valve_ids: []";
    } else {
      out << "valve_ids: [";
      size_t pending_items = msg.valve_ids.size();
      for (auto item : msg.valve_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: valve_states
  {
    if (msg.valve_states.size() == 0) {
      out << "valve_states: []";
    } else {
      out << "valve_states: [";
      size_t pending_items = msg.valve_states.size();
      for (auto item : msg.valve_states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Device_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.port.size() == 0) {
      out << "port: []\n";
    } else {
      out << "port:\n";
      for (auto item : msg.port) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: strip_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "strip_id: ";
    rosidl_generator_traits::value_to_yaml(msg.strip_id, out);
    out << "\n";
  }

  // member: rgb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rgb: ";
    rosidl_generator_traits::value_to_yaml(msg.rgb, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
    out << "\n";
  }

  // member: valve_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.valve_ids.size() == 0) {
      out << "valve_ids: []\n";
    } else {
      out << "valve_ids:\n";
      for (auto item : msg.valve_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: valve_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.valve_states.size() == 0) {
      out << "valve_states: []\n";
    } else {
      out << "valve_states:\n";
      for (auto item : msg.valve_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Device_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ar_srv

namespace rosidl_generator_traits
{

[[deprecated("use ar_srv::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ar_srv::srv::Device_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ar_srv::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ar_srv::srv::to_yaml() instead")]]
inline std::string to_yaml(const ar_srv::srv::Device_Request & msg)
{
  return ar_srv::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ar_srv::srv::Device_Request>()
{
  return "ar_srv::srv::Device_Request";
}

template<>
inline const char * name<ar_srv::srv::Device_Request>()
{
  return "ar_srv/srv/Device_Request";
}

template<>
struct has_fixed_size<ar_srv::srv::Device_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ar_srv::srv::Device_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ar_srv::srv::Device_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ar_srv
{

namespace srv
{

inline void to_flow_style_yaml(
  const Device_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: indicator_data
  {
    if (msg.indicator_data.size() == 0) {
      out << "indicator_data: []";
    } else {
      out << "indicator_data: [";
      size_t pending_items = msg.indicator_data.size();
      for (auto item : msg.indicator_data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: iheartdata
  {
    if (msg.iheartdata.size() == 0) {
      out << "iheartdata: []";
    } else {
      out << "iheartdata: [";
      size_t pending_items = msg.iheartdata.size();
      for (auto item : msg.iheartdata) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vacuumdata
  {
    if (msg.vacuumdata.size() == 0) {
      out << "vacuumdata: []";
    } else {
      out << "vacuumdata: [";
      size_t pending_items = msg.vacuumdata.size();
      for (auto item : msg.vacuumdata) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Device_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: indicator_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.indicator_data.size() == 0) {
      out << "indicator_data: []\n";
    } else {
      out << "indicator_data:\n";
      for (auto item : msg.indicator_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: iheartdata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.iheartdata.size() == 0) {
      out << "iheartdata: []\n";
    } else {
      out << "iheartdata:\n";
      for (auto item : msg.iheartdata) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vacuumdata
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vacuumdata.size() == 0) {
      out << "vacuumdata: []\n";
    } else {
      out << "vacuumdata:\n";
      for (auto item : msg.vacuumdata) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Device_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ar_srv

namespace rosidl_generator_traits
{

[[deprecated("use ar_srv::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ar_srv::srv::Device_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ar_srv::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ar_srv::srv::to_yaml() instead")]]
inline std::string to_yaml(const ar_srv::srv::Device_Response & msg)
{
  return ar_srv::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ar_srv::srv::Device_Response>()
{
  return "ar_srv::srv::Device_Response";
}

template<>
inline const char * name<ar_srv::srv::Device_Response>()
{
  return "ar_srv/srv/Device_Response";
}

template<>
struct has_fixed_size<ar_srv::srv::Device_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ar_srv::srv::Device_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ar_srv::srv::Device_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ar_srv::srv::Device>()
{
  return "ar_srv::srv::Device";
}

template<>
inline const char * name<ar_srv::srv::Device>()
{
  return "ar_srv/srv/Device";
}

template<>
struct has_fixed_size<ar_srv::srv::Device>
  : std::integral_constant<
    bool,
    has_fixed_size<ar_srv::srv::Device_Request>::value &&
    has_fixed_size<ar_srv::srv::Device_Response>::value
  >
{
};

template<>
struct has_bounded_size<ar_srv::srv::Device>
  : std::integral_constant<
    bool,
    has_bounded_size<ar_srv::srv::Device_Request>::value &&
    has_bounded_size<ar_srv::srv::Device_Response>::value
  >
{
};

template<>
struct is_service<ar_srv::srv::Device>
  : std::true_type
{
};

template<>
struct is_service_request<ar_srv::srv::Device_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ar_srv::srv::Device_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AR_SRV__SRV__DETAIL__DEVICE__TRAITS_HPP_
