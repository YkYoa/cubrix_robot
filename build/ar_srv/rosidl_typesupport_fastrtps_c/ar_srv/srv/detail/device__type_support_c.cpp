// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ar_srv:srv/Device.idl
// generated code does not contain a copyright notice
#include "ar_srv/srv/detail/device__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ar_srv/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ar_srv/srv/detail/device__struct.h"
#include "ar_srv/srv/detail/device__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // port, valve_ids, valve_states
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // port, valve_ids, valve_states
#include "rosidl_runtime_c/string.h"  // command, percentage, rgb
#include "rosidl_runtime_c/string_functions.h"  // command, percentage, rgb

// forward declare type support functions


using _Device_Request__ros_msg_type = ar_srv__srv__Device_Request;

static bool _Device_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Device_Request__ros_msg_type * ros_message = static_cast<const _Device_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: command
  {
    const rosidl_runtime_c__String * str = &ros_message->command;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: port
  {
    size_t size = ros_message->port.size;
    auto array_ptr = ros_message->port.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: strip_id
  {
    cdr << ros_message->strip_id;
  }

  // Field name: rgb
  {
    const rosidl_runtime_c__String * str = &ros_message->rgb;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: mode
  {
    cdr << ros_message->mode;
  }

  // Field name: percentage
  {
    const rosidl_runtime_c__String * str = &ros_message->percentage;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: valve_ids
  {
    size_t size = ros_message->valve_ids.size;
    auto array_ptr = ros_message->valve_ids.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: valve_states
  {
    size_t size = ros_message->valve_states.size;
    auto array_ptr = ros_message->valve_states.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _Device_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Device_Request__ros_msg_type * ros_message = static_cast<_Device_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: command
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->command.data) {
      rosidl_runtime_c__String__init(&ros_message->command);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->command,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'command'\n");
      return false;
    }
  }

  // Field name: port
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->port.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->port);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->port, size)) {
      fprintf(stderr, "failed to create array for field 'port'");
      return false;
    }
    auto array_ptr = ros_message->port.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: strip_id
  {
    cdr >> ros_message->strip_id;
  }

  // Field name: rgb
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->rgb.data) {
      rosidl_runtime_c__String__init(&ros_message->rgb);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->rgb,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'rgb'\n");
      return false;
    }
  }

  // Field name: mode
  {
    cdr >> ros_message->mode;
  }

  // Field name: percentage
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->percentage.data) {
      rosidl_runtime_c__String__init(&ros_message->percentage);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->percentage,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'percentage'\n");
      return false;
    }
  }

  // Field name: valve_ids
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->valve_ids.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->valve_ids);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->valve_ids, size)) {
      fprintf(stderr, "failed to create array for field 'valve_ids'");
      return false;
    }
    auto array_ptr = ros_message->valve_ids.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: valve_states
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->valve_states.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->valve_states);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->valve_states, size)) {
      fprintf(stderr, "failed to create array for field 'valve_states'");
      return false;
    }
    auto array_ptr = ros_message->valve_states.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ar_srv
size_t get_serialized_size_ar_srv__srv__Device_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Device_Request__ros_msg_type * ros_message = static_cast<const _Device_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name command
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->command.size + 1);
  // field.name port
  {
    size_t array_size = ros_message->port.size;
    auto array_ptr = ros_message->port.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name strip_id
  {
    size_t item_size = sizeof(ros_message->strip_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rgb
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->rgb.size + 1);
  // field.name mode
  {
    size_t item_size = sizeof(ros_message->mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name percentage
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->percentage.size + 1);
  // field.name valve_ids
  {
    size_t array_size = ros_message->valve_ids.size;
    auto array_ptr = ros_message->valve_ids.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name valve_states
  {
    size_t array_size = ros_message->valve_states.size;
    auto array_ptr = ros_message->valve_states.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Device_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ar_srv__srv__Device_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ar_srv
size_t max_serialized_size_ar_srv__srv__Device_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: command
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: port
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: strip_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rgb
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: percentage
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: valve_ids
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: valve_states
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ar_srv__srv__Device_Request;
    is_plain =
      (
      offsetof(DataType, valve_states) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Device_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ar_srv__srv__Device_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Device_Request = {
  "ar_srv::srv",
  "Device_Request",
  _Device_Request__cdr_serialize,
  _Device_Request__cdr_deserialize,
  _Device_Request__get_serialized_size,
  _Device_Request__max_serialized_size
};

static rosidl_message_type_support_t _Device_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Device_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ar_srv, srv, Device_Request)() {
  return &_Device_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ar_srv/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ar_srv/srv/detail/device__struct.h"
// already included above
// #include "ar_srv/srv/detail/device__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"  // indicator_data
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"  // indicator_data
// already included above
// #include "rosidl_runtime_c/string.h"  // iheartdata, vacuumdata
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // iheartdata, vacuumdata

// forward declare type support functions


using _Device_Response__ros_msg_type = ar_srv__srv__Device_Response;

static bool _Device_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Device_Response__ros_msg_type * ros_message = static_cast<const _Device_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: indicator_data
  {
    size_t size = ros_message->indicator_data.size;
    auto array_ptr = ros_message->indicator_data.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: iheartdata
  {
    size_t size = ros_message->iheartdata.size;
    auto array_ptr = ros_message->iheartdata.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: vacuumdata
  {
    size_t size = ros_message->vacuumdata.size;
    auto array_ptr = ros_message->vacuumdata.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  return true;
}

static bool _Device_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Device_Response__ros_msg_type * ros_message = static_cast<_Device_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: indicator_data
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->indicator_data.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->indicator_data);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->indicator_data, size)) {
      fprintf(stderr, "failed to create array for field 'indicator_data'");
      return false;
    }
    auto array_ptr = ros_message->indicator_data.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: iheartdata
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->iheartdata.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->iheartdata);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->iheartdata, size)) {
      fprintf(stderr, "failed to create array for field 'iheartdata'");
      return false;
    }
    auto array_ptr = ros_message->iheartdata.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'iheartdata'\n");
        return false;
      }
    }
  }

  // Field name: vacuumdata
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->vacuumdata.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->vacuumdata);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->vacuumdata, size)) {
      fprintf(stderr, "failed to create array for field 'vacuumdata'");
      return false;
    }
    auto array_ptr = ros_message->vacuumdata.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'vacuumdata'\n");
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ar_srv
size_t get_serialized_size_ar_srv__srv__Device_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Device_Response__ros_msg_type * ros_message = static_cast<const _Device_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name indicator_data
  {
    size_t array_size = ros_message->indicator_data.size;
    auto array_ptr = ros_message->indicator_data.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name iheartdata
  {
    size_t array_size = ros_message->iheartdata.size;
    auto array_ptr = ros_message->iheartdata.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name vacuumdata
  {
    size_t array_size = ros_message->vacuumdata.size;
    auto array_ptr = ros_message->vacuumdata.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Device_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ar_srv__srv__Device_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ar_srv
size_t max_serialized_size_ar_srv__srv__Device_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: indicator_data
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: iheartdata
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: vacuumdata
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ar_srv__srv__Device_Response;
    is_plain =
      (
      offsetof(DataType, vacuumdata) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Device_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ar_srv__srv__Device_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Device_Response = {
  "ar_srv::srv",
  "Device_Response",
  _Device_Response__cdr_serialize,
  _Device_Response__cdr_deserialize,
  _Device_Response__get_serialized_size,
  _Device_Response__max_serialized_size
};

static rosidl_message_type_support_t _Device_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Device_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ar_srv, srv, Device_Response)() {
  return &_Device_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "ar_srv/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ar_srv/srv/device.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t Device__callbacks = {
  "ar_srv::srv",
  "Device",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ar_srv, srv, Device_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ar_srv, srv, Device_Response)(),
};

static rosidl_service_type_support_t Device__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &Device__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ar_srv, srv, Device)() {
  return &Device__handle;
}

#if defined(__cplusplus)
}
#endif
