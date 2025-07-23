// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ar_srv:srv/Device.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ar_srv/srv/detail/device__rosidl_typesupport_introspection_c.h"
#include "ar_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ar_srv/srv/detail/device__functions.h"
#include "ar_srv/srv/detail/device__struct.h"


// Include directives for member types
// Member `command`
// Member `rgb`
// Member `percentage`
#include "rosidl_runtime_c/string_functions.h"
// Member `port`
// Member `valve_ids`
// Member `valve_states`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ar_srv__srv__Device_Request__init(message_memory);
}

void ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_fini_function(void * message_memory)
{
  ar_srv__srv__Device_Request__fini(message_memory);
}

size_t ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__size_function__Device_Request__port(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__port(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__port(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__fetch_function__Device_Request__port(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__port(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__assign_function__Device_Request__port(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__port(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__resize_function__Device_Request__port(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__size_function__Device_Request__valve_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__valve_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__valve_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__fetch_function__Device_Request__valve_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__valve_ids(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__assign_function__Device_Request__valve_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__valve_ids(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__resize_function__Device_Request__valve_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__size_function__Device_Request__valve_states(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__valve_states(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__valve_states(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__fetch_function__Device_Request__valve_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__valve_states(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__assign_function__Device_Request__valve_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__valve_states(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__resize_function__Device_Request__valve_states(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_message_member_array[8] = {
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Request, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "port",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Request, port),  // bytes offset in struct
    NULL,  // default value
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__size_function__Device_Request__port,  // size() function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__port,  // get_const(index) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__port,  // get(index) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__fetch_function__Device_Request__port,  // fetch(index, &value) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__assign_function__Device_Request__port,  // assign(index, value) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__resize_function__Device_Request__port  // resize(index) function pointer
  },
  {
    "strip_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Request, strip_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rgb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Request, rgb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Request, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "percentage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Request, percentage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valve_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Request, valve_ids),  // bytes offset in struct
    NULL,  // default value
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__size_function__Device_Request__valve_ids,  // size() function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__valve_ids,  // get_const(index) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__valve_ids,  // get(index) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__fetch_function__Device_Request__valve_ids,  // fetch(index, &value) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__assign_function__Device_Request__valve_ids,  // assign(index, value) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__resize_function__Device_Request__valve_ids  // resize(index) function pointer
  },
  {
    "valve_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Request, valve_states),  // bytes offset in struct
    NULL,  // default value
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__size_function__Device_Request__valve_states,  // size() function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_const_function__Device_Request__valve_states,  // get_const(index) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__get_function__Device_Request__valve_states,  // get(index) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__fetch_function__Device_Request__valve_states,  // fetch(index, &value) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__assign_function__Device_Request__valve_states,  // assign(index, value) function pointer
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__resize_function__Device_Request__valve_states  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_message_members = {
  "ar_srv__srv",  // message namespace
  "Device_Request",  // message name
  8,  // number of fields
  sizeof(ar_srv__srv__Device_Request),
  ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_message_member_array,  // message members
  ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_message_type_support_handle = {
  0,
  &ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ar_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ar_srv, srv, Device_Request)() {
  if (!ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_message_type_support_handle.typesupport_identifier) {
    ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ar_srv__srv__Device_Request__rosidl_typesupport_introspection_c__Device_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ar_srv/srv/detail/device__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ar_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ar_srv/srv/detail/device__functions.h"
// already included above
// #include "ar_srv/srv/detail/device__struct.h"


// Include directives for member types
// Member `indicator_data`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `iheartdata`
// Member `vacuumdata`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ar_srv__srv__Device_Response__init(message_memory);
}

void ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_fini_function(void * message_memory)
{
  ar_srv__srv__Device_Response__fini(message_memory);
}

size_t ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__size_function__Device_Response__indicator_data(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__indicator_data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__indicator_data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__fetch_function__Device_Response__indicator_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__indicator_data(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__assign_function__Device_Response__indicator_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__indicator_data(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__resize_function__Device_Response__indicator_data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__size_function__Device_Response__iheartdata(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__iheartdata(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__iheartdata(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__fetch_function__Device_Response__iheartdata(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__iheartdata(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__assign_function__Device_Response__iheartdata(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__iheartdata(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__resize_function__Device_Response__iheartdata(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__size_function__Device_Response__vacuumdata(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__vacuumdata(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__vacuumdata(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__fetch_function__Device_Response__vacuumdata(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__vacuumdata(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__assign_function__Device_Response__vacuumdata(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__vacuumdata(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__resize_function__Device_Response__vacuumdata(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_message_member_array[4] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "indicator_data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Response, indicator_data),  // bytes offset in struct
    NULL,  // default value
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__size_function__Device_Response__indicator_data,  // size() function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__indicator_data,  // get_const(index) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__indicator_data,  // get(index) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__fetch_function__Device_Response__indicator_data,  // fetch(index, &value) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__assign_function__Device_Response__indicator_data,  // assign(index, value) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__resize_function__Device_Response__indicator_data  // resize(index) function pointer
  },
  {
    "iheartdata",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Response, iheartdata),  // bytes offset in struct
    NULL,  // default value
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__size_function__Device_Response__iheartdata,  // size() function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__iheartdata,  // get_const(index) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__iheartdata,  // get(index) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__fetch_function__Device_Response__iheartdata,  // fetch(index, &value) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__assign_function__Device_Response__iheartdata,  // assign(index, value) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__resize_function__Device_Response__iheartdata  // resize(index) function pointer
  },
  {
    "vacuumdata",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_srv__srv__Device_Response, vacuumdata),  // bytes offset in struct
    NULL,  // default value
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__size_function__Device_Response__vacuumdata,  // size() function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_const_function__Device_Response__vacuumdata,  // get_const(index) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__get_function__Device_Response__vacuumdata,  // get(index) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__fetch_function__Device_Response__vacuumdata,  // fetch(index, &value) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__assign_function__Device_Response__vacuumdata,  // assign(index, value) function pointer
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__resize_function__Device_Response__vacuumdata  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_message_members = {
  "ar_srv__srv",  // message namespace
  "Device_Response",  // message name
  4,  // number of fields
  sizeof(ar_srv__srv__Device_Response),
  ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_message_member_array,  // message members
  ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_message_type_support_handle = {
  0,
  &ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ar_srv
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ar_srv, srv, Device_Response)() {
  if (!ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_message_type_support_handle.typesupport_identifier) {
    ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ar_srv__srv__Device_Response__rosidl_typesupport_introspection_c__Device_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ar_srv/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ar_srv/srv/detail/device__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_service_members = {
  "ar_srv__srv",  // service namespace
  "Device",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_Request_message_type_support_handle,
  NULL  // response message
  // ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_Response_message_type_support_handle
};

static rosidl_service_type_support_t ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_service_type_support_handle = {
  0,
  &ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ar_srv, srv, Device_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ar_srv, srv, Device_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ar_srv
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ar_srv, srv, Device)() {
  if (!ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_service_type_support_handle.typesupport_identifier) {
    ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ar_srv, srv, Device_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ar_srv, srv, Device_Response)()->data;
  }

  return &ar_srv__srv__detail__device__rosidl_typesupport_introspection_c__Device_service_type_support_handle;
}
