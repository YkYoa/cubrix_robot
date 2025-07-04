// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ar_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ar_msgs/msg/detail/joy__rosidl_typesupport_introspection_c.h"
#include "ar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ar_msgs/msg/detail/joy__functions.h"
#include "ar_msgs/msg/detail/joy__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ar_msgs__msg__Joy__init(message_memory);
}

void ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_fini_function(void * message_memory)
{
  ar_msgs__msg__Joy__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_message_member_array[3] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_msgs__msg__Joy, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_msgs__msg__Joy, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "button_pressed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ar_msgs__msg__Joy, button_pressed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_message_members = {
  "ar_msgs__msg",  // message namespace
  "Joy",  // message name
  3,  // number of fields
  sizeof(ar_msgs__msg__Joy),
  ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_message_member_array,  // message members
  ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_init_function,  // function to initialize message memory (memory has to be allocated)
  ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_message_type_support_handle = {
  0,
  &ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ar_msgs, msg, Joy)() {
  if (!ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_message_type_support_handle.typesupport_identifier) {
    ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ar_msgs__msg__Joy__rosidl_typesupport_introspection_c__Joy_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
