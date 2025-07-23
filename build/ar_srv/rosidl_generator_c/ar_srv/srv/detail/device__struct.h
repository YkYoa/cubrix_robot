// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ar_srv:srv/Device.idl
// generated code does not contain a copyright notice

#ifndef AR_SRV__SRV__DETAIL__DEVICE__STRUCT_H_
#define AR_SRV__SRV__DETAIL__DEVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
// Member 'rgb'
// Member 'percentage'
#include "rosidl_runtime_c/string.h"
// Member 'port'
// Member 'valve_ids'
// Member 'valve_states'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/Device in the package ar_srv.
typedef struct ar_srv__srv__Device_Request
{
  rosidl_runtime_c__String command;
  /// For Idicator
  ///  por for x y z = 0 1 2, if none, get all port data
  rosidl_runtime_c__uint8__Sequence port;
  /// For Iheart
  int8_t strip_id;
  rosidl_runtime_c__String rgb;
  int8_t mode;
  rosidl_runtime_c__String percentage;
  /// For VaccumTrigger
  rosidl_runtime_c__uint8__Sequence valve_ids;
  rosidl_runtime_c__boolean__Sequence valve_states;
} ar_srv__srv__Device_Request;

// Struct for a sequence of ar_srv__srv__Device_Request.
typedef struct ar_srv__srv__Device_Request__Sequence
{
  ar_srv__srv__Device_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_srv__srv__Device_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'indicator_data'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"
// Member 'iheartdata'
// Member 'vacuumdata'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Device in the package ar_srv.
typedef struct ar_srv__srv__Device_Response
{
  bool success;
  rosidl_runtime_c__float__Sequence indicator_data;
  rosidl_runtime_c__String__Sequence iheartdata;
  rosidl_runtime_c__String__Sequence vacuumdata;
} ar_srv__srv__Device_Response;

// Struct for a sequence of ar_srv__srv__Device_Response.
typedef struct ar_srv__srv__Device_Response__Sequence
{
  ar_srv__srv__Device_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_srv__srv__Device_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AR_SRV__SRV__DETAIL__DEVICE__STRUCT_H_
