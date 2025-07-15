// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ar_action:action/ArControl.idl
// generated code does not contain a copyright notice

#ifndef AR_ACTION__ACTION__DETAIL__AR_CONTROL__STRUCT_H_
#define AR_ACTION__ACTION__DETAIL__AR_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'planning_group'
// Member 'joint_names'
#include "rosidl_runtime_c/string.h"
// Member 'trajectory'
#include "trajectory_msgs/msg/detail/joint_trajectory__struct.h"

/// Struct defined in action/ArControl in the package ar_action.
typedef struct ar_action__action__ArControl_Goal
{
  uint8_t action;
  rosidl_runtime_c__String planning_group;
  trajectory_msgs__msg__JointTrajectory trajectory;
  /// For ui command on off servo
  rosidl_runtime_c__String__Sequence joint_names;
  int32_t drive_id;
  int32_t client_id;
} ar_action__action__ArControl_Goal;

// Struct for a sequence of ar_action__action__ArControl_Goal.
typedef struct ar_action__action__ArControl_Goal__Sequence
{
  ar_action__action__ArControl_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_action__action__ArControl_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/ArControl in the package ar_action.
typedef struct ar_action__action__ArControl_Result
{
  int32_t status;
} ar_action__action__ArControl_Result;

// Struct for a sequence of ar_action__action__ArControl_Result.
typedef struct ar_action__action__ArControl_Result__Sequence
{
  ar_action__action__ArControl_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_action__action__ArControl_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/ArControl in the package ar_action.
typedef struct ar_action__action__ArControl_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} ar_action__action__ArControl_Feedback;

// Struct for a sequence of ar_action__action__ArControl_Feedback.
typedef struct ar_action__action__ArControl_Feedback__Sequence
{
  ar_action__action__ArControl_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_action__action__ArControl_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "ar_action/action/detail/ar_control__struct.h"

/// Struct defined in action/ArControl in the package ar_action.
typedef struct ar_action__action__ArControl_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  ar_action__action__ArControl_Goal goal;
} ar_action__action__ArControl_SendGoal_Request;

// Struct for a sequence of ar_action__action__ArControl_SendGoal_Request.
typedef struct ar_action__action__ArControl_SendGoal_Request__Sequence
{
  ar_action__action__ArControl_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_action__action__ArControl_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ArControl in the package ar_action.
typedef struct ar_action__action__ArControl_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} ar_action__action__ArControl_SendGoal_Response;

// Struct for a sequence of ar_action__action__ArControl_SendGoal_Response.
typedef struct ar_action__action__ArControl_SendGoal_Response__Sequence
{
  ar_action__action__ArControl_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_action__action__ArControl_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ArControl in the package ar_action.
typedef struct ar_action__action__ArControl_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} ar_action__action__ArControl_GetResult_Request;

// Struct for a sequence of ar_action__action__ArControl_GetResult_Request.
typedef struct ar_action__action__ArControl_GetResult_Request__Sequence
{
  ar_action__action__ArControl_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_action__action__ArControl_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "ar_action/action/detail/ar_control__struct.h"

/// Struct defined in action/ArControl in the package ar_action.
typedef struct ar_action__action__ArControl_GetResult_Response
{
  int8_t status;
  ar_action__action__ArControl_Result result;
} ar_action__action__ArControl_GetResult_Response;

// Struct for a sequence of ar_action__action__ArControl_GetResult_Response.
typedef struct ar_action__action__ArControl_GetResult_Response__Sequence
{
  ar_action__action__ArControl_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_action__action__ArControl_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "ar_action/action/detail/ar_control__struct.h"

/// Struct defined in action/ArControl in the package ar_action.
typedef struct ar_action__action__ArControl_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  ar_action__action__ArControl_Feedback feedback;
} ar_action__action__ArControl_FeedbackMessage;

// Struct for a sequence of ar_action__action__ArControl_FeedbackMessage.
typedef struct ar_action__action__ArControl_FeedbackMessage__Sequence
{
  ar_action__action__ArControl_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_action__action__ArControl_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AR_ACTION__ACTION__DETAIL__AR_CONTROL__STRUCT_H_
