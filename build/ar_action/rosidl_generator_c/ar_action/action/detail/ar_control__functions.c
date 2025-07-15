// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ar_action:action/ArControl.idl
// generated code does not contain a copyright notice
#include "ar_action/action/detail/ar_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `planning_group`
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `trajectory`
#include "trajectory_msgs/msg/detail/joint_trajectory__functions.h"

bool
ar_action__action__ArControl_Goal__init(ar_action__action__ArControl_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // action
  // planning_group
  if (!rosidl_runtime_c__String__init(&msg->planning_group)) {
    ar_action__action__ArControl_Goal__fini(msg);
    return false;
  }
  // trajectory
  if (!trajectory_msgs__msg__JointTrajectory__init(&msg->trajectory)) {
    ar_action__action__ArControl_Goal__fini(msg);
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->joint_names, 0)) {
    ar_action__action__ArControl_Goal__fini(msg);
    return false;
  }
  // drive_id
  // client_id
  return true;
}

void
ar_action__action__ArControl_Goal__fini(ar_action__action__ArControl_Goal * msg)
{
  if (!msg) {
    return;
  }
  // action
  // planning_group
  rosidl_runtime_c__String__fini(&msg->planning_group);
  // trajectory
  trajectory_msgs__msg__JointTrajectory__fini(&msg->trajectory);
  // joint_names
  rosidl_runtime_c__String__Sequence__fini(&msg->joint_names);
  // drive_id
  // client_id
}

bool
ar_action__action__ArControl_Goal__are_equal(const ar_action__action__ArControl_Goal * lhs, const ar_action__action__ArControl_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // action
  if (lhs->action != rhs->action) {
    return false;
  }
  // planning_group
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->planning_group), &(rhs->planning_group)))
  {
    return false;
  }
  // trajectory
  if (!trajectory_msgs__msg__JointTrajectory__are_equal(
      &(lhs->trajectory), &(rhs->trajectory)))
  {
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->joint_names), &(rhs->joint_names)))
  {
    return false;
  }
  // drive_id
  if (lhs->drive_id != rhs->drive_id) {
    return false;
  }
  // client_id
  if (lhs->client_id != rhs->client_id) {
    return false;
  }
  return true;
}

bool
ar_action__action__ArControl_Goal__copy(
  const ar_action__action__ArControl_Goal * input,
  ar_action__action__ArControl_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // action
  output->action = input->action;
  // planning_group
  if (!rosidl_runtime_c__String__copy(
      &(input->planning_group), &(output->planning_group)))
  {
    return false;
  }
  // trajectory
  if (!trajectory_msgs__msg__JointTrajectory__copy(
      &(input->trajectory), &(output->trajectory)))
  {
    return false;
  }
  // joint_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->joint_names), &(output->joint_names)))
  {
    return false;
  }
  // drive_id
  output->drive_id = input->drive_id;
  // client_id
  output->client_id = input->client_id;
  return true;
}

ar_action__action__ArControl_Goal *
ar_action__action__ArControl_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Goal * msg = (ar_action__action__ArControl_Goal *)allocator.allocate(sizeof(ar_action__action__ArControl_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_action__action__ArControl_Goal));
  bool success = ar_action__action__ArControl_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_action__action__ArControl_Goal__destroy(ar_action__action__ArControl_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_action__action__ArControl_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_action__action__ArControl_Goal__Sequence__init(ar_action__action__ArControl_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Goal * data = NULL;

  if (size) {
    data = (ar_action__action__ArControl_Goal *)allocator.zero_allocate(size, sizeof(ar_action__action__ArControl_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_action__action__ArControl_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_action__action__ArControl_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ar_action__action__ArControl_Goal__Sequence__fini(ar_action__action__ArControl_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ar_action__action__ArControl_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ar_action__action__ArControl_Goal__Sequence *
ar_action__action__ArControl_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Goal__Sequence * array = (ar_action__action__ArControl_Goal__Sequence *)allocator.allocate(sizeof(ar_action__action__ArControl_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_action__action__ArControl_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_action__action__ArControl_Goal__Sequence__destroy(ar_action__action__ArControl_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_action__action__ArControl_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_action__action__ArControl_Goal__Sequence__are_equal(const ar_action__action__ArControl_Goal__Sequence * lhs, const ar_action__action__ArControl_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_action__action__ArControl_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_action__action__ArControl_Goal__Sequence__copy(
  const ar_action__action__ArControl_Goal__Sequence * input,
  ar_action__action__ArControl_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_action__action__ArControl_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_action__action__ArControl_Goal * data =
      (ar_action__action__ArControl_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_action__action__ArControl_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_action__action__ArControl_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_action__action__ArControl_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ar_action__action__ArControl_Result__init(ar_action__action__ArControl_Result * msg)
{
  if (!msg) {
    return false;
  }
  // status
  return true;
}

void
ar_action__action__ArControl_Result__fini(ar_action__action__ArControl_Result * msg)
{
  if (!msg) {
    return;
  }
  // status
}

bool
ar_action__action__ArControl_Result__are_equal(const ar_action__action__ArControl_Result * lhs, const ar_action__action__ArControl_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
ar_action__action__ArControl_Result__copy(
  const ar_action__action__ArControl_Result * input,
  ar_action__action__ArControl_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

ar_action__action__ArControl_Result *
ar_action__action__ArControl_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Result * msg = (ar_action__action__ArControl_Result *)allocator.allocate(sizeof(ar_action__action__ArControl_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_action__action__ArControl_Result));
  bool success = ar_action__action__ArControl_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_action__action__ArControl_Result__destroy(ar_action__action__ArControl_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_action__action__ArControl_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_action__action__ArControl_Result__Sequence__init(ar_action__action__ArControl_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Result * data = NULL;

  if (size) {
    data = (ar_action__action__ArControl_Result *)allocator.zero_allocate(size, sizeof(ar_action__action__ArControl_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_action__action__ArControl_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_action__action__ArControl_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ar_action__action__ArControl_Result__Sequence__fini(ar_action__action__ArControl_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ar_action__action__ArControl_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ar_action__action__ArControl_Result__Sequence *
ar_action__action__ArControl_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Result__Sequence * array = (ar_action__action__ArControl_Result__Sequence *)allocator.allocate(sizeof(ar_action__action__ArControl_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_action__action__ArControl_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_action__action__ArControl_Result__Sequence__destroy(ar_action__action__ArControl_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_action__action__ArControl_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_action__action__ArControl_Result__Sequence__are_equal(const ar_action__action__ArControl_Result__Sequence * lhs, const ar_action__action__ArControl_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_action__action__ArControl_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_action__action__ArControl_Result__Sequence__copy(
  const ar_action__action__ArControl_Result__Sequence * input,
  ar_action__action__ArControl_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_action__action__ArControl_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_action__action__ArControl_Result * data =
      (ar_action__action__ArControl_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_action__action__ArControl_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_action__action__ArControl_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_action__action__ArControl_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ar_action__action__ArControl_Feedback__init(ar_action__action__ArControl_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ar_action__action__ArControl_Feedback__fini(ar_action__action__ArControl_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ar_action__action__ArControl_Feedback__are_equal(const ar_action__action__ArControl_Feedback * lhs, const ar_action__action__ArControl_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ar_action__action__ArControl_Feedback__copy(
  const ar_action__action__ArControl_Feedback * input,
  ar_action__action__ArControl_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ar_action__action__ArControl_Feedback *
ar_action__action__ArControl_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Feedback * msg = (ar_action__action__ArControl_Feedback *)allocator.allocate(sizeof(ar_action__action__ArControl_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_action__action__ArControl_Feedback));
  bool success = ar_action__action__ArControl_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_action__action__ArControl_Feedback__destroy(ar_action__action__ArControl_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_action__action__ArControl_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_action__action__ArControl_Feedback__Sequence__init(ar_action__action__ArControl_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Feedback * data = NULL;

  if (size) {
    data = (ar_action__action__ArControl_Feedback *)allocator.zero_allocate(size, sizeof(ar_action__action__ArControl_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_action__action__ArControl_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_action__action__ArControl_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ar_action__action__ArControl_Feedback__Sequence__fini(ar_action__action__ArControl_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ar_action__action__ArControl_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ar_action__action__ArControl_Feedback__Sequence *
ar_action__action__ArControl_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_Feedback__Sequence * array = (ar_action__action__ArControl_Feedback__Sequence *)allocator.allocate(sizeof(ar_action__action__ArControl_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_action__action__ArControl_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_action__action__ArControl_Feedback__Sequence__destroy(ar_action__action__ArControl_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_action__action__ArControl_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_action__action__ArControl_Feedback__Sequence__are_equal(const ar_action__action__ArControl_Feedback__Sequence * lhs, const ar_action__action__ArControl_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_action__action__ArControl_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_action__action__ArControl_Feedback__Sequence__copy(
  const ar_action__action__ArControl_Feedback__Sequence * input,
  ar_action__action__ArControl_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_action__action__ArControl_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_action__action__ArControl_Feedback * data =
      (ar_action__action__ArControl_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_action__action__ArControl_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_action__action__ArControl_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_action__action__ArControl_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "ar_action/action/detail/ar_control__functions.h"

bool
ar_action__action__ArControl_SendGoal_Request__init(ar_action__action__ArControl_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ar_action__action__ArControl_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!ar_action__action__ArControl_Goal__init(&msg->goal)) {
    ar_action__action__ArControl_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
ar_action__action__ArControl_SendGoal_Request__fini(ar_action__action__ArControl_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  ar_action__action__ArControl_Goal__fini(&msg->goal);
}

bool
ar_action__action__ArControl_SendGoal_Request__are_equal(const ar_action__action__ArControl_SendGoal_Request * lhs, const ar_action__action__ArControl_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!ar_action__action__ArControl_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
ar_action__action__ArControl_SendGoal_Request__copy(
  const ar_action__action__ArControl_SendGoal_Request * input,
  ar_action__action__ArControl_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!ar_action__action__ArControl_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

ar_action__action__ArControl_SendGoal_Request *
ar_action__action__ArControl_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_SendGoal_Request * msg = (ar_action__action__ArControl_SendGoal_Request *)allocator.allocate(sizeof(ar_action__action__ArControl_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_action__action__ArControl_SendGoal_Request));
  bool success = ar_action__action__ArControl_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_action__action__ArControl_SendGoal_Request__destroy(ar_action__action__ArControl_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_action__action__ArControl_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_action__action__ArControl_SendGoal_Request__Sequence__init(ar_action__action__ArControl_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_SendGoal_Request * data = NULL;

  if (size) {
    data = (ar_action__action__ArControl_SendGoal_Request *)allocator.zero_allocate(size, sizeof(ar_action__action__ArControl_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_action__action__ArControl_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_action__action__ArControl_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ar_action__action__ArControl_SendGoal_Request__Sequence__fini(ar_action__action__ArControl_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ar_action__action__ArControl_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ar_action__action__ArControl_SendGoal_Request__Sequence *
ar_action__action__ArControl_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_SendGoal_Request__Sequence * array = (ar_action__action__ArControl_SendGoal_Request__Sequence *)allocator.allocate(sizeof(ar_action__action__ArControl_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_action__action__ArControl_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_action__action__ArControl_SendGoal_Request__Sequence__destroy(ar_action__action__ArControl_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_action__action__ArControl_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_action__action__ArControl_SendGoal_Request__Sequence__are_equal(const ar_action__action__ArControl_SendGoal_Request__Sequence * lhs, const ar_action__action__ArControl_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_action__action__ArControl_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_action__action__ArControl_SendGoal_Request__Sequence__copy(
  const ar_action__action__ArControl_SendGoal_Request__Sequence * input,
  ar_action__action__ArControl_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_action__action__ArControl_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_action__action__ArControl_SendGoal_Request * data =
      (ar_action__action__ArControl_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_action__action__ArControl_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_action__action__ArControl_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_action__action__ArControl_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
ar_action__action__ArControl_SendGoal_Response__init(ar_action__action__ArControl_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    ar_action__action__ArControl_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
ar_action__action__ArControl_SendGoal_Response__fini(ar_action__action__ArControl_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
ar_action__action__ArControl_SendGoal_Response__are_equal(const ar_action__action__ArControl_SendGoal_Response * lhs, const ar_action__action__ArControl_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
ar_action__action__ArControl_SendGoal_Response__copy(
  const ar_action__action__ArControl_SendGoal_Response * input,
  ar_action__action__ArControl_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

ar_action__action__ArControl_SendGoal_Response *
ar_action__action__ArControl_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_SendGoal_Response * msg = (ar_action__action__ArControl_SendGoal_Response *)allocator.allocate(sizeof(ar_action__action__ArControl_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_action__action__ArControl_SendGoal_Response));
  bool success = ar_action__action__ArControl_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_action__action__ArControl_SendGoal_Response__destroy(ar_action__action__ArControl_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_action__action__ArControl_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_action__action__ArControl_SendGoal_Response__Sequence__init(ar_action__action__ArControl_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_SendGoal_Response * data = NULL;

  if (size) {
    data = (ar_action__action__ArControl_SendGoal_Response *)allocator.zero_allocate(size, sizeof(ar_action__action__ArControl_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_action__action__ArControl_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_action__action__ArControl_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ar_action__action__ArControl_SendGoal_Response__Sequence__fini(ar_action__action__ArControl_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ar_action__action__ArControl_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ar_action__action__ArControl_SendGoal_Response__Sequence *
ar_action__action__ArControl_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_SendGoal_Response__Sequence * array = (ar_action__action__ArControl_SendGoal_Response__Sequence *)allocator.allocate(sizeof(ar_action__action__ArControl_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_action__action__ArControl_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_action__action__ArControl_SendGoal_Response__Sequence__destroy(ar_action__action__ArControl_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_action__action__ArControl_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_action__action__ArControl_SendGoal_Response__Sequence__are_equal(const ar_action__action__ArControl_SendGoal_Response__Sequence * lhs, const ar_action__action__ArControl_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_action__action__ArControl_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_action__action__ArControl_SendGoal_Response__Sequence__copy(
  const ar_action__action__ArControl_SendGoal_Response__Sequence * input,
  ar_action__action__ArControl_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_action__action__ArControl_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_action__action__ArControl_SendGoal_Response * data =
      (ar_action__action__ArControl_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_action__action__ArControl_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_action__action__ArControl_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_action__action__ArControl_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
ar_action__action__ArControl_GetResult_Request__init(ar_action__action__ArControl_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ar_action__action__ArControl_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
ar_action__action__ArControl_GetResult_Request__fini(ar_action__action__ArControl_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
ar_action__action__ArControl_GetResult_Request__are_equal(const ar_action__action__ArControl_GetResult_Request * lhs, const ar_action__action__ArControl_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
ar_action__action__ArControl_GetResult_Request__copy(
  const ar_action__action__ArControl_GetResult_Request * input,
  ar_action__action__ArControl_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

ar_action__action__ArControl_GetResult_Request *
ar_action__action__ArControl_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_GetResult_Request * msg = (ar_action__action__ArControl_GetResult_Request *)allocator.allocate(sizeof(ar_action__action__ArControl_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_action__action__ArControl_GetResult_Request));
  bool success = ar_action__action__ArControl_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_action__action__ArControl_GetResult_Request__destroy(ar_action__action__ArControl_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_action__action__ArControl_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_action__action__ArControl_GetResult_Request__Sequence__init(ar_action__action__ArControl_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_GetResult_Request * data = NULL;

  if (size) {
    data = (ar_action__action__ArControl_GetResult_Request *)allocator.zero_allocate(size, sizeof(ar_action__action__ArControl_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_action__action__ArControl_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_action__action__ArControl_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ar_action__action__ArControl_GetResult_Request__Sequence__fini(ar_action__action__ArControl_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ar_action__action__ArControl_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ar_action__action__ArControl_GetResult_Request__Sequence *
ar_action__action__ArControl_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_GetResult_Request__Sequence * array = (ar_action__action__ArControl_GetResult_Request__Sequence *)allocator.allocate(sizeof(ar_action__action__ArControl_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_action__action__ArControl_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_action__action__ArControl_GetResult_Request__Sequence__destroy(ar_action__action__ArControl_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_action__action__ArControl_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_action__action__ArControl_GetResult_Request__Sequence__are_equal(const ar_action__action__ArControl_GetResult_Request__Sequence * lhs, const ar_action__action__ArControl_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_action__action__ArControl_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_action__action__ArControl_GetResult_Request__Sequence__copy(
  const ar_action__action__ArControl_GetResult_Request__Sequence * input,
  ar_action__action__ArControl_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_action__action__ArControl_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_action__action__ArControl_GetResult_Request * data =
      (ar_action__action__ArControl_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_action__action__ArControl_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_action__action__ArControl_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_action__action__ArControl_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "ar_action/action/detail/ar_control__functions.h"

bool
ar_action__action__ArControl_GetResult_Response__init(ar_action__action__ArControl_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!ar_action__action__ArControl_Result__init(&msg->result)) {
    ar_action__action__ArControl_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
ar_action__action__ArControl_GetResult_Response__fini(ar_action__action__ArControl_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  ar_action__action__ArControl_Result__fini(&msg->result);
}

bool
ar_action__action__ArControl_GetResult_Response__are_equal(const ar_action__action__ArControl_GetResult_Response * lhs, const ar_action__action__ArControl_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!ar_action__action__ArControl_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
ar_action__action__ArControl_GetResult_Response__copy(
  const ar_action__action__ArControl_GetResult_Response * input,
  ar_action__action__ArControl_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!ar_action__action__ArControl_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

ar_action__action__ArControl_GetResult_Response *
ar_action__action__ArControl_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_GetResult_Response * msg = (ar_action__action__ArControl_GetResult_Response *)allocator.allocate(sizeof(ar_action__action__ArControl_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_action__action__ArControl_GetResult_Response));
  bool success = ar_action__action__ArControl_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_action__action__ArControl_GetResult_Response__destroy(ar_action__action__ArControl_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_action__action__ArControl_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_action__action__ArControl_GetResult_Response__Sequence__init(ar_action__action__ArControl_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_GetResult_Response * data = NULL;

  if (size) {
    data = (ar_action__action__ArControl_GetResult_Response *)allocator.zero_allocate(size, sizeof(ar_action__action__ArControl_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_action__action__ArControl_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_action__action__ArControl_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ar_action__action__ArControl_GetResult_Response__Sequence__fini(ar_action__action__ArControl_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ar_action__action__ArControl_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ar_action__action__ArControl_GetResult_Response__Sequence *
ar_action__action__ArControl_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_GetResult_Response__Sequence * array = (ar_action__action__ArControl_GetResult_Response__Sequence *)allocator.allocate(sizeof(ar_action__action__ArControl_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_action__action__ArControl_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_action__action__ArControl_GetResult_Response__Sequence__destroy(ar_action__action__ArControl_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_action__action__ArControl_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_action__action__ArControl_GetResult_Response__Sequence__are_equal(const ar_action__action__ArControl_GetResult_Response__Sequence * lhs, const ar_action__action__ArControl_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_action__action__ArControl_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_action__action__ArControl_GetResult_Response__Sequence__copy(
  const ar_action__action__ArControl_GetResult_Response__Sequence * input,
  ar_action__action__ArControl_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_action__action__ArControl_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_action__action__ArControl_GetResult_Response * data =
      (ar_action__action__ArControl_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_action__action__ArControl_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_action__action__ArControl_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_action__action__ArControl_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "ar_action/action/detail/ar_control__functions.h"

bool
ar_action__action__ArControl_FeedbackMessage__init(ar_action__action__ArControl_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ar_action__action__ArControl_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!ar_action__action__ArControl_Feedback__init(&msg->feedback)) {
    ar_action__action__ArControl_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
ar_action__action__ArControl_FeedbackMessage__fini(ar_action__action__ArControl_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  ar_action__action__ArControl_Feedback__fini(&msg->feedback);
}

bool
ar_action__action__ArControl_FeedbackMessage__are_equal(const ar_action__action__ArControl_FeedbackMessage * lhs, const ar_action__action__ArControl_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!ar_action__action__ArControl_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
ar_action__action__ArControl_FeedbackMessage__copy(
  const ar_action__action__ArControl_FeedbackMessage * input,
  ar_action__action__ArControl_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!ar_action__action__ArControl_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

ar_action__action__ArControl_FeedbackMessage *
ar_action__action__ArControl_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_FeedbackMessage * msg = (ar_action__action__ArControl_FeedbackMessage *)allocator.allocate(sizeof(ar_action__action__ArControl_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_action__action__ArControl_FeedbackMessage));
  bool success = ar_action__action__ArControl_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_action__action__ArControl_FeedbackMessage__destroy(ar_action__action__ArControl_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_action__action__ArControl_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_action__action__ArControl_FeedbackMessage__Sequence__init(ar_action__action__ArControl_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_FeedbackMessage * data = NULL;

  if (size) {
    data = (ar_action__action__ArControl_FeedbackMessage *)allocator.zero_allocate(size, sizeof(ar_action__action__ArControl_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_action__action__ArControl_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_action__action__ArControl_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ar_action__action__ArControl_FeedbackMessage__Sequence__fini(ar_action__action__ArControl_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ar_action__action__ArControl_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ar_action__action__ArControl_FeedbackMessage__Sequence *
ar_action__action__ArControl_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_action__action__ArControl_FeedbackMessage__Sequence * array = (ar_action__action__ArControl_FeedbackMessage__Sequence *)allocator.allocate(sizeof(ar_action__action__ArControl_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_action__action__ArControl_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_action__action__ArControl_FeedbackMessage__Sequence__destroy(ar_action__action__ArControl_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_action__action__ArControl_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_action__action__ArControl_FeedbackMessage__Sequence__are_equal(const ar_action__action__ArControl_FeedbackMessage__Sequence * lhs, const ar_action__action__ArControl_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_action__action__ArControl_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_action__action__ArControl_FeedbackMessage__Sequence__copy(
  const ar_action__action__ArControl_FeedbackMessage__Sequence * input,
  ar_action__action__ArControl_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_action__action__ArControl_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_action__action__ArControl_FeedbackMessage * data =
      (ar_action__action__ArControl_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_action__action__ArControl_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_action__action__ArControl_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_action__action__ArControl_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
