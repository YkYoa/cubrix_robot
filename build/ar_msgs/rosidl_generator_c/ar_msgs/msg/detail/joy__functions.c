// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ar_msgs:msg/Joy.idl
// generated code does not contain a copyright notice
#include "ar_msgs/msg/detail/joy__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ar_msgs__msg__Joy__init(ar_msgs__msg__Joy * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // button_pressed
  return true;
}

void
ar_msgs__msg__Joy__fini(ar_msgs__msg__Joy * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // button_pressed
}

bool
ar_msgs__msg__Joy__are_equal(const ar_msgs__msg__Joy * lhs, const ar_msgs__msg__Joy * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // button_pressed
  if (lhs->button_pressed != rhs->button_pressed) {
    return false;
  }
  return true;
}

bool
ar_msgs__msg__Joy__copy(
  const ar_msgs__msg__Joy * input,
  ar_msgs__msg__Joy * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // button_pressed
  output->button_pressed = input->button_pressed;
  return true;
}

ar_msgs__msg__Joy *
ar_msgs__msg__Joy__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_msgs__msg__Joy * msg = (ar_msgs__msg__Joy *)allocator.allocate(sizeof(ar_msgs__msg__Joy), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ar_msgs__msg__Joy));
  bool success = ar_msgs__msg__Joy__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ar_msgs__msg__Joy__destroy(ar_msgs__msg__Joy * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ar_msgs__msg__Joy__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ar_msgs__msg__Joy__Sequence__init(ar_msgs__msg__Joy__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_msgs__msg__Joy * data = NULL;

  if (size) {
    data = (ar_msgs__msg__Joy *)allocator.zero_allocate(size, sizeof(ar_msgs__msg__Joy), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ar_msgs__msg__Joy__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ar_msgs__msg__Joy__fini(&data[i - 1]);
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
ar_msgs__msg__Joy__Sequence__fini(ar_msgs__msg__Joy__Sequence * array)
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
      ar_msgs__msg__Joy__fini(&array->data[i]);
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

ar_msgs__msg__Joy__Sequence *
ar_msgs__msg__Joy__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ar_msgs__msg__Joy__Sequence * array = (ar_msgs__msg__Joy__Sequence *)allocator.allocate(sizeof(ar_msgs__msg__Joy__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ar_msgs__msg__Joy__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ar_msgs__msg__Joy__Sequence__destroy(ar_msgs__msg__Joy__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ar_msgs__msg__Joy__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ar_msgs__msg__Joy__Sequence__are_equal(const ar_msgs__msg__Joy__Sequence * lhs, const ar_msgs__msg__Joy__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ar_msgs__msg__Joy__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ar_msgs__msg__Joy__Sequence__copy(
  const ar_msgs__msg__Joy__Sequence * input,
  ar_msgs__msg__Joy__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ar_msgs__msg__Joy);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ar_msgs__msg__Joy * data =
      (ar_msgs__msg__Joy *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ar_msgs__msg__Joy__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ar_msgs__msg__Joy__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ar_msgs__msg__Joy__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
