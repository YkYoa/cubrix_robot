// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ar_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

#ifndef AR_MSGS__MSG__DETAIL__JOY__STRUCT_H_
#define AR_MSGS__MSG__DETAIL__JOY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Joy in the package ar_msgs.
typedef struct ar_msgs__msg__Joy
{
  float x;
  float y;
  bool button_pressed;
} ar_msgs__msg__Joy;

// Struct for a sequence of ar_msgs__msg__Joy.
typedef struct ar_msgs__msg__Joy__Sequence
{
  ar_msgs__msg__Joy * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ar_msgs__msg__Joy__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AR_MSGS__MSG__DETAIL__JOY__STRUCT_H_
