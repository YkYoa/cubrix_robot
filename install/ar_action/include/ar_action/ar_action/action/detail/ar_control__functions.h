// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ar_action:action/ArControl.idl
// generated code does not contain a copyright notice

#ifndef AR_ACTION__ACTION__DETAIL__AR_CONTROL__FUNCTIONS_H_
#define AR_ACTION__ACTION__DETAIL__AR_CONTROL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ar_action/msg/rosidl_generator_c__visibility_control.h"

#include "ar_action/action/detail/ar_control__struct.h"

/// Initialize action/ArControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_action__action__ArControl_Goal
 * )) before or use
 * ar_action__action__ArControl_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Goal__init(ar_action__action__ArControl_Goal * msg);

/// Finalize action/ArControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Goal__fini(ar_action__action__ArControl_Goal * msg);

/// Create action/ArControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_action__action__ArControl_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_Goal *
ar_action__action__ArControl_Goal__create();

/// Destroy action/ArControl message.
/**
 * It calls
 * ar_action__action__ArControl_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Goal__destroy(ar_action__action__ArControl_Goal * msg);

/// Check for action/ArControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Goal__are_equal(const ar_action__action__ArControl_Goal * lhs, const ar_action__action__ArControl_Goal * rhs);

/// Copy a action/ArControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Goal__copy(
  const ar_action__action__ArControl_Goal * input,
  ar_action__action__ArControl_Goal * output);

/// Initialize array of action/ArControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_action__action__ArControl_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Goal__Sequence__init(ar_action__action__ArControl_Goal__Sequence * array, size_t size);

/// Finalize array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Goal__Sequence__fini(ar_action__action__ArControl_Goal__Sequence * array);

/// Create array of action/ArControl messages.
/**
 * It allocates the memory for the array and calls
 * ar_action__action__ArControl_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_Goal__Sequence *
ar_action__action__ArControl_Goal__Sequence__create(size_t size);

/// Destroy array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Goal__Sequence__destroy(ar_action__action__ArControl_Goal__Sequence * array);

/// Check for action/ArControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Goal__Sequence__are_equal(const ar_action__action__ArControl_Goal__Sequence * lhs, const ar_action__action__ArControl_Goal__Sequence * rhs);

/// Copy an array of action/ArControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Goal__Sequence__copy(
  const ar_action__action__ArControl_Goal__Sequence * input,
  ar_action__action__ArControl_Goal__Sequence * output);

/// Initialize action/ArControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_action__action__ArControl_Result
 * )) before or use
 * ar_action__action__ArControl_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Result__init(ar_action__action__ArControl_Result * msg);

/// Finalize action/ArControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Result__fini(ar_action__action__ArControl_Result * msg);

/// Create action/ArControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_action__action__ArControl_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_Result *
ar_action__action__ArControl_Result__create();

/// Destroy action/ArControl message.
/**
 * It calls
 * ar_action__action__ArControl_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Result__destroy(ar_action__action__ArControl_Result * msg);

/// Check for action/ArControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Result__are_equal(const ar_action__action__ArControl_Result * lhs, const ar_action__action__ArControl_Result * rhs);

/// Copy a action/ArControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Result__copy(
  const ar_action__action__ArControl_Result * input,
  ar_action__action__ArControl_Result * output);

/// Initialize array of action/ArControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_action__action__ArControl_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Result__Sequence__init(ar_action__action__ArControl_Result__Sequence * array, size_t size);

/// Finalize array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Result__Sequence__fini(ar_action__action__ArControl_Result__Sequence * array);

/// Create array of action/ArControl messages.
/**
 * It allocates the memory for the array and calls
 * ar_action__action__ArControl_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_Result__Sequence *
ar_action__action__ArControl_Result__Sequence__create(size_t size);

/// Destroy array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Result__Sequence__destroy(ar_action__action__ArControl_Result__Sequence * array);

/// Check for action/ArControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Result__Sequence__are_equal(const ar_action__action__ArControl_Result__Sequence * lhs, const ar_action__action__ArControl_Result__Sequence * rhs);

/// Copy an array of action/ArControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Result__Sequence__copy(
  const ar_action__action__ArControl_Result__Sequence * input,
  ar_action__action__ArControl_Result__Sequence * output);

/// Initialize action/ArControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_action__action__ArControl_Feedback
 * )) before or use
 * ar_action__action__ArControl_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Feedback__init(ar_action__action__ArControl_Feedback * msg);

/// Finalize action/ArControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Feedback__fini(ar_action__action__ArControl_Feedback * msg);

/// Create action/ArControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_action__action__ArControl_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_Feedback *
ar_action__action__ArControl_Feedback__create();

/// Destroy action/ArControl message.
/**
 * It calls
 * ar_action__action__ArControl_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Feedback__destroy(ar_action__action__ArControl_Feedback * msg);

/// Check for action/ArControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Feedback__are_equal(const ar_action__action__ArControl_Feedback * lhs, const ar_action__action__ArControl_Feedback * rhs);

/// Copy a action/ArControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Feedback__copy(
  const ar_action__action__ArControl_Feedback * input,
  ar_action__action__ArControl_Feedback * output);

/// Initialize array of action/ArControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_action__action__ArControl_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Feedback__Sequence__init(ar_action__action__ArControl_Feedback__Sequence * array, size_t size);

/// Finalize array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Feedback__Sequence__fini(ar_action__action__ArControl_Feedback__Sequence * array);

/// Create array of action/ArControl messages.
/**
 * It allocates the memory for the array and calls
 * ar_action__action__ArControl_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_Feedback__Sequence *
ar_action__action__ArControl_Feedback__Sequence__create(size_t size);

/// Destroy array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_Feedback__Sequence__destroy(ar_action__action__ArControl_Feedback__Sequence * array);

/// Check for action/ArControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Feedback__Sequence__are_equal(const ar_action__action__ArControl_Feedback__Sequence * lhs, const ar_action__action__ArControl_Feedback__Sequence * rhs);

/// Copy an array of action/ArControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_Feedback__Sequence__copy(
  const ar_action__action__ArControl_Feedback__Sequence * input,
  ar_action__action__ArControl_Feedback__Sequence * output);

/// Initialize action/ArControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_action__action__ArControl_SendGoal_Request
 * )) before or use
 * ar_action__action__ArControl_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Request__init(ar_action__action__ArControl_SendGoal_Request * msg);

/// Finalize action/ArControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_SendGoal_Request__fini(ar_action__action__ArControl_SendGoal_Request * msg);

/// Create action/ArControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_action__action__ArControl_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_SendGoal_Request *
ar_action__action__ArControl_SendGoal_Request__create();

/// Destroy action/ArControl message.
/**
 * It calls
 * ar_action__action__ArControl_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_SendGoal_Request__destroy(ar_action__action__ArControl_SendGoal_Request * msg);

/// Check for action/ArControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Request__are_equal(const ar_action__action__ArControl_SendGoal_Request * lhs, const ar_action__action__ArControl_SendGoal_Request * rhs);

/// Copy a action/ArControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Request__copy(
  const ar_action__action__ArControl_SendGoal_Request * input,
  ar_action__action__ArControl_SendGoal_Request * output);

/// Initialize array of action/ArControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_action__action__ArControl_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Request__Sequence__init(ar_action__action__ArControl_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_SendGoal_Request__Sequence__fini(ar_action__action__ArControl_SendGoal_Request__Sequence * array);

/// Create array of action/ArControl messages.
/**
 * It allocates the memory for the array and calls
 * ar_action__action__ArControl_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_SendGoal_Request__Sequence *
ar_action__action__ArControl_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_SendGoal_Request__Sequence__destroy(ar_action__action__ArControl_SendGoal_Request__Sequence * array);

/// Check for action/ArControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Request__Sequence__are_equal(const ar_action__action__ArControl_SendGoal_Request__Sequence * lhs, const ar_action__action__ArControl_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/ArControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Request__Sequence__copy(
  const ar_action__action__ArControl_SendGoal_Request__Sequence * input,
  ar_action__action__ArControl_SendGoal_Request__Sequence * output);

/// Initialize action/ArControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_action__action__ArControl_SendGoal_Response
 * )) before or use
 * ar_action__action__ArControl_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Response__init(ar_action__action__ArControl_SendGoal_Response * msg);

/// Finalize action/ArControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_SendGoal_Response__fini(ar_action__action__ArControl_SendGoal_Response * msg);

/// Create action/ArControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_action__action__ArControl_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_SendGoal_Response *
ar_action__action__ArControl_SendGoal_Response__create();

/// Destroy action/ArControl message.
/**
 * It calls
 * ar_action__action__ArControl_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_SendGoal_Response__destroy(ar_action__action__ArControl_SendGoal_Response * msg);

/// Check for action/ArControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Response__are_equal(const ar_action__action__ArControl_SendGoal_Response * lhs, const ar_action__action__ArControl_SendGoal_Response * rhs);

/// Copy a action/ArControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Response__copy(
  const ar_action__action__ArControl_SendGoal_Response * input,
  ar_action__action__ArControl_SendGoal_Response * output);

/// Initialize array of action/ArControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_action__action__ArControl_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Response__Sequence__init(ar_action__action__ArControl_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_SendGoal_Response__Sequence__fini(ar_action__action__ArControl_SendGoal_Response__Sequence * array);

/// Create array of action/ArControl messages.
/**
 * It allocates the memory for the array and calls
 * ar_action__action__ArControl_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_SendGoal_Response__Sequence *
ar_action__action__ArControl_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_SendGoal_Response__Sequence__destroy(ar_action__action__ArControl_SendGoal_Response__Sequence * array);

/// Check for action/ArControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Response__Sequence__are_equal(const ar_action__action__ArControl_SendGoal_Response__Sequence * lhs, const ar_action__action__ArControl_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/ArControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_SendGoal_Response__Sequence__copy(
  const ar_action__action__ArControl_SendGoal_Response__Sequence * input,
  ar_action__action__ArControl_SendGoal_Response__Sequence * output);

/// Initialize action/ArControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_action__action__ArControl_GetResult_Request
 * )) before or use
 * ar_action__action__ArControl_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Request__init(ar_action__action__ArControl_GetResult_Request * msg);

/// Finalize action/ArControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_GetResult_Request__fini(ar_action__action__ArControl_GetResult_Request * msg);

/// Create action/ArControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_action__action__ArControl_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_GetResult_Request *
ar_action__action__ArControl_GetResult_Request__create();

/// Destroy action/ArControl message.
/**
 * It calls
 * ar_action__action__ArControl_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_GetResult_Request__destroy(ar_action__action__ArControl_GetResult_Request * msg);

/// Check for action/ArControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Request__are_equal(const ar_action__action__ArControl_GetResult_Request * lhs, const ar_action__action__ArControl_GetResult_Request * rhs);

/// Copy a action/ArControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Request__copy(
  const ar_action__action__ArControl_GetResult_Request * input,
  ar_action__action__ArControl_GetResult_Request * output);

/// Initialize array of action/ArControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_action__action__ArControl_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Request__Sequence__init(ar_action__action__ArControl_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_GetResult_Request__Sequence__fini(ar_action__action__ArControl_GetResult_Request__Sequence * array);

/// Create array of action/ArControl messages.
/**
 * It allocates the memory for the array and calls
 * ar_action__action__ArControl_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_GetResult_Request__Sequence *
ar_action__action__ArControl_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_GetResult_Request__Sequence__destroy(ar_action__action__ArControl_GetResult_Request__Sequence * array);

/// Check for action/ArControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Request__Sequence__are_equal(const ar_action__action__ArControl_GetResult_Request__Sequence * lhs, const ar_action__action__ArControl_GetResult_Request__Sequence * rhs);

/// Copy an array of action/ArControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Request__Sequence__copy(
  const ar_action__action__ArControl_GetResult_Request__Sequence * input,
  ar_action__action__ArControl_GetResult_Request__Sequence * output);

/// Initialize action/ArControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_action__action__ArControl_GetResult_Response
 * )) before or use
 * ar_action__action__ArControl_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Response__init(ar_action__action__ArControl_GetResult_Response * msg);

/// Finalize action/ArControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_GetResult_Response__fini(ar_action__action__ArControl_GetResult_Response * msg);

/// Create action/ArControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_action__action__ArControl_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_GetResult_Response *
ar_action__action__ArControl_GetResult_Response__create();

/// Destroy action/ArControl message.
/**
 * It calls
 * ar_action__action__ArControl_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_GetResult_Response__destroy(ar_action__action__ArControl_GetResult_Response * msg);

/// Check for action/ArControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Response__are_equal(const ar_action__action__ArControl_GetResult_Response * lhs, const ar_action__action__ArControl_GetResult_Response * rhs);

/// Copy a action/ArControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Response__copy(
  const ar_action__action__ArControl_GetResult_Response * input,
  ar_action__action__ArControl_GetResult_Response * output);

/// Initialize array of action/ArControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_action__action__ArControl_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Response__Sequence__init(ar_action__action__ArControl_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_GetResult_Response__Sequence__fini(ar_action__action__ArControl_GetResult_Response__Sequence * array);

/// Create array of action/ArControl messages.
/**
 * It allocates the memory for the array and calls
 * ar_action__action__ArControl_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_GetResult_Response__Sequence *
ar_action__action__ArControl_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_GetResult_Response__Sequence__destroy(ar_action__action__ArControl_GetResult_Response__Sequence * array);

/// Check for action/ArControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Response__Sequence__are_equal(const ar_action__action__ArControl_GetResult_Response__Sequence * lhs, const ar_action__action__ArControl_GetResult_Response__Sequence * rhs);

/// Copy an array of action/ArControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_GetResult_Response__Sequence__copy(
  const ar_action__action__ArControl_GetResult_Response__Sequence * input,
  ar_action__action__ArControl_GetResult_Response__Sequence * output);

/// Initialize action/ArControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_action__action__ArControl_FeedbackMessage
 * )) before or use
 * ar_action__action__ArControl_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_FeedbackMessage__init(ar_action__action__ArControl_FeedbackMessage * msg);

/// Finalize action/ArControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_FeedbackMessage__fini(ar_action__action__ArControl_FeedbackMessage * msg);

/// Create action/ArControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_action__action__ArControl_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_FeedbackMessage *
ar_action__action__ArControl_FeedbackMessage__create();

/// Destroy action/ArControl message.
/**
 * It calls
 * ar_action__action__ArControl_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_FeedbackMessage__destroy(ar_action__action__ArControl_FeedbackMessage * msg);

/// Check for action/ArControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_FeedbackMessage__are_equal(const ar_action__action__ArControl_FeedbackMessage * lhs, const ar_action__action__ArControl_FeedbackMessage * rhs);

/// Copy a action/ArControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_FeedbackMessage__copy(
  const ar_action__action__ArControl_FeedbackMessage * input,
  ar_action__action__ArControl_FeedbackMessage * output);

/// Initialize array of action/ArControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_action__action__ArControl_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_FeedbackMessage__Sequence__init(ar_action__action__ArControl_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_FeedbackMessage__Sequence__fini(ar_action__action__ArControl_FeedbackMessage__Sequence * array);

/// Create array of action/ArControl messages.
/**
 * It allocates the memory for the array and calls
 * ar_action__action__ArControl_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
ar_action__action__ArControl_FeedbackMessage__Sequence *
ar_action__action__ArControl_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/ArControl messages.
/**
 * It calls
 * ar_action__action__ArControl_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
void
ar_action__action__ArControl_FeedbackMessage__Sequence__destroy(ar_action__action__ArControl_FeedbackMessage__Sequence * array);

/// Check for action/ArControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_FeedbackMessage__Sequence__are_equal(const ar_action__action__ArControl_FeedbackMessage__Sequence * lhs, const ar_action__action__ArControl_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/ArControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_action
bool
ar_action__action__ArControl_FeedbackMessage__Sequence__copy(
  const ar_action__action__ArControl_FeedbackMessage__Sequence * input,
  ar_action__action__ArControl_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AR_ACTION__ACTION__DETAIL__AR_CONTROL__FUNCTIONS_H_
