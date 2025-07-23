// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ar_srv:srv/Device.idl
// generated code does not contain a copyright notice

#ifndef AR_SRV__SRV__DETAIL__DEVICE__FUNCTIONS_H_
#define AR_SRV__SRV__DETAIL__DEVICE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ar_srv/msg/rosidl_generator_c__visibility_control.h"

#include "ar_srv/srv/detail/device__struct.h"

/// Initialize srv/Device message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_srv__srv__Device_Request
 * )) before or use
 * ar_srv__srv__Device_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Request__init(ar_srv__srv__Device_Request * msg);

/// Finalize srv/Device message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
void
ar_srv__srv__Device_Request__fini(ar_srv__srv__Device_Request * msg);

/// Create srv/Device message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_srv__srv__Device_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
ar_srv__srv__Device_Request *
ar_srv__srv__Device_Request__create();

/// Destroy srv/Device message.
/**
 * It calls
 * ar_srv__srv__Device_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
void
ar_srv__srv__Device_Request__destroy(ar_srv__srv__Device_Request * msg);

/// Check for srv/Device message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Request__are_equal(const ar_srv__srv__Device_Request * lhs, const ar_srv__srv__Device_Request * rhs);

/// Copy a srv/Device message.
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
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Request__copy(
  const ar_srv__srv__Device_Request * input,
  ar_srv__srv__Device_Request * output);

/// Initialize array of srv/Device messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_srv__srv__Device_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Request__Sequence__init(ar_srv__srv__Device_Request__Sequence * array, size_t size);

/// Finalize array of srv/Device messages.
/**
 * It calls
 * ar_srv__srv__Device_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
void
ar_srv__srv__Device_Request__Sequence__fini(ar_srv__srv__Device_Request__Sequence * array);

/// Create array of srv/Device messages.
/**
 * It allocates the memory for the array and calls
 * ar_srv__srv__Device_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
ar_srv__srv__Device_Request__Sequence *
ar_srv__srv__Device_Request__Sequence__create(size_t size);

/// Destroy array of srv/Device messages.
/**
 * It calls
 * ar_srv__srv__Device_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
void
ar_srv__srv__Device_Request__Sequence__destroy(ar_srv__srv__Device_Request__Sequence * array);

/// Check for srv/Device message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Request__Sequence__are_equal(const ar_srv__srv__Device_Request__Sequence * lhs, const ar_srv__srv__Device_Request__Sequence * rhs);

/// Copy an array of srv/Device messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Request__Sequence__copy(
  const ar_srv__srv__Device_Request__Sequence * input,
  ar_srv__srv__Device_Request__Sequence * output);

/// Initialize srv/Device message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ar_srv__srv__Device_Response
 * )) before or use
 * ar_srv__srv__Device_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Response__init(ar_srv__srv__Device_Response * msg);

/// Finalize srv/Device message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
void
ar_srv__srv__Device_Response__fini(ar_srv__srv__Device_Response * msg);

/// Create srv/Device message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ar_srv__srv__Device_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
ar_srv__srv__Device_Response *
ar_srv__srv__Device_Response__create();

/// Destroy srv/Device message.
/**
 * It calls
 * ar_srv__srv__Device_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
void
ar_srv__srv__Device_Response__destroy(ar_srv__srv__Device_Response * msg);

/// Check for srv/Device message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Response__are_equal(const ar_srv__srv__Device_Response * lhs, const ar_srv__srv__Device_Response * rhs);

/// Copy a srv/Device message.
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
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Response__copy(
  const ar_srv__srv__Device_Response * input,
  ar_srv__srv__Device_Response * output);

/// Initialize array of srv/Device messages.
/**
 * It allocates the memory for the number of elements and calls
 * ar_srv__srv__Device_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Response__Sequence__init(ar_srv__srv__Device_Response__Sequence * array, size_t size);

/// Finalize array of srv/Device messages.
/**
 * It calls
 * ar_srv__srv__Device_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
void
ar_srv__srv__Device_Response__Sequence__fini(ar_srv__srv__Device_Response__Sequence * array);

/// Create array of srv/Device messages.
/**
 * It allocates the memory for the array and calls
 * ar_srv__srv__Device_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
ar_srv__srv__Device_Response__Sequence *
ar_srv__srv__Device_Response__Sequence__create(size_t size);

/// Destroy array of srv/Device messages.
/**
 * It calls
 * ar_srv__srv__Device_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
void
ar_srv__srv__Device_Response__Sequence__destroy(ar_srv__srv__Device_Response__Sequence * array);

/// Check for srv/Device message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Response__Sequence__are_equal(const ar_srv__srv__Device_Response__Sequence * lhs, const ar_srv__srv__Device_Response__Sequence * rhs);

/// Copy an array of srv/Device messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ar_srv
bool
ar_srv__srv__Device_Response__Sequence__copy(
  const ar_srv__srv__Device_Response__Sequence * input,
  ar_srv__srv__Device_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AR_SRV__SRV__DETAIL__DEVICE__FUNCTIONS_H_
