// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ping360_sonar_msgs:msg/SonarEcho.idl
// generated code does not contain a copyright notice

#ifndef PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__FUNCTIONS_H_
#define PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ping360_sonar_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ping360_sonar_msgs/msg/detail/sonar_echo__struct.h"

/// Initialize msg/SonarEcho message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ping360_sonar_msgs__msg__SonarEcho
 * )) before or use
 * ping360_sonar_msgs__msg__SonarEcho__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
bool
ping360_sonar_msgs__msg__SonarEcho__init(ping360_sonar_msgs__msg__SonarEcho * msg);

/// Finalize msg/SonarEcho message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
void
ping360_sonar_msgs__msg__SonarEcho__fini(ping360_sonar_msgs__msg__SonarEcho * msg);

/// Create msg/SonarEcho message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ping360_sonar_msgs__msg__SonarEcho__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
ping360_sonar_msgs__msg__SonarEcho *
ping360_sonar_msgs__msg__SonarEcho__create();

/// Destroy msg/SonarEcho message.
/**
 * It calls
 * ping360_sonar_msgs__msg__SonarEcho__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
void
ping360_sonar_msgs__msg__SonarEcho__destroy(ping360_sonar_msgs__msg__SonarEcho * msg);

/// Check for msg/SonarEcho message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
bool
ping360_sonar_msgs__msg__SonarEcho__are_equal(const ping360_sonar_msgs__msg__SonarEcho * lhs, const ping360_sonar_msgs__msg__SonarEcho * rhs);

/// Copy a msg/SonarEcho message.
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
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
bool
ping360_sonar_msgs__msg__SonarEcho__copy(
  const ping360_sonar_msgs__msg__SonarEcho * input,
  ping360_sonar_msgs__msg__SonarEcho * output);

/// Initialize array of msg/SonarEcho messages.
/**
 * It allocates the memory for the number of elements and calls
 * ping360_sonar_msgs__msg__SonarEcho__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
bool
ping360_sonar_msgs__msg__SonarEcho__Sequence__init(ping360_sonar_msgs__msg__SonarEcho__Sequence * array, size_t size);

/// Finalize array of msg/SonarEcho messages.
/**
 * It calls
 * ping360_sonar_msgs__msg__SonarEcho__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
void
ping360_sonar_msgs__msg__SonarEcho__Sequence__fini(ping360_sonar_msgs__msg__SonarEcho__Sequence * array);

/// Create array of msg/SonarEcho messages.
/**
 * It allocates the memory for the array and calls
 * ping360_sonar_msgs__msg__SonarEcho__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
ping360_sonar_msgs__msg__SonarEcho__Sequence *
ping360_sonar_msgs__msg__SonarEcho__Sequence__create(size_t size);

/// Destroy array of msg/SonarEcho messages.
/**
 * It calls
 * ping360_sonar_msgs__msg__SonarEcho__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
void
ping360_sonar_msgs__msg__SonarEcho__Sequence__destroy(ping360_sonar_msgs__msg__SonarEcho__Sequence * array);

/// Check for msg/SonarEcho message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
bool
ping360_sonar_msgs__msg__SonarEcho__Sequence__are_equal(const ping360_sonar_msgs__msg__SonarEcho__Sequence * lhs, const ping360_sonar_msgs__msg__SonarEcho__Sequence * rhs);

/// Copy an array of msg/SonarEcho messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ping360_sonar_msgs
bool
ping360_sonar_msgs__msg__SonarEcho__Sequence__copy(
  const ping360_sonar_msgs__msg__SonarEcho__Sequence * input,
  ping360_sonar_msgs__msg__SonarEcho__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__FUNCTIONS_H_
