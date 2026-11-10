// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ping360_sonar_msgs:msg/SonarEcho.idl
// generated code does not contain a copyright notice

#ifndef PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__STRUCT_H_
#define PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'intensities'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/SonarEcho in the package ping360_sonar_msgs.
typedef struct ping360_sonar_msgs__msg__SonarEcho
{
  /// header info
  std_msgs__msg__Header header;
  /// the measurement angle
  float angle;
  /// Sonar Gain
  uint8_t gain;
  uint16_t number_of_samples;
  uint16_t transmit_frequency;
  uint16_t speed_of_sound;
  /// range value
  uint8_t range;
  /// intensity data.  This is the actual data received from the sonar
  rosidl_runtime_c__uint8__Sequence intensities;
} ping360_sonar_msgs__msg__SonarEcho;

// Struct for a sequence of ping360_sonar_msgs__msg__SonarEcho.
typedef struct ping360_sonar_msgs__msg__SonarEcho__Sequence
{
  ping360_sonar_msgs__msg__SonarEcho * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ping360_sonar_msgs__msg__SonarEcho__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__STRUCT_H_
