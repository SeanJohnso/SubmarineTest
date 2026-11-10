// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ping360_sonar_msgs:msg/SonarEcho.idl
// generated code does not contain a copyright notice

#ifndef PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ping360_sonar_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ping360_sonar_msgs/msg/detail/sonar_echo__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ping360_sonar_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ping360_sonar_msgs
cdr_serialize(
  const ping360_sonar_msgs::msg::SonarEcho & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ping360_sonar_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ping360_sonar_msgs::msg::SonarEcho & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ping360_sonar_msgs
get_serialized_size(
  const ping360_sonar_msgs::msg::SonarEcho & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ping360_sonar_msgs
max_serialized_size_SonarEcho(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ping360_sonar_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ping360_sonar_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ping360_sonar_msgs, msg, SonarEcho)();

#ifdef __cplusplus
}
#endif

#endif  // PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
