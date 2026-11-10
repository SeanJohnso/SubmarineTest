// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ping360_sonar_msgs:msg/SonarEcho.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ping360_sonar_msgs/msg/detail/sonar_echo__rosidl_typesupport_introspection_c.h"
#include "ping360_sonar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ping360_sonar_msgs/msg/detail/sonar_echo__functions.h"
#include "ping360_sonar_msgs/msg/detail/sonar_echo__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `intensities`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ping360_sonar_msgs__msg__SonarEcho__init(message_memory);
}

void ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_fini_function(void * message_memory)
{
  ping360_sonar_msgs__msg__SonarEcho__fini(message_memory);
}

size_t ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__size_function__SonarEcho__intensities(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__get_const_function__SonarEcho__intensities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__get_function__SonarEcho__intensities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__fetch_function__SonarEcho__intensities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__get_const_function__SonarEcho__intensities(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__assign_function__SonarEcho__intensities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__get_function__SonarEcho__intensities(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__resize_function__SonarEcho__intensities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ping360_sonar_msgs__msg__SonarEcho, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ping360_sonar_msgs__msg__SonarEcho, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gain",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ping360_sonar_msgs__msg__SonarEcho, gain),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "number_of_samples",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ping360_sonar_msgs__msg__SonarEcho, number_of_samples),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "transmit_frequency",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ping360_sonar_msgs__msg__SonarEcho, transmit_frequency),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed_of_sound",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ping360_sonar_msgs__msg__SonarEcho, speed_of_sound),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ping360_sonar_msgs__msg__SonarEcho, range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "intensities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ping360_sonar_msgs__msg__SonarEcho, intensities),  // bytes offset in struct
    NULL,  // default value
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__size_function__SonarEcho__intensities,  // size() function pointer
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__get_const_function__SonarEcho__intensities,  // get_const(index) function pointer
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__get_function__SonarEcho__intensities,  // get(index) function pointer
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__fetch_function__SonarEcho__intensities,  // fetch(index, &value) function pointer
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__assign_function__SonarEcho__intensities,  // assign(index, value) function pointer
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__resize_function__SonarEcho__intensities  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_members = {
  "ping360_sonar_msgs__msg",  // message namespace
  "SonarEcho",  // message name
  8,  // number of fields
  sizeof(ping360_sonar_msgs__msg__SonarEcho),
  ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_member_array,  // message members
  ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_init_function,  // function to initialize message memory (memory has to be allocated)
  ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_type_support_handle = {
  0,
  &ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ping360_sonar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ping360_sonar_msgs, msg, SonarEcho)() {
  ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_type_support_handle.typesupport_identifier) {
    ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ping360_sonar_msgs__msg__SonarEcho__rosidl_typesupport_introspection_c__SonarEcho_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
