// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ping360_sonar_msgs:msg/SonarEcho.idl
// generated code does not contain a copyright notice
#include "ping360_sonar_msgs/msg/detail/sonar_echo__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `intensities`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ping360_sonar_msgs__msg__SonarEcho__init(ping360_sonar_msgs__msg__SonarEcho * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ping360_sonar_msgs__msg__SonarEcho__fini(msg);
    return false;
  }
  // angle
  // gain
  // number_of_samples
  // transmit_frequency
  // speed_of_sound
  // range
  // intensities
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->intensities, 0)) {
    ping360_sonar_msgs__msg__SonarEcho__fini(msg);
    return false;
  }
  return true;
}

void
ping360_sonar_msgs__msg__SonarEcho__fini(ping360_sonar_msgs__msg__SonarEcho * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // angle
  // gain
  // number_of_samples
  // transmit_frequency
  // speed_of_sound
  // range
  // intensities
  rosidl_runtime_c__uint8__Sequence__fini(&msg->intensities);
}

bool
ping360_sonar_msgs__msg__SonarEcho__are_equal(const ping360_sonar_msgs__msg__SonarEcho * lhs, const ping360_sonar_msgs__msg__SonarEcho * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // gain
  if (lhs->gain != rhs->gain) {
    return false;
  }
  // number_of_samples
  if (lhs->number_of_samples != rhs->number_of_samples) {
    return false;
  }
  // transmit_frequency
  if (lhs->transmit_frequency != rhs->transmit_frequency) {
    return false;
  }
  // speed_of_sound
  if (lhs->speed_of_sound != rhs->speed_of_sound) {
    return false;
  }
  // range
  if (lhs->range != rhs->range) {
    return false;
  }
  // intensities
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->intensities), &(rhs->intensities)))
  {
    return false;
  }
  return true;
}

bool
ping360_sonar_msgs__msg__SonarEcho__copy(
  const ping360_sonar_msgs__msg__SonarEcho * input,
  ping360_sonar_msgs__msg__SonarEcho * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // angle
  output->angle = input->angle;
  // gain
  output->gain = input->gain;
  // number_of_samples
  output->number_of_samples = input->number_of_samples;
  // transmit_frequency
  output->transmit_frequency = input->transmit_frequency;
  // speed_of_sound
  output->speed_of_sound = input->speed_of_sound;
  // range
  output->range = input->range;
  // intensities
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->intensities), &(output->intensities)))
  {
    return false;
  }
  return true;
}

ping360_sonar_msgs__msg__SonarEcho *
ping360_sonar_msgs__msg__SonarEcho__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ping360_sonar_msgs__msg__SonarEcho * msg = (ping360_sonar_msgs__msg__SonarEcho *)allocator.allocate(sizeof(ping360_sonar_msgs__msg__SonarEcho), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ping360_sonar_msgs__msg__SonarEcho));
  bool success = ping360_sonar_msgs__msg__SonarEcho__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ping360_sonar_msgs__msg__SonarEcho__destroy(ping360_sonar_msgs__msg__SonarEcho * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ping360_sonar_msgs__msg__SonarEcho__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ping360_sonar_msgs__msg__SonarEcho__Sequence__init(ping360_sonar_msgs__msg__SonarEcho__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ping360_sonar_msgs__msg__SonarEcho * data = NULL;

  if (size) {
    data = (ping360_sonar_msgs__msg__SonarEcho *)allocator.zero_allocate(size, sizeof(ping360_sonar_msgs__msg__SonarEcho), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ping360_sonar_msgs__msg__SonarEcho__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ping360_sonar_msgs__msg__SonarEcho__fini(&data[i - 1]);
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
ping360_sonar_msgs__msg__SonarEcho__Sequence__fini(ping360_sonar_msgs__msg__SonarEcho__Sequence * array)
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
      ping360_sonar_msgs__msg__SonarEcho__fini(&array->data[i]);
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

ping360_sonar_msgs__msg__SonarEcho__Sequence *
ping360_sonar_msgs__msg__SonarEcho__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ping360_sonar_msgs__msg__SonarEcho__Sequence * array = (ping360_sonar_msgs__msg__SonarEcho__Sequence *)allocator.allocate(sizeof(ping360_sonar_msgs__msg__SonarEcho__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ping360_sonar_msgs__msg__SonarEcho__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ping360_sonar_msgs__msg__SonarEcho__Sequence__destroy(ping360_sonar_msgs__msg__SonarEcho__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ping360_sonar_msgs__msg__SonarEcho__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ping360_sonar_msgs__msg__SonarEcho__Sequence__are_equal(const ping360_sonar_msgs__msg__SonarEcho__Sequence * lhs, const ping360_sonar_msgs__msg__SonarEcho__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ping360_sonar_msgs__msg__SonarEcho__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ping360_sonar_msgs__msg__SonarEcho__Sequence__copy(
  const ping360_sonar_msgs__msg__SonarEcho__Sequence * input,
  ping360_sonar_msgs__msg__SonarEcho__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ping360_sonar_msgs__msg__SonarEcho);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ping360_sonar_msgs__msg__SonarEcho * data =
      (ping360_sonar_msgs__msg__SonarEcho *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ping360_sonar_msgs__msg__SonarEcho__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ping360_sonar_msgs__msg__SonarEcho__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ping360_sonar_msgs__msg__SonarEcho__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
