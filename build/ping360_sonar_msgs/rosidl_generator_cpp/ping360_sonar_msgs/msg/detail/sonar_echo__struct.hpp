// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ping360_sonar_msgs:msg/SonarEcho.idl
// generated code does not contain a copyright notice

#ifndef PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__STRUCT_HPP_
#define PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ping360_sonar_msgs__msg__SonarEcho __attribute__((deprecated))
#else
# define DEPRECATED__ping360_sonar_msgs__msg__SonarEcho __declspec(deprecated)
#endif

namespace ping360_sonar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SonarEcho_
{
  using Type = SonarEcho_<ContainerAllocator>;

  explicit SonarEcho_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->gain = 0;
      this->number_of_samples = 0;
      this->transmit_frequency = 0;
      this->speed_of_sound = 0;
      this->range = 0;
    }
  }

  explicit SonarEcho_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->gain = 0;
      this->number_of_samples = 0;
      this->transmit_frequency = 0;
      this->speed_of_sound = 0;
      this->range = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _angle_type =
    float;
  _angle_type angle;
  using _gain_type =
    uint8_t;
  _gain_type gain;
  using _number_of_samples_type =
    uint16_t;
  _number_of_samples_type number_of_samples;
  using _transmit_frequency_type =
    uint16_t;
  _transmit_frequency_type transmit_frequency;
  using _speed_of_sound_type =
    uint16_t;
  _speed_of_sound_type speed_of_sound;
  using _range_type =
    uint8_t;
  _range_type range;
  using _intensities_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _intensities_type intensities;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__gain(
    const uint8_t & _arg)
  {
    this->gain = _arg;
    return *this;
  }
  Type & set__number_of_samples(
    const uint16_t & _arg)
  {
    this->number_of_samples = _arg;
    return *this;
  }
  Type & set__transmit_frequency(
    const uint16_t & _arg)
  {
    this->transmit_frequency = _arg;
    return *this;
  }
  Type & set__speed_of_sound(
    const uint16_t & _arg)
  {
    this->speed_of_sound = _arg;
    return *this;
  }
  Type & set__range(
    const uint8_t & _arg)
  {
    this->range = _arg;
    return *this;
  }
  Type & set__intensities(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->intensities = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator> *;
  using ConstRawPtr =
    const ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ping360_sonar_msgs__msg__SonarEcho
    std::shared_ptr<ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ping360_sonar_msgs__msg__SonarEcho
    std::shared_ptr<ping360_sonar_msgs::msg::SonarEcho_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SonarEcho_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->gain != other.gain) {
      return false;
    }
    if (this->number_of_samples != other.number_of_samples) {
      return false;
    }
    if (this->transmit_frequency != other.transmit_frequency) {
      return false;
    }
    if (this->speed_of_sound != other.speed_of_sound) {
      return false;
    }
    if (this->range != other.range) {
      return false;
    }
    if (this->intensities != other.intensities) {
      return false;
    }
    return true;
  }
  bool operator!=(const SonarEcho_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SonarEcho_

// alias to use template instance with default allocator
using SonarEcho =
  ping360_sonar_msgs::msg::SonarEcho_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ping360_sonar_msgs

#endif  // PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__STRUCT_HPP_
