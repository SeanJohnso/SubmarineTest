// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ping360_sonar_msgs:msg/SonarEcho.idl
// generated code does not contain a copyright notice

#ifndef PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__BUILDER_HPP_
#define PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ping360_sonar_msgs/msg/detail/sonar_echo__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ping360_sonar_msgs
{

namespace msg
{

namespace builder
{

class Init_SonarEcho_intensities
{
public:
  explicit Init_SonarEcho_intensities(::ping360_sonar_msgs::msg::SonarEcho & msg)
  : msg_(msg)
  {}
  ::ping360_sonar_msgs::msg::SonarEcho intensities(::ping360_sonar_msgs::msg::SonarEcho::_intensities_type arg)
  {
    msg_.intensities = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ping360_sonar_msgs::msg::SonarEcho msg_;
};

class Init_SonarEcho_range
{
public:
  explicit Init_SonarEcho_range(::ping360_sonar_msgs::msg::SonarEcho & msg)
  : msg_(msg)
  {}
  Init_SonarEcho_intensities range(::ping360_sonar_msgs::msg::SonarEcho::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_SonarEcho_intensities(msg_);
  }

private:
  ::ping360_sonar_msgs::msg::SonarEcho msg_;
};

class Init_SonarEcho_speed_of_sound
{
public:
  explicit Init_SonarEcho_speed_of_sound(::ping360_sonar_msgs::msg::SonarEcho & msg)
  : msg_(msg)
  {}
  Init_SonarEcho_range speed_of_sound(::ping360_sonar_msgs::msg::SonarEcho::_speed_of_sound_type arg)
  {
    msg_.speed_of_sound = std::move(arg);
    return Init_SonarEcho_range(msg_);
  }

private:
  ::ping360_sonar_msgs::msg::SonarEcho msg_;
};

class Init_SonarEcho_transmit_frequency
{
public:
  explicit Init_SonarEcho_transmit_frequency(::ping360_sonar_msgs::msg::SonarEcho & msg)
  : msg_(msg)
  {}
  Init_SonarEcho_speed_of_sound transmit_frequency(::ping360_sonar_msgs::msg::SonarEcho::_transmit_frequency_type arg)
  {
    msg_.transmit_frequency = std::move(arg);
    return Init_SonarEcho_speed_of_sound(msg_);
  }

private:
  ::ping360_sonar_msgs::msg::SonarEcho msg_;
};

class Init_SonarEcho_number_of_samples
{
public:
  explicit Init_SonarEcho_number_of_samples(::ping360_sonar_msgs::msg::SonarEcho & msg)
  : msg_(msg)
  {}
  Init_SonarEcho_transmit_frequency number_of_samples(::ping360_sonar_msgs::msg::SonarEcho::_number_of_samples_type arg)
  {
    msg_.number_of_samples = std::move(arg);
    return Init_SonarEcho_transmit_frequency(msg_);
  }

private:
  ::ping360_sonar_msgs::msg::SonarEcho msg_;
};

class Init_SonarEcho_gain
{
public:
  explicit Init_SonarEcho_gain(::ping360_sonar_msgs::msg::SonarEcho & msg)
  : msg_(msg)
  {}
  Init_SonarEcho_number_of_samples gain(::ping360_sonar_msgs::msg::SonarEcho::_gain_type arg)
  {
    msg_.gain = std::move(arg);
    return Init_SonarEcho_number_of_samples(msg_);
  }

private:
  ::ping360_sonar_msgs::msg::SonarEcho msg_;
};

class Init_SonarEcho_angle
{
public:
  explicit Init_SonarEcho_angle(::ping360_sonar_msgs::msg::SonarEcho & msg)
  : msg_(msg)
  {}
  Init_SonarEcho_gain angle(::ping360_sonar_msgs::msg::SonarEcho::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_SonarEcho_gain(msg_);
  }

private:
  ::ping360_sonar_msgs::msg::SonarEcho msg_;
};

class Init_SonarEcho_header
{
public:
  Init_SonarEcho_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SonarEcho_angle header(::ping360_sonar_msgs::msg::SonarEcho::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SonarEcho_angle(msg_);
  }

private:
  ::ping360_sonar_msgs::msg::SonarEcho msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ping360_sonar_msgs::msg::SonarEcho>()
{
  return ping360_sonar_msgs::msg::builder::Init_SonarEcho_header();
}

}  // namespace ping360_sonar_msgs

#endif  // PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__BUILDER_HPP_
