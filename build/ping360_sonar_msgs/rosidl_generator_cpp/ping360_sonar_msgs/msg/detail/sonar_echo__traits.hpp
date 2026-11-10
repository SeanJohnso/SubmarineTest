// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ping360_sonar_msgs:msg/SonarEcho.idl
// generated code does not contain a copyright notice

#ifndef PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__TRAITS_HPP_
#define PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ping360_sonar_msgs/msg/detail/sonar_echo__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ping360_sonar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SonarEcho & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: gain
  {
    out << "gain: ";
    rosidl_generator_traits::value_to_yaml(msg.gain, out);
    out << ", ";
  }

  // member: number_of_samples
  {
    out << "number_of_samples: ";
    rosidl_generator_traits::value_to_yaml(msg.number_of_samples, out);
    out << ", ";
  }

  // member: transmit_frequency
  {
    out << "transmit_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.transmit_frequency, out);
    out << ", ";
  }

  // member: speed_of_sound
  {
    out << "speed_of_sound: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_of_sound, out);
    out << ", ";
  }

  // member: range
  {
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << ", ";
  }

  // member: intensities
  {
    if (msg.intensities.size() == 0) {
      out << "intensities: []";
    } else {
      out << "intensities: [";
      size_t pending_items = msg.intensities.size();
      for (auto item : msg.intensities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SonarEcho & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: gain
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gain: ";
    rosidl_generator_traits::value_to_yaml(msg.gain, out);
    out << "\n";
  }

  // member: number_of_samples
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number_of_samples: ";
    rosidl_generator_traits::value_to_yaml(msg.number_of_samples, out);
    out << "\n";
  }

  // member: transmit_frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transmit_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.transmit_frequency, out);
    out << "\n";
  }

  // member: speed_of_sound
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_of_sound: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_of_sound, out);
    out << "\n";
  }

  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << "\n";
  }

  // member: intensities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.intensities.size() == 0) {
      out << "intensities: []\n";
    } else {
      out << "intensities:\n";
      for (auto item : msg.intensities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SonarEcho & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ping360_sonar_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ping360_sonar_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ping360_sonar_msgs::msg::SonarEcho & msg,
  std::ostream & out, size_t indentation = 0)
{
  ping360_sonar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ping360_sonar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ping360_sonar_msgs::msg::SonarEcho & msg)
{
  return ping360_sonar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ping360_sonar_msgs::msg::SonarEcho>()
{
  return "ping360_sonar_msgs::msg::SonarEcho";
}

template<>
inline const char * name<ping360_sonar_msgs::msg::SonarEcho>()
{
  return "ping360_sonar_msgs/msg/SonarEcho";
}

template<>
struct has_fixed_size<ping360_sonar_msgs::msg::SonarEcho>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ping360_sonar_msgs::msg::SonarEcho>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ping360_sonar_msgs::msg::SonarEcho>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PING360_SONAR_MSGS__MSG__DETAIL__SONAR_ECHO__TRAITS_HPP_
