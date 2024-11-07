// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadruped/msg/leg_command.hpp"


#ifndef QUADRUPED__MSG__DETAIL__LEG_COMMAND__TRAITS_HPP_
#define QUADRUPED__MSG__DETAIL__LEG_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadruped/msg/detail/leg_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace quadruped
{

namespace msg
{

inline void to_flow_style_yaml(
  const LegCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd_type
  {
    out << "cmd_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_type, out);
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: end_position
  {
    if (msg.end_position.size() == 0) {
      out << "end_position: []";
    } else {
      out << "end_position: [";
      size_t pending_items = msg.end_position.size();
      for (auto item : msg.end_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: position_end_time
  {
    out << "position_end_time: ";
    rosidl_generator_traits::value_to_yaml(msg.position_end_time, out);
    out << ", ";
  }

  // member: position_kp
  {
    out << "position_kp: ";
    rosidl_generator_traits::value_to_yaml(msg.position_kp, out);
    out << ", ";
  }

  // member: position_kd
  {
    out << "position_kd: ";
    rosidl_generator_traits::value_to_yaml(msg.position_kd, out);
    out << ", ";
  }

  // member: total_impulse
  {
    out << "total_impulse: ";
    rosidl_generator_traits::value_to_yaml(msg.total_impulse, out);
    out << ", ";
  }

  // member: impulse_end_time
  {
    out << "impulse_end_time: ";
    rosidl_generator_traits::value_to_yaml(msg.impulse_end_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LegCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_type: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_type, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: end_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.end_position.size() == 0) {
      out << "end_position: []\n";
    } else {
      out << "end_position:\n";
      for (auto item : msg.end_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: position_end_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_end_time: ";
    rosidl_generator_traits::value_to_yaml(msg.position_end_time, out);
    out << "\n";
  }

  // member: position_kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_kp: ";
    rosidl_generator_traits::value_to_yaml(msg.position_kp, out);
    out << "\n";
  }

  // member: position_kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_kd: ";
    rosidl_generator_traits::value_to_yaml(msg.position_kd, out);
    out << "\n";
  }

  // member: total_impulse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_impulse: ";
    rosidl_generator_traits::value_to_yaml(msg.total_impulse, out);
    out << "\n";
  }

  // member: impulse_end_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "impulse_end_time: ";
    rosidl_generator_traits::value_to_yaml(msg.impulse_end_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LegCommand & msg, bool use_flow_style = false)
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

}  // namespace quadruped

namespace rosidl_generator_traits
{

[[deprecated("use quadruped::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const quadruped::msg::LegCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadruped::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadruped::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadruped::msg::LegCommand & msg)
{
  return quadruped::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadruped::msg::LegCommand>()
{
  return "quadruped::msg::LegCommand";
}

template<>
inline const char * name<quadruped::msg::LegCommand>()
{
  return "quadruped/msg/LegCommand";
}

template<>
struct has_fixed_size<quadruped::msg::LegCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quadruped::msg::LegCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quadruped::msg::LegCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADRUPED__MSG__DETAIL__LEG_COMMAND__TRAITS_HPP_
