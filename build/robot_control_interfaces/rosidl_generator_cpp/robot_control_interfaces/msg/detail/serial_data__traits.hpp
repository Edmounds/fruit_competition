// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_control_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_
#define ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_control_interfaces/msg/detail/serial_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_control_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SerialData & msg,
  std::ostream & out)
{
  out << "{";
  // member: linear_x
  {
    out << "linear_x: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_x, out);
    out << ", ";
  }

  // member: angular_z
  {
    out << "angular_z: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_z, out);
    out << ", ";
  }

  // member: servo1
  {
    out << "servo1: ";
    rosidl_generator_traits::value_to_yaml(msg.servo1, out);
    out << ", ";
  }

  // member: servo2
  {
    out << "servo2: ";
    rosidl_generator_traits::value_to_yaml(msg.servo2, out);
    out << ", ";
  }

  // member: servo3
  {
    out << "servo3: ";
    rosidl_generator_traits::value_to_yaml(msg.servo3, out);
    out << ", ";
  }

  // member: servo4
  {
    out << "servo4: ";
    rosidl_generator_traits::value_to_yaml(msg.servo4, out);
    out << ", ";
  }

  // member: servo5
  {
    out << "servo5: ";
    rosidl_generator_traits::value_to_yaml(msg.servo5, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SerialData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: linear_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_x: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_x, out);
    out << "\n";
  }

  // member: angular_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_z: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_z, out);
    out << "\n";
  }

  // member: servo1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo1: ";
    rosidl_generator_traits::value_to_yaml(msg.servo1, out);
    out << "\n";
  }

  // member: servo2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo2: ";
    rosidl_generator_traits::value_to_yaml(msg.servo2, out);
    out << "\n";
  }

  // member: servo3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo3: ";
    rosidl_generator_traits::value_to_yaml(msg.servo3, out);
    out << "\n";
  }

  // member: servo4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo4: ";
    rosidl_generator_traits::value_to_yaml(msg.servo4, out);
    out << "\n";
  }

  // member: servo5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo5: ";
    rosidl_generator_traits::value_to_yaml(msg.servo5, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SerialData & msg, bool use_flow_style = false)
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

}  // namespace robot_control_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_control_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_control_interfaces::msg::SerialData & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_control_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_control_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_control_interfaces::msg::SerialData & msg)
{
  return robot_control_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_control_interfaces::msg::SerialData>()
{
  return "robot_control_interfaces::msg::SerialData";
}

template<>
inline const char * name<robot_control_interfaces::msg::SerialData>()
{
  return "robot_control_interfaces/msg/SerialData";
}

template<>
struct has_fixed_size<robot_control_interfaces::msg::SerialData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_control_interfaces::msg::SerialData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_control_interfaces::msg::SerialData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_
