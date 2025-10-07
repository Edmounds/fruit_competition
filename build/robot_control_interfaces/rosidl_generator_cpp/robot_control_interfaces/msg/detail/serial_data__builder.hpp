// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_control_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_
#define ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_control_interfaces/msg/detail/serial_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_control_interfaces
{

namespace msg
{

namespace builder
{

class Init_SerialData_servo5
{
public:
  explicit Init_SerialData_servo5(::robot_control_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  ::robot_control_interfaces::msg::SerialData servo5(::robot_control_interfaces::msg::SerialData::_servo5_type arg)
  {
    msg_.servo5 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_interfaces::msg::SerialData msg_;
};

class Init_SerialData_servo4
{
public:
  explicit Init_SerialData_servo4(::robot_control_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_servo5 servo4(::robot_control_interfaces::msg::SerialData::_servo4_type arg)
  {
    msg_.servo4 = std::move(arg);
    return Init_SerialData_servo5(msg_);
  }

private:
  ::robot_control_interfaces::msg::SerialData msg_;
};

class Init_SerialData_servo3
{
public:
  explicit Init_SerialData_servo3(::robot_control_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_servo4 servo3(::robot_control_interfaces::msg::SerialData::_servo3_type arg)
  {
    msg_.servo3 = std::move(arg);
    return Init_SerialData_servo4(msg_);
  }

private:
  ::robot_control_interfaces::msg::SerialData msg_;
};

class Init_SerialData_servo2
{
public:
  explicit Init_SerialData_servo2(::robot_control_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_servo3 servo2(::robot_control_interfaces::msg::SerialData::_servo2_type arg)
  {
    msg_.servo2 = std::move(arg);
    return Init_SerialData_servo3(msg_);
  }

private:
  ::robot_control_interfaces::msg::SerialData msg_;
};

class Init_SerialData_servo1
{
public:
  explicit Init_SerialData_servo1(::robot_control_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_servo2 servo1(::robot_control_interfaces::msg::SerialData::_servo1_type arg)
  {
    msg_.servo1 = std::move(arg);
    return Init_SerialData_servo2(msg_);
  }

private:
  ::robot_control_interfaces::msg::SerialData msg_;
};

class Init_SerialData_angular_z
{
public:
  explicit Init_SerialData_angular_z(::robot_control_interfaces::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_servo1 angular_z(::robot_control_interfaces::msg::SerialData::_angular_z_type arg)
  {
    msg_.angular_z = std::move(arg);
    return Init_SerialData_servo1(msg_);
  }

private:
  ::robot_control_interfaces::msg::SerialData msg_;
};

class Init_SerialData_linear_x
{
public:
  Init_SerialData_linear_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SerialData_angular_z linear_x(::robot_control_interfaces::msg::SerialData::_linear_x_type arg)
  {
    msg_.linear_x = std::move(arg);
    return Init_SerialData_angular_z(msg_);
  }

private:
  ::robot_control_interfaces::msg::SerialData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_interfaces::msg::SerialData>()
{
  return robot_control_interfaces::msg::builder::Init_SerialData_linear_x();
}

}  // namespace robot_control_interfaces

#endif  // ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_
