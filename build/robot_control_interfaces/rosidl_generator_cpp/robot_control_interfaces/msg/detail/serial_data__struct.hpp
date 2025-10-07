// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_control_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_
#define ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_control_interfaces__msg__SerialData __attribute__((deprecated))
#else
# define DEPRECATED__robot_control_interfaces__msg__SerialData __declspec(deprecated)
#endif

namespace robot_control_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SerialData_
{
  using Type = SerialData_<ContainerAllocator>;

  explicit SerialData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_x = 0.0f;
      this->angular_z = 0.0f;
      this->servo1 = 0;
      this->servo2 = 0;
      this->servo3 = 0;
      this->servo4 = 0;
      this->servo5 = 0;
    }
  }

  explicit SerialData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_x = 0.0f;
      this->angular_z = 0.0f;
      this->servo1 = 0;
      this->servo2 = 0;
      this->servo3 = 0;
      this->servo4 = 0;
      this->servo5 = 0;
    }
  }

  // field types and members
  using _linear_x_type =
    float;
  _linear_x_type linear_x;
  using _angular_z_type =
    float;
  _angular_z_type angular_z;
  using _servo1_type =
    uint16_t;
  _servo1_type servo1;
  using _servo2_type =
    uint16_t;
  _servo2_type servo2;
  using _servo3_type =
    uint16_t;
  _servo3_type servo3;
  using _servo4_type =
    uint16_t;
  _servo4_type servo4;
  using _servo5_type =
    uint16_t;
  _servo5_type servo5;

  // setters for named parameter idiom
  Type & set__linear_x(
    const float & _arg)
  {
    this->linear_x = _arg;
    return *this;
  }
  Type & set__angular_z(
    const float & _arg)
  {
    this->angular_z = _arg;
    return *this;
  }
  Type & set__servo1(
    const uint16_t & _arg)
  {
    this->servo1 = _arg;
    return *this;
  }
  Type & set__servo2(
    const uint16_t & _arg)
  {
    this->servo2 = _arg;
    return *this;
  }
  Type & set__servo3(
    const uint16_t & _arg)
  {
    this->servo3 = _arg;
    return *this;
  }
  Type & set__servo4(
    const uint16_t & _arg)
  {
    this->servo4 = _arg;
    return *this;
  }
  Type & set__servo5(
    const uint16_t & _arg)
  {
    this->servo5 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_control_interfaces::msg::SerialData_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_control_interfaces::msg::SerialData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_control_interfaces::msg::SerialData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_control_interfaces::msg::SerialData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_control_interfaces::msg::SerialData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_control_interfaces::msg::SerialData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_control_interfaces::msg::SerialData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_control_interfaces::msg::SerialData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_control_interfaces::msg::SerialData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_control_interfaces::msg::SerialData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_control_interfaces__msg__SerialData
    std::shared_ptr<robot_control_interfaces::msg::SerialData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_control_interfaces__msg__SerialData
    std::shared_ptr<robot_control_interfaces::msg::SerialData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SerialData_ & other) const
  {
    if (this->linear_x != other.linear_x) {
      return false;
    }
    if (this->angular_z != other.angular_z) {
      return false;
    }
    if (this->servo1 != other.servo1) {
      return false;
    }
    if (this->servo2 != other.servo2) {
      return false;
    }
    if (this->servo3 != other.servo3) {
      return false;
    }
    if (this->servo4 != other.servo4) {
      return false;
    }
    if (this->servo5 != other.servo5) {
      return false;
    }
    return true;
  }
  bool operator!=(const SerialData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SerialData_

// alias to use template instance with default allocator
using SerialData =
  robot_control_interfaces::msg::SerialData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_control_interfaces

#endif  // ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_
