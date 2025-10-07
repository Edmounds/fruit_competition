// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_control_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_control_interfaces/msg/detail/serial_data__rosidl_typesupport_introspection_c.h"
#include "robot_control_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_control_interfaces/msg/detail/serial_data__functions.h"
#include "robot_control_interfaces/msg/detail/serial_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_interfaces__msg__SerialData__init(message_memory);
}

void robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_fini_function(void * message_memory)
{
  robot_control_interfaces__msg__SerialData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_message_member_array[7] = {
  {
    "linear_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_interfaces__msg__SerialData, linear_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_interfaces__msg__SerialData, angular_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "servo1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_interfaces__msg__SerialData, servo1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "servo2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_interfaces__msg__SerialData, servo2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "servo3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_interfaces__msg__SerialData, servo3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "servo4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_interfaces__msg__SerialData, servo4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "servo5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_interfaces__msg__SerialData, servo5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_message_members = {
  "robot_control_interfaces__msg",  // message namespace
  "SerialData",  // message name
  7,  // number of fields
  sizeof(robot_control_interfaces__msg__SerialData),
  robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_message_member_array,  // message members
  robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_message_type_support_handle = {
  0,
  &robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_interfaces, msg, SerialData)() {
  if (!robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_message_type_support_handle.typesupport_identifier) {
    robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_interfaces__msg__SerialData__rosidl_typesupport_introspection_c__SerialData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
