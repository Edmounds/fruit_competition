// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_control_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_H_
#define ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SerialData in the package robot_control_interfaces.
/**
  * This is for fruit_robot
 */
typedef struct robot_control_interfaces__msg__SerialData
{
  ///  nuc ->>>
  /// navigate
  ///  机器人前进/后退线速度 (m/s), 字节位置 3~6
  float linear_x;
  /// 机器人自转角速度 (rad/s), 字节位置 7~10
  float angular_z;
  /// arm
  ///  舵机1 (云台) 目标角度 (0-3600 代表 0-360.0 度), 字节位置 11~12
  uint16_t servo1;
  /// 舵机5 目标角度, 字节位置 13~14
  uint16_t servo2;
  /// 舵机2 目标角度, 字节位置 15~16
  uint16_t servo3;
  /// 舵机4 目标角度, 字节位置 17~18
  uint16_t servo4;
  /// 舵机3 (夹爪) 状态值: 1 代表闭合, 0 代表张开, 字节位置 19~20
  uint16_t servo5;
} robot_control_interfaces__msg__SerialData;

// Struct for a sequence of robot_control_interfaces__msg__SerialData.
typedef struct robot_control_interfaces__msg__SerialData__Sequence
{
  robot_control_interfaces__msg__SerialData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_control_interfaces__msg__SerialData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL_INTERFACES__MSG__DETAIL__SERIAL_DATA__STRUCT_H_
