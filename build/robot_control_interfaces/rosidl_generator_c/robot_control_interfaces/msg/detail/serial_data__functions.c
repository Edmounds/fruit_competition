// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_control_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice
#include "robot_control_interfaces/msg/detail/serial_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_control_interfaces__msg__SerialData__init(robot_control_interfaces__msg__SerialData * msg)
{
  if (!msg) {
    return false;
  }
  // linear_x
  // angular_z
  // servo1
  // servo2
  // servo3
  // servo4
  // servo5
  return true;
}

void
robot_control_interfaces__msg__SerialData__fini(robot_control_interfaces__msg__SerialData * msg)
{
  if (!msg) {
    return;
  }
  // linear_x
  // angular_z
  // servo1
  // servo2
  // servo3
  // servo4
  // servo5
}

bool
robot_control_interfaces__msg__SerialData__are_equal(const robot_control_interfaces__msg__SerialData * lhs, const robot_control_interfaces__msg__SerialData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // linear_x
  if (lhs->linear_x != rhs->linear_x) {
    return false;
  }
  // angular_z
  if (lhs->angular_z != rhs->angular_z) {
    return false;
  }
  // servo1
  if (lhs->servo1 != rhs->servo1) {
    return false;
  }
  // servo2
  if (lhs->servo2 != rhs->servo2) {
    return false;
  }
  // servo3
  if (lhs->servo3 != rhs->servo3) {
    return false;
  }
  // servo4
  if (lhs->servo4 != rhs->servo4) {
    return false;
  }
  // servo5
  if (lhs->servo5 != rhs->servo5) {
    return false;
  }
  return true;
}

bool
robot_control_interfaces__msg__SerialData__copy(
  const robot_control_interfaces__msg__SerialData * input,
  robot_control_interfaces__msg__SerialData * output)
{
  if (!input || !output) {
    return false;
  }
  // linear_x
  output->linear_x = input->linear_x;
  // angular_z
  output->angular_z = input->angular_z;
  // servo1
  output->servo1 = input->servo1;
  // servo2
  output->servo2 = input->servo2;
  // servo3
  output->servo3 = input->servo3;
  // servo4
  output->servo4 = input->servo4;
  // servo5
  output->servo5 = input->servo5;
  return true;
}

robot_control_interfaces__msg__SerialData *
robot_control_interfaces__msg__SerialData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_interfaces__msg__SerialData * msg = (robot_control_interfaces__msg__SerialData *)allocator.allocate(sizeof(robot_control_interfaces__msg__SerialData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_control_interfaces__msg__SerialData));
  bool success = robot_control_interfaces__msg__SerialData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_control_interfaces__msg__SerialData__destroy(robot_control_interfaces__msg__SerialData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_control_interfaces__msg__SerialData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_control_interfaces__msg__SerialData__Sequence__init(robot_control_interfaces__msg__SerialData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_interfaces__msg__SerialData * data = NULL;

  if (size) {
    data = (robot_control_interfaces__msg__SerialData *)allocator.zero_allocate(size, sizeof(robot_control_interfaces__msg__SerialData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_control_interfaces__msg__SerialData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_control_interfaces__msg__SerialData__fini(&data[i - 1]);
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
robot_control_interfaces__msg__SerialData__Sequence__fini(robot_control_interfaces__msg__SerialData__Sequence * array)
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
      robot_control_interfaces__msg__SerialData__fini(&array->data[i]);
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

robot_control_interfaces__msg__SerialData__Sequence *
robot_control_interfaces__msg__SerialData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_interfaces__msg__SerialData__Sequence * array = (robot_control_interfaces__msg__SerialData__Sequence *)allocator.allocate(sizeof(robot_control_interfaces__msg__SerialData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_control_interfaces__msg__SerialData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_control_interfaces__msg__SerialData__Sequence__destroy(robot_control_interfaces__msg__SerialData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_control_interfaces__msg__SerialData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_control_interfaces__msg__SerialData__Sequence__are_equal(const robot_control_interfaces__msg__SerialData__Sequence * lhs, const robot_control_interfaces__msg__SerialData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_control_interfaces__msg__SerialData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_control_interfaces__msg__SerialData__Sequence__copy(
  const robot_control_interfaces__msg__SerialData__Sequence * input,
  robot_control_interfaces__msg__SerialData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_control_interfaces__msg__SerialData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_control_interfaces__msg__SerialData * data =
      (robot_control_interfaces__msg__SerialData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_control_interfaces__msg__SerialData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_control_interfaces__msg__SerialData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_control_interfaces__msg__SerialData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
