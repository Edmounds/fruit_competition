// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_control_interfaces:msg/SerialData.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "robot_control_interfaces/msg/detail/serial_data__struct.h"
#include "robot_control_interfaces/msg/detail/serial_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_control_interfaces__msg__serial_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robot_control_interfaces.msg._serial_data.SerialData", full_classname_dest, 52) == 0);
  }
  robot_control_interfaces__msg__SerialData * ros_message = _ros_message;
  {  // linear_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "linear_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->linear_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angular_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angular_z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // servo1
    PyObject * field = PyObject_GetAttrString(_pymsg, "servo1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->servo1 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // servo2
    PyObject * field = PyObject_GetAttrString(_pymsg, "servo2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->servo2 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // servo3
    PyObject * field = PyObject_GetAttrString(_pymsg, "servo3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->servo3 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // servo4
    PyObject * field = PyObject_GetAttrString(_pymsg, "servo4");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->servo4 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // servo5
    PyObject * field = PyObject_GetAttrString(_pymsg, "servo5");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->servo5 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_control_interfaces__msg__serial_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SerialData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_control_interfaces.msg._serial_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SerialData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_control_interfaces__msg__SerialData * ros_message = (robot_control_interfaces__msg__SerialData *)raw_ros_message;
  {  // linear_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->linear_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "linear_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angular_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angular_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // servo1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->servo1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "servo1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // servo2
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->servo2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "servo2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // servo3
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->servo3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "servo3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // servo4
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->servo4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "servo4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // servo5
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->servo5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "servo5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
