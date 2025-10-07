# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_control_interfaces:msg/SerialData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SerialData(type):
    """Metaclass of message 'SerialData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_control_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_control_interfaces.msg.SerialData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__serial_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__serial_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__serial_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__serial_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__serial_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SerialData(metaclass=Metaclass_SerialData):
    """Message class 'SerialData'."""

    __slots__ = [
        '_linear_x',
        '_angular_z',
        '_servo1',
        '_servo2',
        '_servo3',
        '_servo4',
        '_servo5',
    ]

    _fields_and_field_types = {
        'linear_x': 'float',
        'angular_z': 'float',
        'servo1': 'uint16',
        'servo2': 'uint16',
        'servo3': 'uint16',
        'servo4': 'uint16',
        'servo5': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.linear_x = kwargs.get('linear_x', float())
        self.angular_z = kwargs.get('angular_z', float())
        self.servo1 = kwargs.get('servo1', int())
        self.servo2 = kwargs.get('servo2', int())
        self.servo3 = kwargs.get('servo3', int())
        self.servo4 = kwargs.get('servo4', int())
        self.servo5 = kwargs.get('servo5', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.linear_x != other.linear_x:
            return False
        if self.angular_z != other.angular_z:
            return False
        if self.servo1 != other.servo1:
            return False
        if self.servo2 != other.servo2:
            return False
        if self.servo3 != other.servo3:
            return False
        if self.servo4 != other.servo4:
            return False
        if self.servo5 != other.servo5:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def linear_x(self):
        """Message field 'linear_x'."""
        return self._linear_x

    @linear_x.setter
    def linear_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'linear_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'linear_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._linear_x = value

    @builtins.property
    def angular_z(self):
        """Message field 'angular_z'."""
        return self._angular_z

    @angular_z.setter
    def angular_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angular_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angular_z = value

    @builtins.property
    def servo1(self):
        """Message field 'servo1'."""
        return self._servo1

    @servo1.setter
    def servo1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'servo1' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'servo1' field must be an unsigned integer in [0, 65535]"
        self._servo1 = value

    @builtins.property
    def servo2(self):
        """Message field 'servo2'."""
        return self._servo2

    @servo2.setter
    def servo2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'servo2' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'servo2' field must be an unsigned integer in [0, 65535]"
        self._servo2 = value

    @builtins.property
    def servo3(self):
        """Message field 'servo3'."""
        return self._servo3

    @servo3.setter
    def servo3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'servo3' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'servo3' field must be an unsigned integer in [0, 65535]"
        self._servo3 = value

    @builtins.property
    def servo4(self):
        """Message field 'servo4'."""
        return self._servo4

    @servo4.setter
    def servo4(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'servo4' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'servo4' field must be an unsigned integer in [0, 65535]"
        self._servo4 = value

    @builtins.property
    def servo5(self):
        """Message field 'servo5'."""
        return self._servo5

    @servo5.setter
    def servo5(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'servo5' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'servo5' field must be an unsigned integer in [0, 65535]"
        self._servo5 = value
