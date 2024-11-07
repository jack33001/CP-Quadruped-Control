# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadruped:msg/LegCommand.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

# Member 'position'
# Member 'end_position'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LegCommand(type):
    """Metaclass of message 'LegCommand'."""

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
            module = import_type_support('quadruped')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'quadruped.msg.LegCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__leg_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__leg_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__leg_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__leg_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__leg_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LegCommand(metaclass=Metaclass_LegCommand):
    """Message class 'LegCommand'."""

    __slots__ = [
        '_cmd_type',
        '_position',
        '_end_position',
        '_position_end_time',
        '_position_kp',
        '_position_kd',
        '_total_impulse',
        '_impulse_end_time',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'cmd_type': 'string',
        'position': 'sequence<double>',
        'end_position': 'sequence<double>',
        'position_end_time': 'double',
        'position_kp': 'double',
        'position_kd': 'double',
        'total_impulse': 'double',
        'impulse_end_time': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cmd_type = kwargs.get('cmd_type', str())
        self.position = array.array('d', kwargs.get('position', []))
        self.end_position = array.array('d', kwargs.get('end_position', []))
        self.position_end_time = kwargs.get('position_end_time', float())
        self.position_kp = kwargs.get('position_kp', float())
        self.position_kd = kwargs.get('position_kd', float())
        self.total_impulse = kwargs.get('total_impulse', float())
        self.impulse_end_time = kwargs.get('impulse_end_time', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.cmd_type != other.cmd_type:
            return False
        if self.position != other.position:
            return False
        if self.end_position != other.end_position:
            return False
        if self.position_end_time != other.position_end_time:
            return False
        if self.position_kp != other.position_kp:
            return False
        if self.position_kd != other.position_kd:
            return False
        if self.total_impulse != other.total_impulse:
            return False
        if self.impulse_end_time != other.impulse_end_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cmd_type(self):
        """Message field 'cmd_type'."""
        return self._cmd_type

    @cmd_type.setter
    def cmd_type(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'cmd_type' field must be of type 'str'"
        self._cmd_type = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'position' array.array() must have the type code of 'd'"
                self._position = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'position' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._position = array.array('d', value)

    @builtins.property
    def end_position(self):
        """Message field 'end_position'."""
        return self._end_position

    @end_position.setter
    def end_position(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'end_position' array.array() must have the type code of 'd'"
                self._end_position = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'end_position' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._end_position = array.array('d', value)

    @builtins.property
    def position_end_time(self):
        """Message field 'position_end_time'."""
        return self._position_end_time

    @position_end_time.setter
    def position_end_time(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'position_end_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'position_end_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._position_end_time = value

    @builtins.property
    def position_kp(self):
        """Message field 'position_kp'."""
        return self._position_kp

    @position_kp.setter
    def position_kp(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'position_kp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'position_kp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._position_kp = value

    @builtins.property
    def position_kd(self):
        """Message field 'position_kd'."""
        return self._position_kd

    @position_kd.setter
    def position_kd(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'position_kd' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'position_kd' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._position_kd = value

    @builtins.property
    def total_impulse(self):
        """Message field 'total_impulse'."""
        return self._total_impulse

    @total_impulse.setter
    def total_impulse(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'total_impulse' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'total_impulse' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._total_impulse = value

    @builtins.property
    def impulse_end_time(self):
        """Message field 'impulse_end_time'."""
        return self._impulse_end_time

    @impulse_end_time.setter
    def impulse_end_time(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'impulse_end_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'impulse_end_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._impulse_end_time = value
