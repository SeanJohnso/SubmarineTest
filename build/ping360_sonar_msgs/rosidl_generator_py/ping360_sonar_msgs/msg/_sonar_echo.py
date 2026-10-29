# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ping360_sonar_msgs:msg/SonarEcho.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'intensities'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SonarEcho(type):
    """Metaclass of message 'SonarEcho'."""

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
            module = import_type_support('ping360_sonar_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ping360_sonar_msgs.msg.SonarEcho')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sonar_echo
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sonar_echo
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sonar_echo
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sonar_echo
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sonar_echo

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SonarEcho(metaclass=Metaclass_SonarEcho):
    """Message class 'SonarEcho'."""

    __slots__ = [
        '_header',
        '_angle',
        '_gain',
        '_number_of_samples',
        '_transmit_frequency',
        '_speed_of_sound',
        '_range',
        '_intensities',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'angle': 'float',
        'gain': 'uint8',
        'number_of_samples': 'uint16',
        'transmit_frequency': 'uint16',
        'speed_of_sound': 'uint16',
        'range': 'uint8',
        'intensities': 'sequence<uint8>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.angle = kwargs.get('angle', float())
        self.gain = kwargs.get('gain', int())
        self.number_of_samples = kwargs.get('number_of_samples', int())
        self.transmit_frequency = kwargs.get('transmit_frequency', int())
        self.speed_of_sound = kwargs.get('speed_of_sound', int())
        self.range = kwargs.get('range', int())
        self.intensities = array.array('B', kwargs.get('intensities', []))

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
        if self.header != other.header:
            return False
        if self.angle != other.angle:
            return False
        if self.gain != other.gain:
            return False
        if self.number_of_samples != other.number_of_samples:
            return False
        if self.transmit_frequency != other.transmit_frequency:
            return False
        if self.speed_of_sound != other.speed_of_sound:
            return False
        if self.range != other.range:
            return False
        if self.intensities != other.intensities:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def angle(self):
        """Message field 'angle'."""
        return self._angle

    @angle.setter
    def angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle = value

    @builtins.property
    def gain(self):
        """Message field 'gain'."""
        return self._gain

    @gain.setter
    def gain(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gain' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'gain' field must be an unsigned integer in [0, 255]"
        self._gain = value

    @builtins.property
    def number_of_samples(self):
        """Message field 'number_of_samples'."""
        return self._number_of_samples

    @number_of_samples.setter
    def number_of_samples(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'number_of_samples' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'number_of_samples' field must be an unsigned integer in [0, 65535]"
        self._number_of_samples = value

    @builtins.property
    def transmit_frequency(self):
        """Message field 'transmit_frequency'."""
        return self._transmit_frequency

    @transmit_frequency.setter
    def transmit_frequency(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'transmit_frequency' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'transmit_frequency' field must be an unsigned integer in [0, 65535]"
        self._transmit_frequency = value

    @builtins.property
    def speed_of_sound(self):
        """Message field 'speed_of_sound'."""
        return self._speed_of_sound

    @speed_of_sound.setter
    def speed_of_sound(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'speed_of_sound' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'speed_of_sound' field must be an unsigned integer in [0, 65535]"
        self._speed_of_sound = value

    @builtins.property  # noqa: A003
    def range(self):  # noqa: A003
        """Message field 'range'."""
        return self._range

    @range.setter  # noqa: A003
    def range(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'range' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'range' field must be an unsigned integer in [0, 255]"
        self._range = value

    @builtins.property
    def intensities(self):
        """Message field 'intensities'."""
        return self._intensities

    @intensities.setter
    def intensities(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'intensities' array.array() must have the type code of 'B'"
            self._intensities = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'intensities' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._intensities = array.array('B', value)
