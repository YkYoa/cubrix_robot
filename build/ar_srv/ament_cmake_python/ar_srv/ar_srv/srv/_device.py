# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ar_srv:srv/Device.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'port'
# Member 'valve_ids'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Device_Request(type):
    """Metaclass of message 'Device_Request'."""

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
            module = import_type_support('ar_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ar_srv.srv.Device_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__device__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__device__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__device__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__device__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__device__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Device_Request(metaclass=Metaclass_Device_Request):
    """Message class 'Device_Request'."""

    __slots__ = [
        '_command',
        '_port',
        '_strip_id',
        '_rgb',
        '_mode',
        '_percentage',
        '_valve_ids',
        '_valve_states',
    ]

    _fields_and_field_types = {
        'command': 'string',
        'port': 'sequence<uint8>',
        'strip_id': 'int8',
        'rgb': 'string',
        'mode': 'int8',
        'percentage': 'string',
        'valve_ids': 'sequence<uint8>',
        'valve_states': 'sequence<boolean>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.command = kwargs.get('command', str())
        self.port = array.array('B', kwargs.get('port', []))
        self.strip_id = kwargs.get('strip_id', int())
        self.rgb = kwargs.get('rgb', str())
        self.mode = kwargs.get('mode', int())
        self.percentage = kwargs.get('percentage', str())
        self.valve_ids = array.array('B', kwargs.get('valve_ids', []))
        self.valve_states = kwargs.get('valve_states', [])

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
        if self.command != other.command:
            return False
        if self.port != other.port:
            return False
        if self.strip_id != other.strip_id:
            return False
        if self.rgb != other.rgb:
            return False
        if self.mode != other.mode:
            return False
        if self.percentage != other.percentage:
            return False
        if self.valve_ids != other.valve_ids:
            return False
        if self.valve_states != other.valve_states:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'command' field must be of type 'str'"
        self._command = value

    @builtins.property
    def port(self):
        """Message field 'port'."""
        return self._port

    @port.setter
    def port(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'port' array.array() must have the type code of 'B'"
            self._port = value
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
                "The 'port' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._port = array.array('B', value)

    @builtins.property
    def strip_id(self):
        """Message field 'strip_id'."""
        return self._strip_id

    @strip_id.setter
    def strip_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'strip_id' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'strip_id' field must be an integer in [-128, 127]"
        self._strip_id = value

    @builtins.property
    def rgb(self):
        """Message field 'rgb'."""
        return self._rgb

    @rgb.setter
    def rgb(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'rgb' field must be of type 'str'"
        self._rgb = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'mode' field must be an integer in [-128, 127]"
        self._mode = value

    @builtins.property
    def percentage(self):
        """Message field 'percentage'."""
        return self._percentage

    @percentage.setter
    def percentage(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'percentage' field must be of type 'str'"
        self._percentage = value

    @builtins.property
    def valve_ids(self):
        """Message field 'valve_ids'."""
        return self._valve_ids

    @valve_ids.setter
    def valve_ids(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'valve_ids' array.array() must have the type code of 'B'"
            self._valve_ids = value
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
                "The 'valve_ids' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._valve_ids = array.array('B', value)

    @builtins.property
    def valve_states(self):
        """Message field 'valve_states'."""
        return self._valve_states

    @valve_states.setter
    def valve_states(self, value):
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'valve_states' field must be a set or sequence and each value of type 'bool'"
        self._valve_states = value


# Import statements for member types

# Member 'indicator_data'
# already imported above
# import array

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_Device_Response(type):
    """Metaclass of message 'Device_Response'."""

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
            module = import_type_support('ar_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ar_srv.srv.Device_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__device__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__device__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__device__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__device__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__device__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Device_Response(metaclass=Metaclass_Device_Response):
    """Message class 'Device_Response'."""

    __slots__ = [
        '_success',
        '_indicator_data',
        '_iheartdata',
        '_vacuumdata',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'indicator_data': 'sequence<float>',
        'iheartdata': 'sequence<string>',
        'vacuumdata': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.indicator_data = array.array('f', kwargs.get('indicator_data', []))
        self.iheartdata = kwargs.get('iheartdata', [])
        self.vacuumdata = kwargs.get('vacuumdata', [])

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
        if self.success != other.success:
            return False
        if self.indicator_data != other.indicator_data:
            return False
        if self.iheartdata != other.iheartdata:
            return False
        if self.vacuumdata != other.vacuumdata:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def indicator_data(self):
        """Message field 'indicator_data'."""
        return self._indicator_data

    @indicator_data.setter
    def indicator_data(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'indicator_data' array.array() must have the type code of 'f'"
            self._indicator_data = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'indicator_data' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._indicator_data = array.array('f', value)

    @builtins.property
    def iheartdata(self):
        """Message field 'iheartdata'."""
        return self._iheartdata

    @iheartdata.setter
    def iheartdata(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'iheartdata' field must be a set or sequence and each value of type 'str'"
        self._iheartdata = value

    @builtins.property
    def vacuumdata(self):
        """Message field 'vacuumdata'."""
        return self._vacuumdata

    @vacuumdata.setter
    def vacuumdata(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'vacuumdata' field must be a set or sequence and each value of type 'str'"
        self._vacuumdata = value


class Metaclass_Device(type):
    """Metaclass of service 'Device'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ar_srv')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ar_srv.srv.Device')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__device

            from ar_srv.srv import _device
            if _device.Metaclass_Device_Request._TYPE_SUPPORT is None:
                _device.Metaclass_Device_Request.__import_type_support__()
            if _device.Metaclass_Device_Response._TYPE_SUPPORT is None:
                _device.Metaclass_Device_Response.__import_type_support__()


class Device(metaclass=Metaclass_Device):
    from ar_srv.srv._device import Device_Request as Request
    from ar_srv.srv._device import Device_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
