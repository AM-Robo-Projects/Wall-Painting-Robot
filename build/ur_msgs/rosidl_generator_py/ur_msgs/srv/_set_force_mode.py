# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ur_msgs:srv/SetForceMode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'deviation_limits'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetForceMode_Request(type):
    """Metaclass of message 'SetForceMode_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TCP_TO_ORIGIN': 1,
        'NO_TRANSFORM': 2,
        'TCP_VELOCITY_TO_X_Y': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ur_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ur_msgs.srv.SetForceMode_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_force_mode__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_force_mode__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_force_mode__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_force_mode__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_force_mode__request

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

            from geometry_msgs.msg import Twist
            if Twist.__class__._TYPE_SUPPORT is None:
                Twist.__class__.__import_type_support__()

            from geometry_msgs.msg import Wrench
            if Wrench.__class__._TYPE_SUPPORT is None:
                Wrench.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TCP_TO_ORIGIN': cls.__constants['TCP_TO_ORIGIN'],
            'NO_TRANSFORM': cls.__constants['NO_TRANSFORM'],
            'TCP_VELOCITY_TO_X_Y': cls.__constants['TCP_VELOCITY_TO_X_Y'],
            'SELECTION_VECTOR_X__DEFAULT': False,
            'SELECTION_VECTOR_Y__DEFAULT': False,
            'SELECTION_VECTOR_Z__DEFAULT': False,
            'SELECTION_VECTOR_RX__DEFAULT': False,
            'SELECTION_VECTOR_RY__DEFAULT': False,
            'SELECTION_VECTOR_RZ__DEFAULT': False,
            'TYPE__DEFAULT': 2,
            'DEVIATION_LIMITS__DEFAULT': numpy.array((0.01, 0.01, 0.01, 0.01, 0.01, 0.01, ), dtype=numpy.float32),
            'DAMPING_FACTOR__DEFAULT': 0.025,
            'GAIN_SCALING__DEFAULT': 0.5,
        }

    @property
    def TCP_TO_ORIGIN(self):
        """Message constant 'TCP_TO_ORIGIN'."""
        return Metaclass_SetForceMode_Request.__constants['TCP_TO_ORIGIN']

    @property
    def NO_TRANSFORM(self):
        """Message constant 'NO_TRANSFORM'."""
        return Metaclass_SetForceMode_Request.__constants['NO_TRANSFORM']

    @property
    def TCP_VELOCITY_TO_X_Y(self):
        """Message constant 'TCP_VELOCITY_TO_X_Y'."""
        return Metaclass_SetForceMode_Request.__constants['TCP_VELOCITY_TO_X_Y']

    @property
    def SELECTION_VECTOR_X__DEFAULT(cls):
        """Return default value for message field 'selection_vector_x'."""
        return False

    @property
    def SELECTION_VECTOR_Y__DEFAULT(cls):
        """Return default value for message field 'selection_vector_y'."""
        return False

    @property
    def SELECTION_VECTOR_Z__DEFAULT(cls):
        """Return default value for message field 'selection_vector_z'."""
        return False

    @property
    def SELECTION_VECTOR_RX__DEFAULT(cls):
        """Return default value for message field 'selection_vector_rx'."""
        return False

    @property
    def SELECTION_VECTOR_RY__DEFAULT(cls):
        """Return default value for message field 'selection_vector_ry'."""
        return False

    @property
    def SELECTION_VECTOR_RZ__DEFAULT(cls):
        """Return default value for message field 'selection_vector_rz'."""
        return False

    @property
    def TYPE__DEFAULT(cls):
        """Return default value for message field 'type'."""
        return 2

    @property
    def DEVIATION_LIMITS__DEFAULT(cls):
        """Return default value for message field 'deviation_limits'."""
        return numpy.array((0.01, 0.01, 0.01, 0.01, 0.01, 0.01, ), dtype=numpy.float32)

    @property
    def DAMPING_FACTOR__DEFAULT(cls):
        """Return default value for message field 'damping_factor'."""
        return 0.025

    @property
    def GAIN_SCALING__DEFAULT(cls):
        """Return default value for message field 'gain_scaling'."""
        return 0.5


class SetForceMode_Request(metaclass=Metaclass_SetForceMode_Request):
    """
    Message class 'SetForceMode_Request'.

    Constants:
      TCP_TO_ORIGIN
      NO_TRANSFORM
      TCP_VELOCITY_TO_X_Y
    """

    __slots__ = [
        '_task_frame',
        '_selection_vector_x',
        '_selection_vector_y',
        '_selection_vector_z',
        '_selection_vector_rx',
        '_selection_vector_ry',
        '_selection_vector_rz',
        '_wrench',
        '_type',
        '_speed_limits',
        '_deviation_limits',
        '_damping_factor',
        '_gain_scaling',
    ]

    _fields_and_field_types = {
        'task_frame': 'geometry_msgs/PoseStamped',
        'selection_vector_x': 'boolean',
        'selection_vector_y': 'boolean',
        'selection_vector_z': 'boolean',
        'selection_vector_rx': 'boolean',
        'selection_vector_ry': 'boolean',
        'selection_vector_rz': 'boolean',
        'wrench': 'geometry_msgs/Wrench',
        'type': 'uint8',
        'speed_limits': 'geometry_msgs/Twist',
        'deviation_limits': 'float[6]',
        'damping_factor': 'float',
        'gain_scaling': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Wrench'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 6),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import PoseStamped
        self.task_frame = kwargs.get('task_frame', PoseStamped())
        self.selection_vector_x = kwargs.get(
            'selection_vector_x', SetForceMode_Request.SELECTION_VECTOR_X__DEFAULT)
        self.selection_vector_y = kwargs.get(
            'selection_vector_y', SetForceMode_Request.SELECTION_VECTOR_Y__DEFAULT)
        self.selection_vector_z = kwargs.get(
            'selection_vector_z', SetForceMode_Request.SELECTION_VECTOR_Z__DEFAULT)
        self.selection_vector_rx = kwargs.get(
            'selection_vector_rx', SetForceMode_Request.SELECTION_VECTOR_RX__DEFAULT)
        self.selection_vector_ry = kwargs.get(
            'selection_vector_ry', SetForceMode_Request.SELECTION_VECTOR_RY__DEFAULT)
        self.selection_vector_rz = kwargs.get(
            'selection_vector_rz', SetForceMode_Request.SELECTION_VECTOR_RZ__DEFAULT)
        from geometry_msgs.msg import Wrench
        self.wrench = kwargs.get('wrench', Wrench())
        self.type = kwargs.get(
            'type', SetForceMode_Request.TYPE__DEFAULT)
        from geometry_msgs.msg import Twist
        self.speed_limits = kwargs.get('speed_limits', Twist())
        self.deviation_limits = kwargs.get(
            'deviation_limits', SetForceMode_Request.DEVIATION_LIMITS__DEFAULT)
        self.damping_factor = kwargs.get(
            'damping_factor', SetForceMode_Request.DAMPING_FACTOR__DEFAULT)
        self.gain_scaling = kwargs.get(
            'gain_scaling', SetForceMode_Request.GAIN_SCALING__DEFAULT)

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
        if self.task_frame != other.task_frame:
            return False
        if self.selection_vector_x != other.selection_vector_x:
            return False
        if self.selection_vector_y != other.selection_vector_y:
            return False
        if self.selection_vector_z != other.selection_vector_z:
            return False
        if self.selection_vector_rx != other.selection_vector_rx:
            return False
        if self.selection_vector_ry != other.selection_vector_ry:
            return False
        if self.selection_vector_rz != other.selection_vector_rz:
            return False
        if self.wrench != other.wrench:
            return False
        if self.type != other.type:
            return False
        if self.speed_limits != other.speed_limits:
            return False
        if all(self.deviation_limits != other.deviation_limits):
            return False
        if self.damping_factor != other.damping_factor:
            return False
        if self.gain_scaling != other.gain_scaling:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def task_frame(self):
        """Message field 'task_frame'."""
        return self._task_frame

    @task_frame.setter
    def task_frame(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'task_frame' field must be a sub message of type 'PoseStamped'"
        self._task_frame = value

    @builtins.property
    def selection_vector_x(self):
        """Message field 'selection_vector_x'."""
        return self._selection_vector_x

    @selection_vector_x.setter
    def selection_vector_x(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'selection_vector_x' field must be of type 'bool'"
        self._selection_vector_x = value

    @builtins.property
    def selection_vector_y(self):
        """Message field 'selection_vector_y'."""
        return self._selection_vector_y

    @selection_vector_y.setter
    def selection_vector_y(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'selection_vector_y' field must be of type 'bool'"
        self._selection_vector_y = value

    @builtins.property
    def selection_vector_z(self):
        """Message field 'selection_vector_z'."""
        return self._selection_vector_z

    @selection_vector_z.setter
    def selection_vector_z(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'selection_vector_z' field must be of type 'bool'"
        self._selection_vector_z = value

    @builtins.property
    def selection_vector_rx(self):
        """Message field 'selection_vector_rx'."""
        return self._selection_vector_rx

    @selection_vector_rx.setter
    def selection_vector_rx(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'selection_vector_rx' field must be of type 'bool'"
        self._selection_vector_rx = value

    @builtins.property
    def selection_vector_ry(self):
        """Message field 'selection_vector_ry'."""
        return self._selection_vector_ry

    @selection_vector_ry.setter
    def selection_vector_ry(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'selection_vector_ry' field must be of type 'bool'"
        self._selection_vector_ry = value

    @builtins.property
    def selection_vector_rz(self):
        """Message field 'selection_vector_rz'."""
        return self._selection_vector_rz

    @selection_vector_rz.setter
    def selection_vector_rz(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'selection_vector_rz' field must be of type 'bool'"
        self._selection_vector_rz = value

    @builtins.property
    def wrench(self):
        """Message field 'wrench'."""
        return self._wrench

    @wrench.setter
    def wrench(self, value):
        if __debug__:
            from geometry_msgs.msg import Wrench
            assert \
                isinstance(value, Wrench), \
                "The 'wrench' field must be a sub message of type 'Wrench'"
        self._wrench = value

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type' field must be an unsigned integer in [0, 255]"
        self._type = value

    @builtins.property
    def speed_limits(self):
        """Message field 'speed_limits'."""
        return self._speed_limits

    @speed_limits.setter
    def speed_limits(self, value):
        if __debug__:
            from geometry_msgs.msg import Twist
            assert \
                isinstance(value, Twist), \
                "The 'speed_limits' field must be a sub message of type 'Twist'"
        self._speed_limits = value

    @builtins.property
    def deviation_limits(self):
        """Message field 'deviation_limits'."""
        return self._deviation_limits

    @deviation_limits.setter
    def deviation_limits(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'deviation_limits' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 6, \
                "The 'deviation_limits' numpy.ndarray() must have a size of 6"
            self._deviation_limits = value
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'deviation_limits' field must be a set or sequence with length 6 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._deviation_limits = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def damping_factor(self):
        """Message field 'damping_factor'."""
        return self._damping_factor

    @damping_factor.setter
    def damping_factor(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'damping_factor' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'damping_factor' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._damping_factor = value

    @builtins.property
    def gain_scaling(self):
        """Message field 'gain_scaling'."""
        return self._gain_scaling

    @gain_scaling.setter
    def gain_scaling(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gain_scaling' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'gain_scaling' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._gain_scaling = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetForceMode_Response(type):
    """Metaclass of message 'SetForceMode_Response'."""

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
            module = import_type_support('ur_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ur_msgs.srv.SetForceMode_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_force_mode__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_force_mode__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_force_mode__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_force_mode__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_force_mode__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetForceMode_Response(metaclass=Metaclass_SetForceMode_Response):
    """Message class 'SetForceMode_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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


class Metaclass_SetForceMode(type):
    """Metaclass of service 'SetForceMode'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ur_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ur_msgs.srv.SetForceMode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_force_mode

            from ur_msgs.srv import _set_force_mode
            if _set_force_mode.Metaclass_SetForceMode_Request._TYPE_SUPPORT is None:
                _set_force_mode.Metaclass_SetForceMode_Request.__import_type_support__()
            if _set_force_mode.Metaclass_SetForceMode_Response._TYPE_SUPPORT is None:
                _set_force_mode.Metaclass_SetForceMode_Response.__import_type_support__()


class SetForceMode(metaclass=Metaclass_SetForceMode):
    from ur_msgs.srv._set_force_mode import SetForceMode_Request as Request
    from ur_msgs.srv._set_force_mode import SetForceMode_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
