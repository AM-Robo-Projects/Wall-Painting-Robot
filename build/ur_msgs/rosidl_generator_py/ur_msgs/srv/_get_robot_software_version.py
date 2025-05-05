# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ur_msgs:srv/GetRobotSoftwareVersion.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetRobotSoftwareVersion_Request(type):
    """Metaclass of message 'GetRobotSoftwareVersion_Request'."""

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
                'ur_msgs.srv.GetRobotSoftwareVersion_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_robot_software_version__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_robot_software_version__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_robot_software_version__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_robot_software_version__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_robot_software_version__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetRobotSoftwareVersion_Request(metaclass=Metaclass_GetRobotSoftwareVersion_Request):
    """Message class 'GetRobotSoftwareVersion_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetRobotSoftwareVersion_Response(type):
    """Metaclass of message 'GetRobotSoftwareVersion_Response'."""

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
                'ur_msgs.srv.GetRobotSoftwareVersion_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_robot_software_version__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_robot_software_version__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_robot_software_version__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_robot_software_version__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_robot_software_version__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetRobotSoftwareVersion_Response(metaclass=Metaclass_GetRobotSoftwareVersion_Response):
    """Message class 'GetRobotSoftwareVersion_Response'."""

    __slots__ = [
        '_major',
        '_minor',
        '_bugfix',
        '_build',
    ]

    _fields_and_field_types = {
        'major': 'uint32',
        'minor': 'uint32',
        'bugfix': 'uint32',
        'build': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.major = kwargs.get('major', int())
        self.minor = kwargs.get('minor', int())
        self.bugfix = kwargs.get('bugfix', int())
        self.build = kwargs.get('build', int())

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
        if self.major != other.major:
            return False
        if self.minor != other.minor:
            return False
        if self.bugfix != other.bugfix:
            return False
        if self.build != other.build:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def major(self):
        """Message field 'major'."""
        return self._major

    @major.setter
    def major(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'major' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'major' field must be an unsigned integer in [0, 4294967295]"
        self._major = value

    @builtins.property
    def minor(self):
        """Message field 'minor'."""
        return self._minor

    @minor.setter
    def minor(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'minor' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'minor' field must be an unsigned integer in [0, 4294967295]"
        self._minor = value

    @builtins.property
    def bugfix(self):
        """Message field 'bugfix'."""
        return self._bugfix

    @bugfix.setter
    def bugfix(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bugfix' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'bugfix' field must be an unsigned integer in [0, 4294967295]"
        self._bugfix = value

    @builtins.property
    def build(self):
        """Message field 'build'."""
        return self._build

    @build.setter
    def build(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'build' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'build' field must be an unsigned integer in [0, 4294967295]"
        self._build = value


class Metaclass_GetRobotSoftwareVersion(type):
    """Metaclass of service 'GetRobotSoftwareVersion'."""

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
                'ur_msgs.srv.GetRobotSoftwareVersion')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_robot_software_version

            from ur_msgs.srv import _get_robot_software_version
            if _get_robot_software_version.Metaclass_GetRobotSoftwareVersion_Request._TYPE_SUPPORT is None:
                _get_robot_software_version.Metaclass_GetRobotSoftwareVersion_Request.__import_type_support__()
            if _get_robot_software_version.Metaclass_GetRobotSoftwareVersion_Response._TYPE_SUPPORT is None:
                _get_robot_software_version.Metaclass_GetRobotSoftwareVersion_Response.__import_type_support__()


class GetRobotSoftwareVersion(metaclass=Metaclass_GetRobotSoftwareVersion):
    from ur_msgs.srv._get_robot_software_version import GetRobotSoftwareVersion_Request as Request
    from ur_msgs.srv._get_robot_software_version import GetRobotSoftwareVersion_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
