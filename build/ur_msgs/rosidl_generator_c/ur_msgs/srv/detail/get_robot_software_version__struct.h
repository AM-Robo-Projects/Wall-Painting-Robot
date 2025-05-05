// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_msgs:srv/GetRobotSoftwareVersion.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__STRUCT_H_
#define UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetRobotSoftwareVersion in the package ur_msgs.
typedef struct ur_msgs__srv__GetRobotSoftwareVersion_Request
{
  uint8_t structure_needs_at_least_one_member;
} ur_msgs__srv__GetRobotSoftwareVersion_Request;

// Struct for a sequence of ur_msgs__srv__GetRobotSoftwareVersion_Request.
typedef struct ur_msgs__srv__GetRobotSoftwareVersion_Request__Sequence
{
  ur_msgs__srv__GetRobotSoftwareVersion_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__srv__GetRobotSoftwareVersion_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetRobotSoftwareVersion in the package ur_msgs.
typedef struct ur_msgs__srv__GetRobotSoftwareVersion_Response
{
  /// Major version number
  uint32_t major;
  /// Minor version number
  uint32_t minor;
  /// Bugfix version number
  uint32_t bugfix;
  /// Build number
  uint32_t build;
} ur_msgs__srv__GetRobotSoftwareVersion_Response;

// Struct for a sequence of ur_msgs__srv__GetRobotSoftwareVersion_Response.
typedef struct ur_msgs__srv__GetRobotSoftwareVersion_Response__Sequence
{
  ur_msgs__srv__GetRobotSoftwareVersion_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__srv__GetRobotSoftwareVersion_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__STRUCT_H_
