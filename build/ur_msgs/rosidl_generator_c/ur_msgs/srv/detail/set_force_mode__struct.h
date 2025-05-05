// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_msgs:srv/SetForceMode.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__STRUCT_H_
#define UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TCP_TO_ORIGIN'.
/**
  * Type constants:
 */
enum
{
  ur_msgs__srv__SetForceMode_Request__TCP_TO_ORIGIN = 1
};

/// Constant 'NO_TRANSFORM'.
enum
{
  ur_msgs__srv__SetForceMode_Request__NO_TRANSFORM = 2
};

/// Constant 'TCP_VELOCITY_TO_X_Y'.
enum
{
  ur_msgs__srv__SetForceMode_Request__TCP_VELOCITY_TO_X_Y = 3
};

// Include directives for member types
// Member 'task_frame'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__struct.h"
// Member 'speed_limits'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in srv/SetForceMode in the package ur_msgs.
typedef struct ur_msgs__srv__SetForceMode_Request
{
  geometry_msgs__msg__PoseStamped task_frame;
  /// 1 means that the robot will be compliant in the corresponding axis of the task frame
  bool selection_vector_x;
  bool selection_vector_y;
  bool selection_vector_z;
  bool selection_vector_rx;
  bool selection_vector_ry;
  bool selection_vector_rz;
  /// The forces/torques the robot will apply to its environment. For geometric interpretation, please
  /// see parameter `type`
  geometry_msgs__msg__Wrench wrench;
  /// An integer specifying how the robot interprets the force frame
  /// 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing
  ///    from the robot tcp towards the origin of the force frame.
  /// 2: The force frame is not transformed.
  /// 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp
  ///     velocity vector onto the x-y plane of the force frame.
  uint8_t type;
  /// Maximum allowed tcp speed (relative to the task frame).
  /// PLEASE NOTE: This is only relevant for axes marked as compliant in the selection_vector
  geometry_msgs__msg__Twist speed_limits;
  /// For non-compliant axes, these values are the maximum allowed deviation along/about an axis
  /// between the actual tcp position and the one set by the program.
  float deviation_limits[6];
  /// Force mode damping factor. Sets the damping parameter in force mode. In range, default value is 0.025
  /// A value of 1 is full damping, so the robot will decelerate quickly if no force is present. A value of 0
  /// is no damping, here the robot will maintain the speed.
  float damping_factor;
  /// Force mode gain scaling factor. Scales the gain in force mode. scaling parameter is in range, default is 0.5.
  /// A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard surfaces.
  float gain_scaling;
} ur_msgs__srv__SetForceMode_Request;

// Struct for a sequence of ur_msgs__srv__SetForceMode_Request.
typedef struct ur_msgs__srv__SetForceMode_Request__Sequence
{
  ur_msgs__srv__SetForceMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__srv__SetForceMode_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetForceMode in the package ur_msgs.
typedef struct ur_msgs__srv__SetForceMode_Response
{
  bool success;
} ur_msgs__srv__SetForceMode_Response;

// Struct for a sequence of ur_msgs__srv__SetForceMode_Response.
typedef struct ur_msgs__srv__SetForceMode_Response__Sequence
{
  ur_msgs__srv__SetForceMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__srv__SetForceMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__STRUCT_H_
