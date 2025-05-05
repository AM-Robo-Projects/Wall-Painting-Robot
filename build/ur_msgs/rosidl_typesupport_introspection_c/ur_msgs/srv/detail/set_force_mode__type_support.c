// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ur_msgs:srv/SetForceMode.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ur_msgs/srv/detail/set_force_mode__rosidl_typesupport_introspection_c.h"
#include "ur_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ur_msgs/srv/detail/set_force_mode__functions.h"
#include "ur_msgs/srv/detail/set_force_mode__struct.h"


// Include directives for member types
// Member `task_frame`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `task_frame`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `wrench`
#include "geometry_msgs/msg/wrench.h"
// Member `wrench`
#include "geometry_msgs/msg/detail/wrench__rosidl_typesupport_introspection_c.h"
// Member `speed_limits`
#include "geometry_msgs/msg/twist.h"
// Member `speed_limits`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ur_msgs__srv__SetForceMode_Request__init(message_memory);
}

void ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_fini_function(void * message_memory)
{
  ur_msgs__srv__SetForceMode_Request__fini(message_memory);
}

size_t ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__size_function__SetForceMode_Request__deviation_limits(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__get_const_function__SetForceMode_Request__deviation_limits(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__get_function__SetForceMode_Request__deviation_limits(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__fetch_function__SetForceMode_Request__deviation_limits(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__get_const_function__SetForceMode_Request__deviation_limits(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__assign_function__SetForceMode_Request__deviation_limits(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__get_function__SetForceMode_Request__deviation_limits(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_member_array[13] = {
  {
    "task_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, task_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "selection_vector_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, selection_vector_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "selection_vector_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, selection_vector_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "selection_vector_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, selection_vector_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "selection_vector_rx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, selection_vector_rx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "selection_vector_ry",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, selection_vector_ry),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "selection_vector_rz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, selection_vector_rz),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wrench",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, wrench),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed_limits",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, speed_limits),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "deviation_limits",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, deviation_limits),  // bytes offset in struct
    NULL,  // default value
    ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__size_function__SetForceMode_Request__deviation_limits,  // size() function pointer
    ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__get_const_function__SetForceMode_Request__deviation_limits,  // get_const(index) function pointer
    ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__get_function__SetForceMode_Request__deviation_limits,  // get(index) function pointer
    ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__fetch_function__SetForceMode_Request__deviation_limits,  // fetch(index, &value) function pointer
    ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__assign_function__SetForceMode_Request__deviation_limits,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "damping_factor",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, damping_factor),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gain_scaling",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Request, gain_scaling),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_members = {
  "ur_msgs__srv",  // message namespace
  "SetForceMode_Request",  // message name
  13,  // number of fields
  sizeof(ur_msgs__srv__SetForceMode_Request),
  ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_member_array,  // message members
  ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_type_support_handle = {
  0,
  &ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ur_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_msgs, srv, SetForceMode_Request)() {
  ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Wrench)();
  ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  if (!ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_type_support_handle.typesupport_identifier) {
    ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ur_msgs__srv__SetForceMode_Request__rosidl_typesupport_introspection_c__SetForceMode_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ur_msgs/srv/detail/set_force_mode__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ur_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ur_msgs/srv/detail/set_force_mode__functions.h"
// already included above
// #include "ur_msgs/srv/detail/set_force_mode__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ur_msgs__srv__SetForceMode_Response__init(message_memory);
}

void ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_fini_function(void * message_memory)
{
  ur_msgs__srv__SetForceMode_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ur_msgs__srv__SetForceMode_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_message_members = {
  "ur_msgs__srv",  // message namespace
  "SetForceMode_Response",  // message name
  1,  // number of fields
  sizeof(ur_msgs__srv__SetForceMode_Response),
  ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_message_member_array,  // message members
  ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_message_type_support_handle = {
  0,
  &ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ur_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_msgs, srv, SetForceMode_Response)() {
  if (!ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_message_type_support_handle.typesupport_identifier) {
    ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ur_msgs__srv__SetForceMode_Response__rosidl_typesupport_introspection_c__SetForceMode_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ur_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ur_msgs/srv/detail/set_force_mode__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_service_members = {
  "ur_msgs__srv",  // service namespace
  "SetForceMode",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_Request_message_type_support_handle,
  NULL  // response message
  // ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_Response_message_type_support_handle
};

static rosidl_service_type_support_t ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_service_type_support_handle = {
  0,
  &ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_msgs, srv, SetForceMode_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_msgs, srv, SetForceMode_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ur_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_msgs, srv, SetForceMode)() {
  if (!ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_service_type_support_handle.typesupport_identifier) {
    ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_msgs, srv, SetForceMode_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ur_msgs, srv, SetForceMode_Response)()->data;
  }

  return &ur_msgs__srv__detail__set_force_mode__rosidl_typesupport_introspection_c__SetForceMode_service_type_support_handle;
}
