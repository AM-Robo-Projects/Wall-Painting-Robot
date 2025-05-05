// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ur_msgs:action/ToolContact.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__ACTION__DETAIL__TOOL_CONTACT__STRUCT_H_
#define UR_MSGS__ACTION__DETAIL__TOOL_CONTACT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/ToolContact in the package ur_msgs.
typedef struct ur_msgs__action__ToolContact_Goal
{
  uint8_t structure_needs_at_least_one_member;
} ur_msgs__action__ToolContact_Goal;

// Struct for a sequence of ur_msgs__action__ToolContact_Goal.
typedef struct ur_msgs__action__ToolContact_Goal__Sequence
{
  ur_msgs__action__ToolContact_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__action__ToolContact_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/ToolContact in the package ur_msgs.
typedef struct ur_msgs__action__ToolContact_Result
{
  uint8_t structure_needs_at_least_one_member;
} ur_msgs__action__ToolContact_Result;

// Struct for a sequence of ur_msgs__action__ToolContact_Result.
typedef struct ur_msgs__action__ToolContact_Result__Sequence
{
  ur_msgs__action__ToolContact_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__action__ToolContact_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/ToolContact in the package ur_msgs.
typedef struct ur_msgs__action__ToolContact_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} ur_msgs__action__ToolContact_Feedback;

// Struct for a sequence of ur_msgs__action__ToolContact_Feedback.
typedef struct ur_msgs__action__ToolContact_Feedback__Sequence
{
  ur_msgs__action__ToolContact_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__action__ToolContact_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "ur_msgs/action/detail/tool_contact__struct.h"

/// Struct defined in action/ToolContact in the package ur_msgs.
typedef struct ur_msgs__action__ToolContact_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  ur_msgs__action__ToolContact_Goal goal;
} ur_msgs__action__ToolContact_SendGoal_Request;

// Struct for a sequence of ur_msgs__action__ToolContact_SendGoal_Request.
typedef struct ur_msgs__action__ToolContact_SendGoal_Request__Sequence
{
  ur_msgs__action__ToolContact_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__action__ToolContact_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ToolContact in the package ur_msgs.
typedef struct ur_msgs__action__ToolContact_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} ur_msgs__action__ToolContact_SendGoal_Response;

// Struct for a sequence of ur_msgs__action__ToolContact_SendGoal_Response.
typedef struct ur_msgs__action__ToolContact_SendGoal_Response__Sequence
{
  ur_msgs__action__ToolContact_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__action__ToolContact_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ToolContact in the package ur_msgs.
typedef struct ur_msgs__action__ToolContact_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} ur_msgs__action__ToolContact_GetResult_Request;

// Struct for a sequence of ur_msgs__action__ToolContact_GetResult_Request.
typedef struct ur_msgs__action__ToolContact_GetResult_Request__Sequence
{
  ur_msgs__action__ToolContact_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__action__ToolContact_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "ur_msgs/action/detail/tool_contact__struct.h"

/// Struct defined in action/ToolContact in the package ur_msgs.
typedef struct ur_msgs__action__ToolContact_GetResult_Response
{
  int8_t status;
  ur_msgs__action__ToolContact_Result result;
} ur_msgs__action__ToolContact_GetResult_Response;

// Struct for a sequence of ur_msgs__action__ToolContact_GetResult_Response.
typedef struct ur_msgs__action__ToolContact_GetResult_Response__Sequence
{
  ur_msgs__action__ToolContact_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__action__ToolContact_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "ur_msgs/action/detail/tool_contact__struct.h"

/// Struct defined in action/ToolContact in the package ur_msgs.
typedef struct ur_msgs__action__ToolContact_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  ur_msgs__action__ToolContact_Feedback feedback;
} ur_msgs__action__ToolContact_FeedbackMessage;

// Struct for a sequence of ur_msgs__action__ToolContact_FeedbackMessage.
typedef struct ur_msgs__action__ToolContact_FeedbackMessage__Sequence
{
  ur_msgs__action__ToolContact_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ur_msgs__action__ToolContact_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UR_MSGS__ACTION__DETAIL__TOOL_CONTACT__STRUCT_H_
