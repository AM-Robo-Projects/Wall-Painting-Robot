// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ur_msgs:srv/SetForceMode.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__TRAITS_HPP_
#define UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ur_msgs/srv/detail/set_force_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'task_frame'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"
// Member 'speed_limits'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace ur_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetForceMode_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_frame
  {
    out << "task_frame: ";
    to_flow_style_yaml(msg.task_frame, out);
    out << ", ";
  }

  // member: selection_vector_x
  {
    out << "selection_vector_x: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_x, out);
    out << ", ";
  }

  // member: selection_vector_y
  {
    out << "selection_vector_y: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_y, out);
    out << ", ";
  }

  // member: selection_vector_z
  {
    out << "selection_vector_z: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_z, out);
    out << ", ";
  }

  // member: selection_vector_rx
  {
    out << "selection_vector_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_rx, out);
    out << ", ";
  }

  // member: selection_vector_ry
  {
    out << "selection_vector_ry: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_ry, out);
    out << ", ";
  }

  // member: selection_vector_rz
  {
    out << "selection_vector_rz: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_rz, out);
    out << ", ";
  }

  // member: wrench
  {
    out << "wrench: ";
    to_flow_style_yaml(msg.wrench, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: speed_limits
  {
    out << "speed_limits: ";
    to_flow_style_yaml(msg.speed_limits, out);
    out << ", ";
  }

  // member: deviation_limits
  {
    if (msg.deviation_limits.size() == 0) {
      out << "deviation_limits: []";
    } else {
      out << "deviation_limits: [";
      size_t pending_items = msg.deviation_limits.size();
      for (auto item : msg.deviation_limits) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: damping_factor
  {
    out << "damping_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_factor, out);
    out << ", ";
  }

  // member: gain_scaling
  {
    out << "gain_scaling: ";
    rosidl_generator_traits::value_to_yaml(msg.gain_scaling, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetForceMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_frame:\n";
    to_block_style_yaml(msg.task_frame, out, indentation + 2);
  }

  // member: selection_vector_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selection_vector_x: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_x, out);
    out << "\n";
  }

  // member: selection_vector_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selection_vector_y: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_y, out);
    out << "\n";
  }

  // member: selection_vector_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selection_vector_z: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_z, out);
    out << "\n";
  }

  // member: selection_vector_rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selection_vector_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_rx, out);
    out << "\n";
  }

  // member: selection_vector_ry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selection_vector_ry: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_ry, out);
    out << "\n";
  }

  // member: selection_vector_rz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selection_vector_rz: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_vector_rz, out);
    out << "\n";
  }

  // member: wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrench:\n";
    to_block_style_yaml(msg.wrench, out, indentation + 2);
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: speed_limits
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_limits:\n";
    to_block_style_yaml(msg.speed_limits, out, indentation + 2);
  }

  // member: deviation_limits
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.deviation_limits.size() == 0) {
      out << "deviation_limits: []\n";
    } else {
      out << "deviation_limits:\n";
      for (auto item : msg.deviation_limits) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: damping_factor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "damping_factor: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_factor, out);
    out << "\n";
  }

  // member: gain_scaling
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gain_scaling: ";
    rosidl_generator_traits::value_to_yaml(msg.gain_scaling, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetForceMode_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ur_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ur_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ur_msgs::srv::SetForceMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ur_msgs::srv::SetForceMode_Request & msg)
{
  return ur_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ur_msgs::srv::SetForceMode_Request>()
{
  return "ur_msgs::srv::SetForceMode_Request";
}

template<>
inline const char * name<ur_msgs::srv::SetForceMode_Request>()
{
  return "ur_msgs/srv/SetForceMode_Request";
}

template<>
struct has_fixed_size<ur_msgs::srv::SetForceMode_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<geometry_msgs::msg::Wrench>::value> {};

template<>
struct has_bounded_size<ur_msgs::srv::SetForceMode_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<geometry_msgs::msg::Wrench>::value> {};

template<>
struct is_message<ur_msgs::srv::SetForceMode_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ur_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetForceMode_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetForceMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetForceMode_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ur_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ur_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ur_msgs::srv::SetForceMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ur_msgs::srv::SetForceMode_Response & msg)
{
  return ur_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ur_msgs::srv::SetForceMode_Response>()
{
  return "ur_msgs::srv::SetForceMode_Response";
}

template<>
inline const char * name<ur_msgs::srv::SetForceMode_Response>()
{
  return "ur_msgs/srv/SetForceMode_Response";
}

template<>
struct has_fixed_size<ur_msgs::srv::SetForceMode_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ur_msgs::srv::SetForceMode_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ur_msgs::srv::SetForceMode_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ur_msgs::srv::SetForceMode>()
{
  return "ur_msgs::srv::SetForceMode";
}

template<>
inline const char * name<ur_msgs::srv::SetForceMode>()
{
  return "ur_msgs/srv/SetForceMode";
}

template<>
struct has_fixed_size<ur_msgs::srv::SetForceMode>
  : std::integral_constant<
    bool,
    has_fixed_size<ur_msgs::srv::SetForceMode_Request>::value &&
    has_fixed_size<ur_msgs::srv::SetForceMode_Response>::value
  >
{
};

template<>
struct has_bounded_size<ur_msgs::srv::SetForceMode>
  : std::integral_constant<
    bool,
    has_bounded_size<ur_msgs::srv::SetForceMode_Request>::value &&
    has_bounded_size<ur_msgs::srv::SetForceMode_Response>::value
  >
{
};

template<>
struct is_service<ur_msgs::srv::SetForceMode>
  : std::true_type
{
};

template<>
struct is_service_request<ur_msgs::srv::SetForceMode_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ur_msgs::srv::SetForceMode_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__TRAITS_HPP_
