// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ur_msgs:srv/SetAnalogOutput.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__TRAITS_HPP_
#define UR_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ur_msgs/srv/detail/set_analog_output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'data'
#include "ur_msgs/msg/detail/analog__traits.hpp"

namespace ur_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetAnalogOutput_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    to_flow_style_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetAnalogOutput_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data:\n";
    to_block_style_yaml(msg.data, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetAnalogOutput_Request & msg, bool use_flow_style = false)
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
  const ur_msgs::srv::SetAnalogOutput_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ur_msgs::srv::SetAnalogOutput_Request & msg)
{
  return ur_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ur_msgs::srv::SetAnalogOutput_Request>()
{
  return "ur_msgs::srv::SetAnalogOutput_Request";
}

template<>
inline const char * name<ur_msgs::srv::SetAnalogOutput_Request>()
{
  return "ur_msgs/srv/SetAnalogOutput_Request";
}

template<>
struct has_fixed_size<ur_msgs::srv::SetAnalogOutput_Request>
  : std::integral_constant<bool, has_fixed_size<ur_msgs::msg::Analog>::value> {};

template<>
struct has_bounded_size<ur_msgs::srv::SetAnalogOutput_Request>
  : std::integral_constant<bool, has_bounded_size<ur_msgs::msg::Analog>::value> {};

template<>
struct is_message<ur_msgs::srv::SetAnalogOutput_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ur_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetAnalogOutput_Response & msg,
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
  const SetAnalogOutput_Response & msg,
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

inline std::string to_yaml(const SetAnalogOutput_Response & msg, bool use_flow_style = false)
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
  const ur_msgs::srv::SetAnalogOutput_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ur_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ur_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ur_msgs::srv::SetAnalogOutput_Response & msg)
{
  return ur_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ur_msgs::srv::SetAnalogOutput_Response>()
{
  return "ur_msgs::srv::SetAnalogOutput_Response";
}

template<>
inline const char * name<ur_msgs::srv::SetAnalogOutput_Response>()
{
  return "ur_msgs/srv/SetAnalogOutput_Response";
}

template<>
struct has_fixed_size<ur_msgs::srv::SetAnalogOutput_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ur_msgs::srv::SetAnalogOutput_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ur_msgs::srv::SetAnalogOutput_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ur_msgs::srv::SetAnalogOutput>()
{
  return "ur_msgs::srv::SetAnalogOutput";
}

template<>
inline const char * name<ur_msgs::srv::SetAnalogOutput>()
{
  return "ur_msgs/srv/SetAnalogOutput";
}

template<>
struct has_fixed_size<ur_msgs::srv::SetAnalogOutput>
  : std::integral_constant<
    bool,
    has_fixed_size<ur_msgs::srv::SetAnalogOutput_Request>::value &&
    has_fixed_size<ur_msgs::srv::SetAnalogOutput_Response>::value
  >
{
};

template<>
struct has_bounded_size<ur_msgs::srv::SetAnalogOutput>
  : std::integral_constant<
    bool,
    has_bounded_size<ur_msgs::srv::SetAnalogOutput_Request>::value &&
    has_bounded_size<ur_msgs::srv::SetAnalogOutput_Response>::value
  >
{
};

template<>
struct is_service<ur_msgs::srv::SetAnalogOutput>
  : std::true_type
{
};

template<>
struct is_service_request<ur_msgs::srv::SetAnalogOutput_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ur_msgs::srv::SetAnalogOutput_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UR_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__TRAITS_HPP_
