// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur_msgs:srv/SetForceMode.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__BUILDER_HPP_
#define UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur_msgs/srv/detail/set_force_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur_msgs
{

namespace srv
{

namespace builder
{

class Init_SetForceMode_Request_gain_scaling
{
public:
  explicit Init_SetForceMode_Request_gain_scaling(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  ::ur_msgs::srv::SetForceMode_Request gain_scaling(::ur_msgs::srv::SetForceMode_Request::_gain_scaling_type arg)
  {
    msg_.gain_scaling = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_damping_factor
{
public:
  explicit Init_SetForceMode_Request_damping_factor(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_gain_scaling damping_factor(::ur_msgs::srv::SetForceMode_Request::_damping_factor_type arg)
  {
    msg_.damping_factor = std::move(arg);
    return Init_SetForceMode_Request_gain_scaling(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_deviation_limits
{
public:
  explicit Init_SetForceMode_Request_deviation_limits(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_damping_factor deviation_limits(::ur_msgs::srv::SetForceMode_Request::_deviation_limits_type arg)
  {
    msg_.deviation_limits = std::move(arg);
    return Init_SetForceMode_Request_damping_factor(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_speed_limits
{
public:
  explicit Init_SetForceMode_Request_speed_limits(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_deviation_limits speed_limits(::ur_msgs::srv::SetForceMode_Request::_speed_limits_type arg)
  {
    msg_.speed_limits = std::move(arg);
    return Init_SetForceMode_Request_deviation_limits(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_type
{
public:
  explicit Init_SetForceMode_Request_type(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_speed_limits type(::ur_msgs::srv::SetForceMode_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_SetForceMode_Request_speed_limits(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_wrench
{
public:
  explicit Init_SetForceMode_Request_wrench(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_type wrench(::ur_msgs::srv::SetForceMode_Request::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return Init_SetForceMode_Request_type(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_selection_vector_rz
{
public:
  explicit Init_SetForceMode_Request_selection_vector_rz(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_wrench selection_vector_rz(::ur_msgs::srv::SetForceMode_Request::_selection_vector_rz_type arg)
  {
    msg_.selection_vector_rz = std::move(arg);
    return Init_SetForceMode_Request_wrench(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_selection_vector_ry
{
public:
  explicit Init_SetForceMode_Request_selection_vector_ry(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_selection_vector_rz selection_vector_ry(::ur_msgs::srv::SetForceMode_Request::_selection_vector_ry_type arg)
  {
    msg_.selection_vector_ry = std::move(arg);
    return Init_SetForceMode_Request_selection_vector_rz(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_selection_vector_rx
{
public:
  explicit Init_SetForceMode_Request_selection_vector_rx(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_selection_vector_ry selection_vector_rx(::ur_msgs::srv::SetForceMode_Request::_selection_vector_rx_type arg)
  {
    msg_.selection_vector_rx = std::move(arg);
    return Init_SetForceMode_Request_selection_vector_ry(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_selection_vector_z
{
public:
  explicit Init_SetForceMode_Request_selection_vector_z(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_selection_vector_rx selection_vector_z(::ur_msgs::srv::SetForceMode_Request::_selection_vector_z_type arg)
  {
    msg_.selection_vector_z = std::move(arg);
    return Init_SetForceMode_Request_selection_vector_rx(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_selection_vector_y
{
public:
  explicit Init_SetForceMode_Request_selection_vector_y(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_selection_vector_z selection_vector_y(::ur_msgs::srv::SetForceMode_Request::_selection_vector_y_type arg)
  {
    msg_.selection_vector_y = std::move(arg);
    return Init_SetForceMode_Request_selection_vector_z(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_selection_vector_x
{
public:
  explicit Init_SetForceMode_Request_selection_vector_x(::ur_msgs::srv::SetForceMode_Request & msg)
  : msg_(msg)
  {}
  Init_SetForceMode_Request_selection_vector_y selection_vector_x(::ur_msgs::srv::SetForceMode_Request::_selection_vector_x_type arg)
  {
    msg_.selection_vector_x = std::move(arg);
    return Init_SetForceMode_Request_selection_vector_y(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

class Init_SetForceMode_Request_task_frame
{
public:
  Init_SetForceMode_Request_task_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetForceMode_Request_selection_vector_x task_frame(::ur_msgs::srv::SetForceMode_Request::_task_frame_type arg)
  {
    msg_.task_frame = std::move(arg);
    return Init_SetForceMode_Request_selection_vector_x(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_msgs::srv::SetForceMode_Request>()
{
  return ur_msgs::srv::builder::Init_SetForceMode_Request_task_frame();
}

}  // namespace ur_msgs


namespace ur_msgs
{

namespace srv
{

namespace builder
{

class Init_SetForceMode_Response_success
{
public:
  Init_SetForceMode_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ur_msgs::srv::SetForceMode_Response success(::ur_msgs::srv::SetForceMode_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_msgs::srv::SetForceMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_msgs::srv::SetForceMode_Response>()
{
  return ur_msgs::srv::builder::Init_SetForceMode_Response_success();
}

}  // namespace ur_msgs

#endif  // UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__BUILDER_HPP_
