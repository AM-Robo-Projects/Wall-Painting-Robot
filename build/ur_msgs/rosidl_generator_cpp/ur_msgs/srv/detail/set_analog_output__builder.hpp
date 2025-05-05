// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur_msgs:srv/SetAnalogOutput.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__BUILDER_HPP_
#define UR_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur_msgs/srv/detail/set_analog_output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur_msgs
{

namespace srv
{

namespace builder
{

class Init_SetAnalogOutput_Request_data
{
public:
  Init_SetAnalogOutput_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ur_msgs::srv::SetAnalogOutput_Request data(::ur_msgs::srv::SetAnalogOutput_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_msgs::srv::SetAnalogOutput_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_msgs::srv::SetAnalogOutput_Request>()
{
  return ur_msgs::srv::builder::Init_SetAnalogOutput_Request_data();
}

}  // namespace ur_msgs


namespace ur_msgs
{

namespace srv
{

namespace builder
{

class Init_SetAnalogOutput_Response_success
{
public:
  Init_SetAnalogOutput_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ur_msgs::srv::SetAnalogOutput_Response success(::ur_msgs::srv::SetAnalogOutput_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_msgs::srv::SetAnalogOutput_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_msgs::srv::SetAnalogOutput_Response>()
{
  return ur_msgs::srv::builder::Init_SetAnalogOutput_Response_success();
}

}  // namespace ur_msgs

#endif  // UR_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__BUILDER_HPP_
