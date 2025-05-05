// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur_msgs:srv/GetRobotSoftwareVersion.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__BUILDER_HPP_
#define UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur_msgs/srv/detail/get_robot_software_version__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_msgs::srv::GetRobotSoftwareVersion_Request>()
{
  return ::ur_msgs::srv::GetRobotSoftwareVersion_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ur_msgs


namespace ur_msgs
{

namespace srv
{

namespace builder
{

class Init_GetRobotSoftwareVersion_Response_build
{
public:
  explicit Init_GetRobotSoftwareVersion_Response_build(::ur_msgs::srv::GetRobotSoftwareVersion_Response & msg)
  : msg_(msg)
  {}
  ::ur_msgs::srv::GetRobotSoftwareVersion_Response build(::ur_msgs::srv::GetRobotSoftwareVersion_Response::_build_type arg)
  {
    msg_.build = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_msgs::srv::GetRobotSoftwareVersion_Response msg_;
};

class Init_GetRobotSoftwareVersion_Response_bugfix
{
public:
  explicit Init_GetRobotSoftwareVersion_Response_bugfix(::ur_msgs::srv::GetRobotSoftwareVersion_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotSoftwareVersion_Response_build bugfix(::ur_msgs::srv::GetRobotSoftwareVersion_Response::_bugfix_type arg)
  {
    msg_.bugfix = std::move(arg);
    return Init_GetRobotSoftwareVersion_Response_build(msg_);
  }

private:
  ::ur_msgs::srv::GetRobotSoftwareVersion_Response msg_;
};

class Init_GetRobotSoftwareVersion_Response_minor
{
public:
  explicit Init_GetRobotSoftwareVersion_Response_minor(::ur_msgs::srv::GetRobotSoftwareVersion_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobotSoftwareVersion_Response_bugfix minor(::ur_msgs::srv::GetRobotSoftwareVersion_Response::_minor_type arg)
  {
    msg_.minor = std::move(arg);
    return Init_GetRobotSoftwareVersion_Response_bugfix(msg_);
  }

private:
  ::ur_msgs::srv::GetRobotSoftwareVersion_Response msg_;
};

class Init_GetRobotSoftwareVersion_Response_major
{
public:
  Init_GetRobotSoftwareVersion_Response_major()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetRobotSoftwareVersion_Response_minor major(::ur_msgs::srv::GetRobotSoftwareVersion_Response::_major_type arg)
  {
    msg_.major = std::move(arg);
    return Init_GetRobotSoftwareVersion_Response_minor(msg_);
  }

private:
  ::ur_msgs::srv::GetRobotSoftwareVersion_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_msgs::srv::GetRobotSoftwareVersion_Response>()
{
  return ur_msgs::srv::builder::Init_GetRobotSoftwareVersion_Response_major();
}

}  // namespace ur_msgs

#endif  // UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__BUILDER_HPP_
