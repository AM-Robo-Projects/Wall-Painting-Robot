// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ur_msgs:srv/GetRobotSoftwareVersion.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__STRUCT_HPP_
#define UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ur_msgs__srv__GetRobotSoftwareVersion_Request __attribute__((deprecated))
#else
# define DEPRECATED__ur_msgs__srv__GetRobotSoftwareVersion_Request __declspec(deprecated)
#endif

namespace ur_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetRobotSoftwareVersion_Request_
{
  using Type = GetRobotSoftwareVersion_Request_<ContainerAllocator>;

  explicit GetRobotSoftwareVersion_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetRobotSoftwareVersion_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_msgs__srv__GetRobotSoftwareVersion_Request
    std::shared_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_msgs__srv__GetRobotSoftwareVersion_Request
    std::shared_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetRobotSoftwareVersion_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetRobotSoftwareVersion_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetRobotSoftwareVersion_Request_

// alias to use template instance with default allocator
using GetRobotSoftwareVersion_Request =
  ur_msgs::srv::GetRobotSoftwareVersion_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ur_msgs


#ifndef _WIN32
# define DEPRECATED__ur_msgs__srv__GetRobotSoftwareVersion_Response __attribute__((deprecated))
#else
# define DEPRECATED__ur_msgs__srv__GetRobotSoftwareVersion_Response __declspec(deprecated)
#endif

namespace ur_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetRobotSoftwareVersion_Response_
{
  using Type = GetRobotSoftwareVersion_Response_<ContainerAllocator>;

  explicit GetRobotSoftwareVersion_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->major = 0ul;
      this->minor = 0ul;
      this->bugfix = 0ul;
      this->build = 0ul;
    }
  }

  explicit GetRobotSoftwareVersion_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->major = 0ul;
      this->minor = 0ul;
      this->bugfix = 0ul;
      this->build = 0ul;
    }
  }

  // field types and members
  using _major_type =
    uint32_t;
  _major_type major;
  using _minor_type =
    uint32_t;
  _minor_type minor;
  using _bugfix_type =
    uint32_t;
  _bugfix_type bugfix;
  using _build_type =
    uint32_t;
  _build_type build;

  // setters for named parameter idiom
  Type & set__major(
    const uint32_t & _arg)
  {
    this->major = _arg;
    return *this;
  }
  Type & set__minor(
    const uint32_t & _arg)
  {
    this->minor = _arg;
    return *this;
  }
  Type & set__bugfix(
    const uint32_t & _arg)
  {
    this->bugfix = _arg;
    return *this;
  }
  Type & set__build(
    const uint32_t & _arg)
  {
    this->build = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_msgs__srv__GetRobotSoftwareVersion_Response
    std::shared_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_msgs__srv__GetRobotSoftwareVersion_Response
    std::shared_ptr<ur_msgs::srv::GetRobotSoftwareVersion_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetRobotSoftwareVersion_Response_ & other) const
  {
    if (this->major != other.major) {
      return false;
    }
    if (this->minor != other.minor) {
      return false;
    }
    if (this->bugfix != other.bugfix) {
      return false;
    }
    if (this->build != other.build) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetRobotSoftwareVersion_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetRobotSoftwareVersion_Response_

// alias to use template instance with default allocator
using GetRobotSoftwareVersion_Response =
  ur_msgs::srv::GetRobotSoftwareVersion_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ur_msgs

namespace ur_msgs
{

namespace srv
{

struct GetRobotSoftwareVersion
{
  using Request = ur_msgs::srv::GetRobotSoftwareVersion_Request;
  using Response = ur_msgs::srv::GetRobotSoftwareVersion_Response;
};

}  // namespace srv

}  // namespace ur_msgs

#endif  // UR_MSGS__SRV__DETAIL__GET_ROBOT_SOFTWARE_VERSION__STRUCT_HPP_
