// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ur_msgs:srv/SetForceMode.idl
// generated code does not contain a copyright notice

#ifndef UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__STRUCT_HPP_
#define UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'task_frame'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__struct.hpp"
// Member 'speed_limits'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ur_msgs__srv__SetForceMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__ur_msgs__srv__SetForceMode_Request __declspec(deprecated)
#endif

namespace ur_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetForceMode_Request_
{
  using Type = SetForceMode_Request_<ContainerAllocator>;

  explicit SetForceMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_frame(_init),
    wrench(_init),
    speed_limits(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->selection_vector_x = false;
      this->selection_vector_y = false;
      this->selection_vector_z = false;
      this->selection_vector_rx = false;
      this->selection_vector_ry = false;
      this->selection_vector_rz = false;
      this->type = 2;
      std::fill<typename std::array<float, 6>::iterator, float>(this->deviation_limits.begin(), this->deviation_limits.end(), 0.01f);
      this->damping_factor = 0.025f;
      this->gain_scaling = 0.5f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->selection_vector_x = false;
      this->selection_vector_y = false;
      this->selection_vector_z = false;
      this->selection_vector_rx = false;
      this->selection_vector_ry = false;
      this->selection_vector_rz = false;
      this->type = 0;
      std::fill<typename std::array<float, 6>::iterator, float>(this->deviation_limits.begin(), this->deviation_limits.end(), 0.0f);
      this->damping_factor = 0.0f;
      this->gain_scaling = 0.0f;
    }
  }

  explicit SetForceMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_frame(_alloc, _init),
    wrench(_alloc, _init),
    speed_limits(_alloc, _init),
    deviation_limits(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->selection_vector_x = false;
      this->selection_vector_y = false;
      this->selection_vector_z = false;
      this->selection_vector_rx = false;
      this->selection_vector_ry = false;
      this->selection_vector_rz = false;
      this->type = 2;
      std::fill<typename std::array<float, 6>::iterator, float>(this->deviation_limits.begin(), this->deviation_limits.end(), 0.01f);
      this->damping_factor = 0.025f;
      this->gain_scaling = 0.5f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->selection_vector_x = false;
      this->selection_vector_y = false;
      this->selection_vector_z = false;
      this->selection_vector_rx = false;
      this->selection_vector_ry = false;
      this->selection_vector_rz = false;
      this->type = 0;
      std::fill<typename std::array<float, 6>::iterator, float>(this->deviation_limits.begin(), this->deviation_limits.end(), 0.0f);
      this->damping_factor = 0.0f;
      this->gain_scaling = 0.0f;
    }
  }

  // field types and members
  using _task_frame_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _task_frame_type task_frame;
  using _selection_vector_x_type =
    bool;
  _selection_vector_x_type selection_vector_x;
  using _selection_vector_y_type =
    bool;
  _selection_vector_y_type selection_vector_y;
  using _selection_vector_z_type =
    bool;
  _selection_vector_z_type selection_vector_z;
  using _selection_vector_rx_type =
    bool;
  _selection_vector_rx_type selection_vector_rx;
  using _selection_vector_ry_type =
    bool;
  _selection_vector_ry_type selection_vector_ry;
  using _selection_vector_rz_type =
    bool;
  _selection_vector_rz_type selection_vector_rz;
  using _wrench_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _wrench_type wrench;
  using _type_type =
    uint8_t;
  _type_type type;
  using _speed_limits_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _speed_limits_type speed_limits;
  using _deviation_limits_type =
    std::array<float, 6>;
  _deviation_limits_type deviation_limits;
  using _damping_factor_type =
    float;
  _damping_factor_type damping_factor;
  using _gain_scaling_type =
    float;
  _gain_scaling_type gain_scaling;

  // setters for named parameter idiom
  Type & set__task_frame(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->task_frame = _arg;
    return *this;
  }
  Type & set__selection_vector_x(
    const bool & _arg)
  {
    this->selection_vector_x = _arg;
    return *this;
  }
  Type & set__selection_vector_y(
    const bool & _arg)
  {
    this->selection_vector_y = _arg;
    return *this;
  }
  Type & set__selection_vector_z(
    const bool & _arg)
  {
    this->selection_vector_z = _arg;
    return *this;
  }
  Type & set__selection_vector_rx(
    const bool & _arg)
  {
    this->selection_vector_rx = _arg;
    return *this;
  }
  Type & set__selection_vector_ry(
    const bool & _arg)
  {
    this->selection_vector_ry = _arg;
    return *this;
  }
  Type & set__selection_vector_rz(
    const bool & _arg)
  {
    this->selection_vector_rz = _arg;
    return *this;
  }
  Type & set__wrench(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->wrench = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__speed_limits(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->speed_limits = _arg;
    return *this;
  }
  Type & set__deviation_limits(
    const std::array<float, 6> & _arg)
  {
    this->deviation_limits = _arg;
    return *this;
  }
  Type & set__damping_factor(
    const float & _arg)
  {
    this->damping_factor = _arg;
    return *this;
  }
  Type & set__gain_scaling(
    const float & _arg)
  {
    this->gain_scaling = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t TCP_TO_ORIGIN =
    1u;
  static constexpr uint8_t NO_TRANSFORM =
    2u;
  static constexpr uint8_t TCP_VELOCITY_TO_X_Y =
    3u;

  // pointer types
  using RawPtr =
    ur_msgs::srv::SetForceMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_msgs::srv::SetForceMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_msgs::srv::SetForceMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_msgs::srv::SetForceMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_msgs::srv::SetForceMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_msgs::srv::SetForceMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_msgs::srv::SetForceMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_msgs::srv::SetForceMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_msgs::srv::SetForceMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_msgs::srv::SetForceMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_msgs__srv__SetForceMode_Request
    std::shared_ptr<ur_msgs::srv::SetForceMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_msgs__srv__SetForceMode_Request
    std::shared_ptr<ur_msgs::srv::SetForceMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetForceMode_Request_ & other) const
  {
    if (this->task_frame != other.task_frame) {
      return false;
    }
    if (this->selection_vector_x != other.selection_vector_x) {
      return false;
    }
    if (this->selection_vector_y != other.selection_vector_y) {
      return false;
    }
    if (this->selection_vector_z != other.selection_vector_z) {
      return false;
    }
    if (this->selection_vector_rx != other.selection_vector_rx) {
      return false;
    }
    if (this->selection_vector_ry != other.selection_vector_ry) {
      return false;
    }
    if (this->selection_vector_rz != other.selection_vector_rz) {
      return false;
    }
    if (this->wrench != other.wrench) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->speed_limits != other.speed_limits) {
      return false;
    }
    if (this->deviation_limits != other.deviation_limits) {
      return false;
    }
    if (this->damping_factor != other.damping_factor) {
      return false;
    }
    if (this->gain_scaling != other.gain_scaling) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetForceMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetForceMode_Request_

// alias to use template instance with default allocator
using SetForceMode_Request =
  ur_msgs::srv::SetForceMode_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetForceMode_Request_<ContainerAllocator>::TCP_TO_ORIGIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetForceMode_Request_<ContainerAllocator>::NO_TRANSFORM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetForceMode_Request_<ContainerAllocator>::TCP_VELOCITY_TO_X_Y;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace ur_msgs


#ifndef _WIN32
# define DEPRECATED__ur_msgs__srv__SetForceMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__ur_msgs__srv__SetForceMode_Response __declspec(deprecated)
#endif

namespace ur_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetForceMode_Response_
{
  using Type = SetForceMode_Response_<ContainerAllocator>;

  explicit SetForceMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetForceMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ur_msgs::srv::SetForceMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ur_msgs::srv::SetForceMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ur_msgs::srv::SetForceMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ur_msgs::srv::SetForceMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ur_msgs::srv::SetForceMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ur_msgs::srv::SetForceMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ur_msgs::srv::SetForceMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ur_msgs::srv::SetForceMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ur_msgs::srv::SetForceMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ur_msgs::srv::SetForceMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ur_msgs__srv__SetForceMode_Response
    std::shared_ptr<ur_msgs::srv::SetForceMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ur_msgs__srv__SetForceMode_Response
    std::shared_ptr<ur_msgs::srv::SetForceMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetForceMode_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetForceMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetForceMode_Response_

// alias to use template instance with default allocator
using SetForceMode_Response =
  ur_msgs::srv::SetForceMode_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ur_msgs

namespace ur_msgs
{

namespace srv
{

struct SetForceMode
{
  using Request = ur_msgs::srv::SetForceMode_Request;
  using Response = ur_msgs::srv::SetForceMode_Response;
};

}  // namespace srv

}  // namespace ur_msgs

#endif  // UR_MSGS__SRV__DETAIL__SET_FORCE_MODE__STRUCT_HPP_
