// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadruped/msg/leg_command.hpp"


#ifndef QUADRUPED__MSG__DETAIL__LEG_COMMAND__STRUCT_HPP_
#define QUADRUPED__MSG__DETAIL__LEG_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__quadruped__msg__LegCommand __attribute__((deprecated))
#else
# define DEPRECATED__quadruped__msg__LegCommand __declspec(deprecated)
#endif

namespace quadruped
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LegCommand_
{
  using Type = LegCommand_<ContainerAllocator>;

  explicit LegCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd_type = "";
      this->position_end_time = 0.0;
      this->position_kp = 0.0;
      this->position_kd = 0.0;
      this->total_impulse = 0.0;
      this->impulse_end_time = 0.0;
    }
  }

  explicit LegCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cmd_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd_type = "";
      this->position_end_time = 0.0;
      this->position_kp = 0.0;
      this->position_kd = 0.0;
      this->total_impulse = 0.0;
      this->impulse_end_time = 0.0;
    }
  }

  // field types and members
  using _cmd_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cmd_type_type cmd_type;
  using _position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _position_type position;
  using _end_position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _end_position_type end_position;
  using _position_end_time_type =
    double;
  _position_end_time_type position_end_time;
  using _position_kp_type =
    double;
  _position_kp_type position_kp;
  using _position_kd_type =
    double;
  _position_kd_type position_kd;
  using _total_impulse_type =
    double;
  _total_impulse_type total_impulse;
  using _impulse_end_time_type =
    double;
  _impulse_end_time_type impulse_end_time;

  // setters for named parameter idiom
  Type & set__cmd_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cmd_type = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__end_position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->end_position = _arg;
    return *this;
  }
  Type & set__position_end_time(
    const double & _arg)
  {
    this->position_end_time = _arg;
    return *this;
  }
  Type & set__position_kp(
    const double & _arg)
  {
    this->position_kp = _arg;
    return *this;
  }
  Type & set__position_kd(
    const double & _arg)
  {
    this->position_kd = _arg;
    return *this;
  }
  Type & set__total_impulse(
    const double & _arg)
  {
    this->total_impulse = _arg;
    return *this;
  }
  Type & set__impulse_end_time(
    const double & _arg)
  {
    this->impulse_end_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadruped::msg::LegCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadruped::msg::LegCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadruped::msg::LegCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadruped::msg::LegCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadruped::msg::LegCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadruped::msg::LegCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadruped::msg::LegCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadruped::msg::LegCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadruped::msg::LegCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadruped::msg::LegCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadruped__msg__LegCommand
    std::shared_ptr<quadruped::msg::LegCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadruped__msg__LegCommand
    std::shared_ptr<quadruped::msg::LegCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LegCommand_ & other) const
  {
    if (this->cmd_type != other.cmd_type) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->end_position != other.end_position) {
      return false;
    }
    if (this->position_end_time != other.position_end_time) {
      return false;
    }
    if (this->position_kp != other.position_kp) {
      return false;
    }
    if (this->position_kd != other.position_kd) {
      return false;
    }
    if (this->total_impulse != other.total_impulse) {
      return false;
    }
    if (this->impulse_end_time != other.impulse_end_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const LegCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LegCommand_

// alias to use template instance with default allocator
using LegCommand =
  quadruped::msg::LegCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadruped

#endif  // QUADRUPED__MSG__DETAIL__LEG_COMMAND__STRUCT_HPP_
