// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadruped/msg/leg_command.hpp"


#ifndef QUADRUPED__MSG__DETAIL__LEG_COMMAND__BUILDER_HPP_
#define QUADRUPED__MSG__DETAIL__LEG_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadruped/msg/detail/leg_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadruped
{

namespace msg
{

namespace builder
{

class Init_LegCommand_impulse_end_time
{
public:
  explicit Init_LegCommand_impulse_end_time(::quadruped::msg::LegCommand & msg)
  : msg_(msg)
  {}
  ::quadruped::msg::LegCommand impulse_end_time(::quadruped::msg::LegCommand::_impulse_end_time_type arg)
  {
    msg_.impulse_end_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadruped::msg::LegCommand msg_;
};

class Init_LegCommand_total_impulse
{
public:
  explicit Init_LegCommand_total_impulse(::quadruped::msg::LegCommand & msg)
  : msg_(msg)
  {}
  Init_LegCommand_impulse_end_time total_impulse(::quadruped::msg::LegCommand::_total_impulse_type arg)
  {
    msg_.total_impulse = std::move(arg);
    return Init_LegCommand_impulse_end_time(msg_);
  }

private:
  ::quadruped::msg::LegCommand msg_;
};

class Init_LegCommand_position_kd
{
public:
  explicit Init_LegCommand_position_kd(::quadruped::msg::LegCommand & msg)
  : msg_(msg)
  {}
  Init_LegCommand_total_impulse position_kd(::quadruped::msg::LegCommand::_position_kd_type arg)
  {
    msg_.position_kd = std::move(arg);
    return Init_LegCommand_total_impulse(msg_);
  }

private:
  ::quadruped::msg::LegCommand msg_;
};

class Init_LegCommand_position_kp
{
public:
  explicit Init_LegCommand_position_kp(::quadruped::msg::LegCommand & msg)
  : msg_(msg)
  {}
  Init_LegCommand_position_kd position_kp(::quadruped::msg::LegCommand::_position_kp_type arg)
  {
    msg_.position_kp = std::move(arg);
    return Init_LegCommand_position_kd(msg_);
  }

private:
  ::quadruped::msg::LegCommand msg_;
};

class Init_LegCommand_position_end_time
{
public:
  explicit Init_LegCommand_position_end_time(::quadruped::msg::LegCommand & msg)
  : msg_(msg)
  {}
  Init_LegCommand_position_kp position_end_time(::quadruped::msg::LegCommand::_position_end_time_type arg)
  {
    msg_.position_end_time = std::move(arg);
    return Init_LegCommand_position_kp(msg_);
  }

private:
  ::quadruped::msg::LegCommand msg_;
};

class Init_LegCommand_end_position
{
public:
  explicit Init_LegCommand_end_position(::quadruped::msg::LegCommand & msg)
  : msg_(msg)
  {}
  Init_LegCommand_position_end_time end_position(::quadruped::msg::LegCommand::_end_position_type arg)
  {
    msg_.end_position = std::move(arg);
    return Init_LegCommand_position_end_time(msg_);
  }

private:
  ::quadruped::msg::LegCommand msg_;
};

class Init_LegCommand_position
{
public:
  explicit Init_LegCommand_position(::quadruped::msg::LegCommand & msg)
  : msg_(msg)
  {}
  Init_LegCommand_end_position position(::quadruped::msg::LegCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_LegCommand_end_position(msg_);
  }

private:
  ::quadruped::msg::LegCommand msg_;
};

class Init_LegCommand_cmd_type
{
public:
  Init_LegCommand_cmd_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LegCommand_position cmd_type(::quadruped::msg::LegCommand::_cmd_type_type arg)
  {
    msg_.cmd_type = std::move(arg);
    return Init_LegCommand_position(msg_);
  }

private:
  ::quadruped::msg::LegCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadruped::msg::LegCommand>()
{
  return quadruped::msg::builder::Init_LegCommand_cmd_type();
}

}  // namespace quadruped

#endif  // QUADRUPED__MSG__DETAIL__LEG_COMMAND__BUILDER_HPP_
