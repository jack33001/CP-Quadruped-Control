// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice
#ifndef QUADRUPED__MSG__DETAIL__LEG_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define QUADRUPED__MSG__DETAIL__LEG_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "quadruped/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "quadruped/msg/detail/leg_command__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
bool cdr_serialize_quadruped__msg__LegCommand(
  const quadruped__msg__LegCommand * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
bool cdr_deserialize_quadruped__msg__LegCommand(
  eprosima::fastcdr::Cdr &,
  quadruped__msg__LegCommand * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
size_t get_serialized_size_quadruped__msg__LegCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
size_t max_serialized_size_quadruped__msg__LegCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
bool cdr_serialize_key_quadruped__msg__LegCommand(
  const quadruped__msg__LegCommand * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
size_t get_serialized_size_key_quadruped__msg__LegCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
size_t max_serialized_size_key_quadruped__msg__LegCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, quadruped, msg, LegCommand)();

#ifdef __cplusplus
}
#endif

#endif  // QUADRUPED__MSG__DETAIL__LEG_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
