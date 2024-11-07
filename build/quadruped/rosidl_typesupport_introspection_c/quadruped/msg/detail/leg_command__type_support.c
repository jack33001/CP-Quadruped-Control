// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quadruped/msg/detail/leg_command__rosidl_typesupport_introspection_c.h"
#include "quadruped/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quadruped/msg/detail/leg_command__functions.h"
#include "quadruped/msg/detail/leg_command__struct.h"


// Include directives for member types
// Member `cmd_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `end_position`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quadruped__msg__LegCommand__init(message_memory);
}

void quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_fini_function(void * message_memory)
{
  quadruped__msg__LegCommand__fini(message_memory);
}

size_t quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__size_function__LegCommand__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_const_function__LegCommand__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_function__LegCommand__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__fetch_function__LegCommand__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_const_function__LegCommand__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__assign_function__LegCommand__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_function__LegCommand__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__resize_function__LegCommand__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__size_function__LegCommand__end_position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_const_function__LegCommand__end_position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_function__LegCommand__end_position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__fetch_function__LegCommand__end_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_const_function__LegCommand__end_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__assign_function__LegCommand__end_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_function__LegCommand__end_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__resize_function__LegCommand__end_position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_message_member_array[8] = {
  {
    "cmd_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadruped__msg__LegCommand, cmd_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadruped__msg__LegCommand, position),  // bytes offset in struct
    NULL,  // default value
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__size_function__LegCommand__position,  // size() function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_const_function__LegCommand__position,  // get_const(index) function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_function__LegCommand__position,  // get(index) function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__fetch_function__LegCommand__position,  // fetch(index, &value) function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__assign_function__LegCommand__position,  // assign(index, value) function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__resize_function__LegCommand__position  // resize(index) function pointer
  },
  {
    "end_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadruped__msg__LegCommand, end_position),  // bytes offset in struct
    NULL,  // default value
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__size_function__LegCommand__end_position,  // size() function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_const_function__LegCommand__end_position,  // get_const(index) function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__get_function__LegCommand__end_position,  // get(index) function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__fetch_function__LegCommand__end_position,  // fetch(index, &value) function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__assign_function__LegCommand__end_position,  // assign(index, value) function pointer
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__resize_function__LegCommand__end_position  // resize(index) function pointer
  },
  {
    "position_end_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadruped__msg__LegCommand, position_end_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadruped__msg__LegCommand, position_kp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadruped__msg__LegCommand, position_kd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_impulse",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadruped__msg__LegCommand, total_impulse),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "impulse_end_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadruped__msg__LegCommand, impulse_end_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_message_members = {
  "quadruped__msg",  // message namespace
  "LegCommand",  // message name
  8,  // number of fields
  sizeof(quadruped__msg__LegCommand),
  false,  // has_any_key_member_
  quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_message_member_array,  // message members
  quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_message_type_support_handle = {
  0,
  &quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_message_members,
  get_message_typesupport_handle_function,
  &quadruped__msg__LegCommand__get_type_hash,
  &quadruped__msg__LegCommand__get_type_description,
  &quadruped__msg__LegCommand__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quadruped
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quadruped, msg, LegCommand)() {
  if (!quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_message_type_support_handle.typesupport_identifier) {
    quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quadruped__msg__LegCommand__rosidl_typesupport_introspection_c__LegCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
