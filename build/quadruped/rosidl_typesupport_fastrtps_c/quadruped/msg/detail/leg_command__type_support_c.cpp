// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice
#include "quadruped/msg/detail/leg_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "quadruped/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "quadruped/msg/detail/leg_command__struct.h"
#include "quadruped/msg/detail/leg_command__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // end_position, position
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // end_position, position
#include "rosidl_runtime_c/string.h"  // cmd_type
#include "rosidl_runtime_c/string_functions.h"  // cmd_type

// forward declare type support functions


using _LegCommand__ros_msg_type = quadruped__msg__LegCommand;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
bool cdr_serialize_quadruped__msg__LegCommand(
  const quadruped__msg__LegCommand * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: cmd_type
  {
    const rosidl_runtime_c__String * str = &ros_message->cmd_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: position
  {
    size_t size = ros_message->position.size;
    auto array_ptr = ros_message->position.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: end_position
  {
    size_t size = ros_message->end_position.size;
    auto array_ptr = ros_message->end_position.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: position_end_time
  {
    cdr << ros_message->position_end_time;
  }

  // Field name: position_kp
  {
    cdr << ros_message->position_kp;
  }

  // Field name: position_kd
  {
    cdr << ros_message->position_kd;
  }

  // Field name: total_impulse
  {
    cdr << ros_message->total_impulse;
  }

  // Field name: impulse_end_time
  {
    cdr << ros_message->impulse_end_time;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
bool cdr_deserialize_quadruped__msg__LegCommand(
  eprosima::fastcdr::Cdr & cdr,
  quadruped__msg__LegCommand * ros_message)
{
  // Field name: cmd_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->cmd_type.data) {
      rosidl_runtime_c__String__init(&ros_message->cmd_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->cmd_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'cmd_type'\n");
      return false;
    }
  }

  // Field name: position
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->position.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->position);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->position, size)) {
      fprintf(stderr, "failed to create array for field 'position'");
      return false;
    }
    auto array_ptr = ros_message->position.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: end_position
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->end_position.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->end_position);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->end_position, size)) {
      fprintf(stderr, "failed to create array for field 'end_position'");
      return false;
    }
    auto array_ptr = ros_message->end_position.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: position_end_time
  {
    cdr >> ros_message->position_end_time;
  }

  // Field name: position_kp
  {
    cdr >> ros_message->position_kp;
  }

  // Field name: position_kd
  {
    cdr >> ros_message->position_kd;
  }

  // Field name: total_impulse
  {
    cdr >> ros_message->total_impulse;
  }

  // Field name: impulse_end_time
  {
    cdr >> ros_message->impulse_end_time;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
size_t get_serialized_size_quadruped__msg__LegCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LegCommand__ros_msg_type * ros_message = static_cast<const _LegCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: cmd_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->cmd_type.size + 1);

  // Field name: position
  {
    size_t array_size = ros_message->position.size;
    auto array_ptr = ros_message->position.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: end_position
  {
    size_t array_size = ros_message->end_position.size;
    auto array_ptr = ros_message->end_position.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: position_end_time
  {
    size_t item_size = sizeof(ros_message->position_end_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: position_kp
  {
    size_t item_size = sizeof(ros_message->position_kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: position_kd
  {
    size_t item_size = sizeof(ros_message->position_kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: total_impulse
  {
    size_t item_size = sizeof(ros_message->total_impulse);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: impulse_end_time
  {
    size_t item_size = sizeof(ros_message->impulse_end_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
size_t max_serialized_size_quadruped__msg__LegCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: cmd_type
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: position
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: end_position
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: position_end_time
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: position_kp
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: position_kd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: total_impulse
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: impulse_end_time
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = quadruped__msg__LegCommand;
    is_plain =
      (
      offsetof(DataType, impulse_end_time) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
bool cdr_serialize_key_quadruped__msg__LegCommand(
  const quadruped__msg__LegCommand * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: cmd_type
  {
    const rosidl_runtime_c__String * str = &ros_message->cmd_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: position
  {
    size_t size = ros_message->position.size;
    auto array_ptr = ros_message->position.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: end_position
  {
    size_t size = ros_message->end_position.size;
    auto array_ptr = ros_message->end_position.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: position_end_time
  {
    cdr << ros_message->position_end_time;
  }

  // Field name: position_kp
  {
    cdr << ros_message->position_kp;
  }

  // Field name: position_kd
  {
    cdr << ros_message->position_kd;
  }

  // Field name: total_impulse
  {
    cdr << ros_message->total_impulse;
  }

  // Field name: impulse_end_time
  {
    cdr << ros_message->impulse_end_time;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
size_t get_serialized_size_key_quadruped__msg__LegCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LegCommand__ros_msg_type * ros_message = static_cast<const _LegCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: cmd_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->cmd_type.size + 1);

  // Field name: position
  {
    size_t array_size = ros_message->position.size;
    auto array_ptr = ros_message->position.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: end_position
  {
    size_t array_size = ros_message->end_position.size;
    auto array_ptr = ros_message->end_position.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: position_end_time
  {
    size_t item_size = sizeof(ros_message->position_end_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: position_kp
  {
    size_t item_size = sizeof(ros_message->position_kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: position_kd
  {
    size_t item_size = sizeof(ros_message->position_kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: total_impulse
  {
    size_t item_size = sizeof(ros_message->total_impulse);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: impulse_end_time
  {
    size_t item_size = sizeof(ros_message->impulse_end_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadruped
size_t max_serialized_size_key_quadruped__msg__LegCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: cmd_type
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: position
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: end_position
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: position_end_time
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: position_kp
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: position_kd
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: total_impulse
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: impulse_end_time
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = quadruped__msg__LegCommand;
    is_plain =
      (
      offsetof(DataType, impulse_end_time) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _LegCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const quadruped__msg__LegCommand * ros_message = static_cast<const quadruped__msg__LegCommand *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_quadruped__msg__LegCommand(ros_message, cdr);
}

static bool _LegCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  quadruped__msg__LegCommand * ros_message = static_cast<quadruped__msg__LegCommand *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_quadruped__msg__LegCommand(cdr, ros_message);
}

static uint32_t _LegCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_quadruped__msg__LegCommand(
      untyped_ros_message, 0));
}

static size_t _LegCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_quadruped__msg__LegCommand(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LegCommand = {
  "quadruped::msg",
  "LegCommand",
  _LegCommand__cdr_serialize,
  _LegCommand__cdr_deserialize,
  _LegCommand__get_serialized_size,
  _LegCommand__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _LegCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LegCommand,
  get_message_typesupport_handle_function,
  &quadruped__msg__LegCommand__get_type_hash,
  &quadruped__msg__LegCommand__get_type_description,
  &quadruped__msg__LegCommand__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, quadruped, msg, LegCommand)() {
  return &_LegCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
