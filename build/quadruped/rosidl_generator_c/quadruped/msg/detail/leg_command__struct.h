// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadruped/msg/leg_command.h"


#ifndef QUADRUPED__MSG__DETAIL__LEG_COMMAND__STRUCT_H_
#define QUADRUPED__MSG__DETAIL__LEG_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'cmd_type'
#include "rosidl_runtime_c/string.h"
// Member 'position'
// Member 'end_position'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/LegCommand in the package quadruped.
/**
  * This message contains commands to a leg. 
 */
typedef struct quadruped__msg__LegCommand
{
  /// command type (position trajectory, impulse trajectory, etc)
  rosidl_runtime_c__String cmd_type;
  /// position control commands
  /// desired position
  rosidl_runtime_c__double__Sequence position;
  /// position trajectory control commands
  /// desired end position [x,y]
  rosidl_runtime_c__double__Sequence end_position;
  /// the time at which the stance should be completed
  double position_end_time;
  /// proportional gain for position control
  double position_kp;
  /// derivative gain for position control
  double position_kd;
  /// impulse control commands
  /// the total impulse to be exerted in the stance phase
  double total_impulse;
  /// the time at which the stance should be completed
  double impulse_end_time;
} quadruped__msg__LegCommand;

// Struct for a sequence of quadruped__msg__LegCommand.
typedef struct quadruped__msg__LegCommand__Sequence
{
  quadruped__msg__LegCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadruped__msg__LegCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADRUPED__MSG__DETAIL__LEG_COMMAND__STRUCT_H_
