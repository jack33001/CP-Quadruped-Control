// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice

#include "quadruped/msg/detail/leg_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_quadruped
const rosidl_type_hash_t *
quadruped__msg__LegCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x02, 0x7e, 0xab, 0x15, 0xe3, 0x0a, 0xb8, 0xd5,
      0x89, 0x36, 0x9a, 0x78, 0x23, 0x02, 0x28, 0xca,
      0xd2, 0x03, 0x49, 0xe3, 0x0d, 0x31, 0xc7, 0xf8,
      0x21, 0x68, 0x25, 0xb9, 0x70, 0x5f, 0x6e, 0x61,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char quadruped__msg__LegCommand__TYPE_NAME[] = "quadruped/msg/LegCommand";

// Define type names, field names, and default values
static char quadruped__msg__LegCommand__FIELD_NAME__cmd_type[] = "cmd_type";
static char quadruped__msg__LegCommand__FIELD_NAME__position[] = "position";
static char quadruped__msg__LegCommand__FIELD_NAME__end_position[] = "end_position";
static char quadruped__msg__LegCommand__FIELD_NAME__position_end_time[] = "position_end_time";
static char quadruped__msg__LegCommand__FIELD_NAME__position_kp[] = "position_kp";
static char quadruped__msg__LegCommand__FIELD_NAME__position_kd[] = "position_kd";
static char quadruped__msg__LegCommand__FIELD_NAME__total_impulse[] = "total_impulse";
static char quadruped__msg__LegCommand__FIELD_NAME__impulse_end_time[] = "impulse_end_time";

static rosidl_runtime_c__type_description__Field quadruped__msg__LegCommand__FIELDS[] = {
  {
    {quadruped__msg__LegCommand__FIELD_NAME__cmd_type, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadruped__msg__LegCommand__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadruped__msg__LegCommand__FIELD_NAME__end_position, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadruped__msg__LegCommand__FIELD_NAME__position_end_time, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadruped__msg__LegCommand__FIELD_NAME__position_kp, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadruped__msg__LegCommand__FIELD_NAME__position_kd, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadruped__msg__LegCommand__FIELD_NAME__total_impulse, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {quadruped__msg__LegCommand__FIELD_NAME__impulse_end_time, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
quadruped__msg__LegCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {quadruped__msg__LegCommand__TYPE_NAME, 24, 24},
      {quadruped__msg__LegCommand__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This message contains commands to a leg. \n"
  "\n"
  "string      cmd_type            # command type (position trajectory, impulse trajectory, etc)\n"
  "\n"
  "# position control commands\n"
  "float64[]   position            # desired position\n"
  "\n"
  "# position trajectory control commands\n"
  "float64[]   end_position        # desired end position [x,y]\n"
  "float64     position_end_time   # the time at which the stance should be completed\n"
  "float64     position_kp         # proportional gain for position control\n"
  "float64     position_kd         # derivative gain for position control\n"
  "\n"
  "# impulse control commands\n"
  "float64     total_impulse       # the total impulse to be exerted in the stance phase\n"
  "float64     impulse_end_time    # the time at which the stance should be completed";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
quadruped__msg__LegCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {quadruped__msg__LegCommand__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 744, 744},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
quadruped__msg__LegCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *quadruped__msg__LegCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
