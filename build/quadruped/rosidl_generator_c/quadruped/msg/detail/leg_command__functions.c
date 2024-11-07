// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadruped:msg/LegCommand.idl
// generated code does not contain a copyright notice
#include "quadruped/msg/detail/leg_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `cmd_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `end_position`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
quadruped__msg__LegCommand__init(quadruped__msg__LegCommand * msg)
{
  if (!msg) {
    return false;
  }
  // cmd_type
  if (!rosidl_runtime_c__String__init(&msg->cmd_type)) {
    quadruped__msg__LegCommand__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    quadruped__msg__LegCommand__fini(msg);
    return false;
  }
  // end_position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->end_position, 0)) {
    quadruped__msg__LegCommand__fini(msg);
    return false;
  }
  // position_end_time
  // position_kp
  // position_kd
  // total_impulse
  // impulse_end_time
  return true;
}

void
quadruped__msg__LegCommand__fini(quadruped__msg__LegCommand * msg)
{
  if (!msg) {
    return;
  }
  // cmd_type
  rosidl_runtime_c__String__fini(&msg->cmd_type);
  // position
  rosidl_runtime_c__double__Sequence__fini(&msg->position);
  // end_position
  rosidl_runtime_c__double__Sequence__fini(&msg->end_position);
  // position_end_time
  // position_kp
  // position_kd
  // total_impulse
  // impulse_end_time
}

bool
quadruped__msg__LegCommand__are_equal(const quadruped__msg__LegCommand * lhs, const quadruped__msg__LegCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cmd_type), &(rhs->cmd_type)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // end_position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->end_position), &(rhs->end_position)))
  {
    return false;
  }
  // position_end_time
  if (lhs->position_end_time != rhs->position_end_time) {
    return false;
  }
  // position_kp
  if (lhs->position_kp != rhs->position_kp) {
    return false;
  }
  // position_kd
  if (lhs->position_kd != rhs->position_kd) {
    return false;
  }
  // total_impulse
  if (lhs->total_impulse != rhs->total_impulse) {
    return false;
  }
  // impulse_end_time
  if (lhs->impulse_end_time != rhs->impulse_end_time) {
    return false;
  }
  return true;
}

bool
quadruped__msg__LegCommand__copy(
  const quadruped__msg__LegCommand * input,
  quadruped__msg__LegCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd_type
  if (!rosidl_runtime_c__String__copy(
      &(input->cmd_type), &(output->cmd_type)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // end_position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->end_position), &(output->end_position)))
  {
    return false;
  }
  // position_end_time
  output->position_end_time = input->position_end_time;
  // position_kp
  output->position_kp = input->position_kp;
  // position_kd
  output->position_kd = input->position_kd;
  // total_impulse
  output->total_impulse = input->total_impulse;
  // impulse_end_time
  output->impulse_end_time = input->impulse_end_time;
  return true;
}

quadruped__msg__LegCommand *
quadruped__msg__LegCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadruped__msg__LegCommand * msg = (quadruped__msg__LegCommand *)allocator.allocate(sizeof(quadruped__msg__LegCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadruped__msg__LegCommand));
  bool success = quadruped__msg__LegCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadruped__msg__LegCommand__destroy(quadruped__msg__LegCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadruped__msg__LegCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadruped__msg__LegCommand__Sequence__init(quadruped__msg__LegCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadruped__msg__LegCommand * data = NULL;

  if (size) {
    data = (quadruped__msg__LegCommand *)allocator.zero_allocate(size, sizeof(quadruped__msg__LegCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadruped__msg__LegCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadruped__msg__LegCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
quadruped__msg__LegCommand__Sequence__fini(quadruped__msg__LegCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      quadruped__msg__LegCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

quadruped__msg__LegCommand__Sequence *
quadruped__msg__LegCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadruped__msg__LegCommand__Sequence * array = (quadruped__msg__LegCommand__Sequence *)allocator.allocate(sizeof(quadruped__msg__LegCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadruped__msg__LegCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadruped__msg__LegCommand__Sequence__destroy(quadruped__msg__LegCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadruped__msg__LegCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadruped__msg__LegCommand__Sequence__are_equal(const quadruped__msg__LegCommand__Sequence * lhs, const quadruped__msg__LegCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadruped__msg__LegCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadruped__msg__LegCommand__Sequence__copy(
  const quadruped__msg__LegCommand__Sequence * input,
  quadruped__msg__LegCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadruped__msg__LegCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadruped__msg__LegCommand * data =
      (quadruped__msg__LegCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadruped__msg__LegCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadruped__msg__LegCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadruped__msg__LegCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
