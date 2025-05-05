// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ur_msgs:action/ToolContact.idl
// generated code does not contain a copyright notice
#include "ur_msgs/action/detail/tool_contact__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ur_msgs__action__ToolContact_Goal__init(ur_msgs__action__ToolContact_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ur_msgs__action__ToolContact_Goal__fini(ur_msgs__action__ToolContact_Goal * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ur_msgs__action__ToolContact_Goal__are_equal(const ur_msgs__action__ToolContact_Goal * lhs, const ur_msgs__action__ToolContact_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ur_msgs__action__ToolContact_Goal__copy(
  const ur_msgs__action__ToolContact_Goal * input,
  ur_msgs__action__ToolContact_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ur_msgs__action__ToolContact_Goal *
ur_msgs__action__ToolContact_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Goal * msg = (ur_msgs__action__ToolContact_Goal *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__action__ToolContact_Goal));
  bool success = ur_msgs__action__ToolContact_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__action__ToolContact_Goal__destroy(ur_msgs__action__ToolContact_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__action__ToolContact_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__action__ToolContact_Goal__Sequence__init(ur_msgs__action__ToolContact_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Goal * data = NULL;

  if (size) {
    data = (ur_msgs__action__ToolContact_Goal *)allocator.zero_allocate(size, sizeof(ur_msgs__action__ToolContact_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__action__ToolContact_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__action__ToolContact_Goal__fini(&data[i - 1]);
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
ur_msgs__action__ToolContact_Goal__Sequence__fini(ur_msgs__action__ToolContact_Goal__Sequence * array)
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
      ur_msgs__action__ToolContact_Goal__fini(&array->data[i]);
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

ur_msgs__action__ToolContact_Goal__Sequence *
ur_msgs__action__ToolContact_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Goal__Sequence * array = (ur_msgs__action__ToolContact_Goal__Sequence *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__action__ToolContact_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__action__ToolContact_Goal__Sequence__destroy(ur_msgs__action__ToolContact_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__action__ToolContact_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__action__ToolContact_Goal__Sequence__are_equal(const ur_msgs__action__ToolContact_Goal__Sequence * lhs, const ur_msgs__action__ToolContact_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__action__ToolContact_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__action__ToolContact_Goal__Sequence__copy(
  const ur_msgs__action__ToolContact_Goal__Sequence * input,
  ur_msgs__action__ToolContact_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__action__ToolContact_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__action__ToolContact_Goal * data =
      (ur_msgs__action__ToolContact_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__action__ToolContact_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__action__ToolContact_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__action__ToolContact_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ur_msgs__action__ToolContact_Result__init(ur_msgs__action__ToolContact_Result * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ur_msgs__action__ToolContact_Result__fini(ur_msgs__action__ToolContact_Result * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ur_msgs__action__ToolContact_Result__are_equal(const ur_msgs__action__ToolContact_Result * lhs, const ur_msgs__action__ToolContact_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ur_msgs__action__ToolContact_Result__copy(
  const ur_msgs__action__ToolContact_Result * input,
  ur_msgs__action__ToolContact_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ur_msgs__action__ToolContact_Result *
ur_msgs__action__ToolContact_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Result * msg = (ur_msgs__action__ToolContact_Result *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__action__ToolContact_Result));
  bool success = ur_msgs__action__ToolContact_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__action__ToolContact_Result__destroy(ur_msgs__action__ToolContact_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__action__ToolContact_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__action__ToolContact_Result__Sequence__init(ur_msgs__action__ToolContact_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Result * data = NULL;

  if (size) {
    data = (ur_msgs__action__ToolContact_Result *)allocator.zero_allocate(size, sizeof(ur_msgs__action__ToolContact_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__action__ToolContact_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__action__ToolContact_Result__fini(&data[i - 1]);
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
ur_msgs__action__ToolContact_Result__Sequence__fini(ur_msgs__action__ToolContact_Result__Sequence * array)
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
      ur_msgs__action__ToolContact_Result__fini(&array->data[i]);
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

ur_msgs__action__ToolContact_Result__Sequence *
ur_msgs__action__ToolContact_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Result__Sequence * array = (ur_msgs__action__ToolContact_Result__Sequence *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__action__ToolContact_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__action__ToolContact_Result__Sequence__destroy(ur_msgs__action__ToolContact_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__action__ToolContact_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__action__ToolContact_Result__Sequence__are_equal(const ur_msgs__action__ToolContact_Result__Sequence * lhs, const ur_msgs__action__ToolContact_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__action__ToolContact_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__action__ToolContact_Result__Sequence__copy(
  const ur_msgs__action__ToolContact_Result__Sequence * input,
  ur_msgs__action__ToolContact_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__action__ToolContact_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__action__ToolContact_Result * data =
      (ur_msgs__action__ToolContact_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__action__ToolContact_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__action__ToolContact_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__action__ToolContact_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ur_msgs__action__ToolContact_Feedback__init(ur_msgs__action__ToolContact_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ur_msgs__action__ToolContact_Feedback__fini(ur_msgs__action__ToolContact_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ur_msgs__action__ToolContact_Feedback__are_equal(const ur_msgs__action__ToolContact_Feedback * lhs, const ur_msgs__action__ToolContact_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ur_msgs__action__ToolContact_Feedback__copy(
  const ur_msgs__action__ToolContact_Feedback * input,
  ur_msgs__action__ToolContact_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ur_msgs__action__ToolContact_Feedback *
ur_msgs__action__ToolContact_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Feedback * msg = (ur_msgs__action__ToolContact_Feedback *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__action__ToolContact_Feedback));
  bool success = ur_msgs__action__ToolContact_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__action__ToolContact_Feedback__destroy(ur_msgs__action__ToolContact_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__action__ToolContact_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__action__ToolContact_Feedback__Sequence__init(ur_msgs__action__ToolContact_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Feedback * data = NULL;

  if (size) {
    data = (ur_msgs__action__ToolContact_Feedback *)allocator.zero_allocate(size, sizeof(ur_msgs__action__ToolContact_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__action__ToolContact_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__action__ToolContact_Feedback__fini(&data[i - 1]);
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
ur_msgs__action__ToolContact_Feedback__Sequence__fini(ur_msgs__action__ToolContact_Feedback__Sequence * array)
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
      ur_msgs__action__ToolContact_Feedback__fini(&array->data[i]);
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

ur_msgs__action__ToolContact_Feedback__Sequence *
ur_msgs__action__ToolContact_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_Feedback__Sequence * array = (ur_msgs__action__ToolContact_Feedback__Sequence *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__action__ToolContact_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__action__ToolContact_Feedback__Sequence__destroy(ur_msgs__action__ToolContact_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__action__ToolContact_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__action__ToolContact_Feedback__Sequence__are_equal(const ur_msgs__action__ToolContact_Feedback__Sequence * lhs, const ur_msgs__action__ToolContact_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__action__ToolContact_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__action__ToolContact_Feedback__Sequence__copy(
  const ur_msgs__action__ToolContact_Feedback__Sequence * input,
  ur_msgs__action__ToolContact_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__action__ToolContact_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__action__ToolContact_Feedback * data =
      (ur_msgs__action__ToolContact_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__action__ToolContact_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__action__ToolContact_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__action__ToolContact_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "ur_msgs/action/detail/tool_contact__functions.h"

bool
ur_msgs__action__ToolContact_SendGoal_Request__init(ur_msgs__action__ToolContact_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ur_msgs__action__ToolContact_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!ur_msgs__action__ToolContact_Goal__init(&msg->goal)) {
    ur_msgs__action__ToolContact_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
ur_msgs__action__ToolContact_SendGoal_Request__fini(ur_msgs__action__ToolContact_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  ur_msgs__action__ToolContact_Goal__fini(&msg->goal);
}

bool
ur_msgs__action__ToolContact_SendGoal_Request__are_equal(const ur_msgs__action__ToolContact_SendGoal_Request * lhs, const ur_msgs__action__ToolContact_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!ur_msgs__action__ToolContact_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
ur_msgs__action__ToolContact_SendGoal_Request__copy(
  const ur_msgs__action__ToolContact_SendGoal_Request * input,
  ur_msgs__action__ToolContact_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!ur_msgs__action__ToolContact_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

ur_msgs__action__ToolContact_SendGoal_Request *
ur_msgs__action__ToolContact_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_SendGoal_Request * msg = (ur_msgs__action__ToolContact_SendGoal_Request *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__action__ToolContact_SendGoal_Request));
  bool success = ur_msgs__action__ToolContact_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__action__ToolContact_SendGoal_Request__destroy(ur_msgs__action__ToolContact_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__action__ToolContact_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__action__ToolContact_SendGoal_Request__Sequence__init(ur_msgs__action__ToolContact_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_SendGoal_Request * data = NULL;

  if (size) {
    data = (ur_msgs__action__ToolContact_SendGoal_Request *)allocator.zero_allocate(size, sizeof(ur_msgs__action__ToolContact_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__action__ToolContact_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__action__ToolContact_SendGoal_Request__fini(&data[i - 1]);
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
ur_msgs__action__ToolContact_SendGoal_Request__Sequence__fini(ur_msgs__action__ToolContact_SendGoal_Request__Sequence * array)
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
      ur_msgs__action__ToolContact_SendGoal_Request__fini(&array->data[i]);
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

ur_msgs__action__ToolContact_SendGoal_Request__Sequence *
ur_msgs__action__ToolContact_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_SendGoal_Request__Sequence * array = (ur_msgs__action__ToolContact_SendGoal_Request__Sequence *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__action__ToolContact_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__action__ToolContact_SendGoal_Request__Sequence__destroy(ur_msgs__action__ToolContact_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__action__ToolContact_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__action__ToolContact_SendGoal_Request__Sequence__are_equal(const ur_msgs__action__ToolContact_SendGoal_Request__Sequence * lhs, const ur_msgs__action__ToolContact_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__action__ToolContact_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__action__ToolContact_SendGoal_Request__Sequence__copy(
  const ur_msgs__action__ToolContact_SendGoal_Request__Sequence * input,
  ur_msgs__action__ToolContact_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__action__ToolContact_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__action__ToolContact_SendGoal_Request * data =
      (ur_msgs__action__ToolContact_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__action__ToolContact_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__action__ToolContact_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__action__ToolContact_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
ur_msgs__action__ToolContact_SendGoal_Response__init(ur_msgs__action__ToolContact_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    ur_msgs__action__ToolContact_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
ur_msgs__action__ToolContact_SendGoal_Response__fini(ur_msgs__action__ToolContact_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
ur_msgs__action__ToolContact_SendGoal_Response__are_equal(const ur_msgs__action__ToolContact_SendGoal_Response * lhs, const ur_msgs__action__ToolContact_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
ur_msgs__action__ToolContact_SendGoal_Response__copy(
  const ur_msgs__action__ToolContact_SendGoal_Response * input,
  ur_msgs__action__ToolContact_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

ur_msgs__action__ToolContact_SendGoal_Response *
ur_msgs__action__ToolContact_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_SendGoal_Response * msg = (ur_msgs__action__ToolContact_SendGoal_Response *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__action__ToolContact_SendGoal_Response));
  bool success = ur_msgs__action__ToolContact_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__action__ToolContact_SendGoal_Response__destroy(ur_msgs__action__ToolContact_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__action__ToolContact_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__action__ToolContact_SendGoal_Response__Sequence__init(ur_msgs__action__ToolContact_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_SendGoal_Response * data = NULL;

  if (size) {
    data = (ur_msgs__action__ToolContact_SendGoal_Response *)allocator.zero_allocate(size, sizeof(ur_msgs__action__ToolContact_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__action__ToolContact_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__action__ToolContact_SendGoal_Response__fini(&data[i - 1]);
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
ur_msgs__action__ToolContact_SendGoal_Response__Sequence__fini(ur_msgs__action__ToolContact_SendGoal_Response__Sequence * array)
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
      ur_msgs__action__ToolContact_SendGoal_Response__fini(&array->data[i]);
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

ur_msgs__action__ToolContact_SendGoal_Response__Sequence *
ur_msgs__action__ToolContact_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_SendGoal_Response__Sequence * array = (ur_msgs__action__ToolContact_SendGoal_Response__Sequence *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__action__ToolContact_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__action__ToolContact_SendGoal_Response__Sequence__destroy(ur_msgs__action__ToolContact_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__action__ToolContact_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__action__ToolContact_SendGoal_Response__Sequence__are_equal(const ur_msgs__action__ToolContact_SendGoal_Response__Sequence * lhs, const ur_msgs__action__ToolContact_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__action__ToolContact_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__action__ToolContact_SendGoal_Response__Sequence__copy(
  const ur_msgs__action__ToolContact_SendGoal_Response__Sequence * input,
  ur_msgs__action__ToolContact_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__action__ToolContact_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__action__ToolContact_SendGoal_Response * data =
      (ur_msgs__action__ToolContact_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__action__ToolContact_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__action__ToolContact_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__action__ToolContact_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
ur_msgs__action__ToolContact_GetResult_Request__init(ur_msgs__action__ToolContact_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ur_msgs__action__ToolContact_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
ur_msgs__action__ToolContact_GetResult_Request__fini(ur_msgs__action__ToolContact_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
ur_msgs__action__ToolContact_GetResult_Request__are_equal(const ur_msgs__action__ToolContact_GetResult_Request * lhs, const ur_msgs__action__ToolContact_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
ur_msgs__action__ToolContact_GetResult_Request__copy(
  const ur_msgs__action__ToolContact_GetResult_Request * input,
  ur_msgs__action__ToolContact_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

ur_msgs__action__ToolContact_GetResult_Request *
ur_msgs__action__ToolContact_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_GetResult_Request * msg = (ur_msgs__action__ToolContact_GetResult_Request *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__action__ToolContact_GetResult_Request));
  bool success = ur_msgs__action__ToolContact_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__action__ToolContact_GetResult_Request__destroy(ur_msgs__action__ToolContact_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__action__ToolContact_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__action__ToolContact_GetResult_Request__Sequence__init(ur_msgs__action__ToolContact_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_GetResult_Request * data = NULL;

  if (size) {
    data = (ur_msgs__action__ToolContact_GetResult_Request *)allocator.zero_allocate(size, sizeof(ur_msgs__action__ToolContact_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__action__ToolContact_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__action__ToolContact_GetResult_Request__fini(&data[i - 1]);
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
ur_msgs__action__ToolContact_GetResult_Request__Sequence__fini(ur_msgs__action__ToolContact_GetResult_Request__Sequence * array)
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
      ur_msgs__action__ToolContact_GetResult_Request__fini(&array->data[i]);
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

ur_msgs__action__ToolContact_GetResult_Request__Sequence *
ur_msgs__action__ToolContact_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_GetResult_Request__Sequence * array = (ur_msgs__action__ToolContact_GetResult_Request__Sequence *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__action__ToolContact_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__action__ToolContact_GetResult_Request__Sequence__destroy(ur_msgs__action__ToolContact_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__action__ToolContact_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__action__ToolContact_GetResult_Request__Sequence__are_equal(const ur_msgs__action__ToolContact_GetResult_Request__Sequence * lhs, const ur_msgs__action__ToolContact_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__action__ToolContact_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__action__ToolContact_GetResult_Request__Sequence__copy(
  const ur_msgs__action__ToolContact_GetResult_Request__Sequence * input,
  ur_msgs__action__ToolContact_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__action__ToolContact_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__action__ToolContact_GetResult_Request * data =
      (ur_msgs__action__ToolContact_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__action__ToolContact_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__action__ToolContact_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__action__ToolContact_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "ur_msgs/action/detail/tool_contact__functions.h"

bool
ur_msgs__action__ToolContact_GetResult_Response__init(ur_msgs__action__ToolContact_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!ur_msgs__action__ToolContact_Result__init(&msg->result)) {
    ur_msgs__action__ToolContact_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
ur_msgs__action__ToolContact_GetResult_Response__fini(ur_msgs__action__ToolContact_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  ur_msgs__action__ToolContact_Result__fini(&msg->result);
}

bool
ur_msgs__action__ToolContact_GetResult_Response__are_equal(const ur_msgs__action__ToolContact_GetResult_Response * lhs, const ur_msgs__action__ToolContact_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!ur_msgs__action__ToolContact_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
ur_msgs__action__ToolContact_GetResult_Response__copy(
  const ur_msgs__action__ToolContact_GetResult_Response * input,
  ur_msgs__action__ToolContact_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!ur_msgs__action__ToolContact_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

ur_msgs__action__ToolContact_GetResult_Response *
ur_msgs__action__ToolContact_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_GetResult_Response * msg = (ur_msgs__action__ToolContact_GetResult_Response *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__action__ToolContact_GetResult_Response));
  bool success = ur_msgs__action__ToolContact_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__action__ToolContact_GetResult_Response__destroy(ur_msgs__action__ToolContact_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__action__ToolContact_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__action__ToolContact_GetResult_Response__Sequence__init(ur_msgs__action__ToolContact_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_GetResult_Response * data = NULL;

  if (size) {
    data = (ur_msgs__action__ToolContact_GetResult_Response *)allocator.zero_allocate(size, sizeof(ur_msgs__action__ToolContact_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__action__ToolContact_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__action__ToolContact_GetResult_Response__fini(&data[i - 1]);
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
ur_msgs__action__ToolContact_GetResult_Response__Sequence__fini(ur_msgs__action__ToolContact_GetResult_Response__Sequence * array)
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
      ur_msgs__action__ToolContact_GetResult_Response__fini(&array->data[i]);
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

ur_msgs__action__ToolContact_GetResult_Response__Sequence *
ur_msgs__action__ToolContact_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_GetResult_Response__Sequence * array = (ur_msgs__action__ToolContact_GetResult_Response__Sequence *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__action__ToolContact_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__action__ToolContact_GetResult_Response__Sequence__destroy(ur_msgs__action__ToolContact_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__action__ToolContact_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__action__ToolContact_GetResult_Response__Sequence__are_equal(const ur_msgs__action__ToolContact_GetResult_Response__Sequence * lhs, const ur_msgs__action__ToolContact_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__action__ToolContact_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__action__ToolContact_GetResult_Response__Sequence__copy(
  const ur_msgs__action__ToolContact_GetResult_Response__Sequence * input,
  ur_msgs__action__ToolContact_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__action__ToolContact_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__action__ToolContact_GetResult_Response * data =
      (ur_msgs__action__ToolContact_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__action__ToolContact_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__action__ToolContact_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__action__ToolContact_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "ur_msgs/action/detail/tool_contact__functions.h"

bool
ur_msgs__action__ToolContact_FeedbackMessage__init(ur_msgs__action__ToolContact_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ur_msgs__action__ToolContact_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!ur_msgs__action__ToolContact_Feedback__init(&msg->feedback)) {
    ur_msgs__action__ToolContact_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
ur_msgs__action__ToolContact_FeedbackMessage__fini(ur_msgs__action__ToolContact_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  ur_msgs__action__ToolContact_Feedback__fini(&msg->feedback);
}

bool
ur_msgs__action__ToolContact_FeedbackMessage__are_equal(const ur_msgs__action__ToolContact_FeedbackMessage * lhs, const ur_msgs__action__ToolContact_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!ur_msgs__action__ToolContact_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
ur_msgs__action__ToolContact_FeedbackMessage__copy(
  const ur_msgs__action__ToolContact_FeedbackMessage * input,
  ur_msgs__action__ToolContact_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!ur_msgs__action__ToolContact_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

ur_msgs__action__ToolContact_FeedbackMessage *
ur_msgs__action__ToolContact_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_FeedbackMessage * msg = (ur_msgs__action__ToolContact_FeedbackMessage *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__action__ToolContact_FeedbackMessage));
  bool success = ur_msgs__action__ToolContact_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__action__ToolContact_FeedbackMessage__destroy(ur_msgs__action__ToolContact_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__action__ToolContact_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__action__ToolContact_FeedbackMessage__Sequence__init(ur_msgs__action__ToolContact_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_FeedbackMessage * data = NULL;

  if (size) {
    data = (ur_msgs__action__ToolContact_FeedbackMessage *)allocator.zero_allocate(size, sizeof(ur_msgs__action__ToolContact_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__action__ToolContact_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__action__ToolContact_FeedbackMessage__fini(&data[i - 1]);
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
ur_msgs__action__ToolContact_FeedbackMessage__Sequence__fini(ur_msgs__action__ToolContact_FeedbackMessage__Sequence * array)
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
      ur_msgs__action__ToolContact_FeedbackMessage__fini(&array->data[i]);
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

ur_msgs__action__ToolContact_FeedbackMessage__Sequence *
ur_msgs__action__ToolContact_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__action__ToolContact_FeedbackMessage__Sequence * array = (ur_msgs__action__ToolContact_FeedbackMessage__Sequence *)allocator.allocate(sizeof(ur_msgs__action__ToolContact_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__action__ToolContact_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__action__ToolContact_FeedbackMessage__Sequence__destroy(ur_msgs__action__ToolContact_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__action__ToolContact_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__action__ToolContact_FeedbackMessage__Sequence__are_equal(const ur_msgs__action__ToolContact_FeedbackMessage__Sequence * lhs, const ur_msgs__action__ToolContact_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__action__ToolContact_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__action__ToolContact_FeedbackMessage__Sequence__copy(
  const ur_msgs__action__ToolContact_FeedbackMessage__Sequence * input,
  ur_msgs__action__ToolContact_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__action__ToolContact_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__action__ToolContact_FeedbackMessage * data =
      (ur_msgs__action__ToolContact_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__action__ToolContact_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__action__ToolContact_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__action__ToolContact_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
