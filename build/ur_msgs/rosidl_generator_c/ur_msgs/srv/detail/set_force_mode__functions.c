// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ur_msgs:srv/SetForceMode.idl
// generated code does not contain a copyright notice
#include "ur_msgs/srv/detail/set_force_mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `task_frame`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `wrench`
#include "geometry_msgs/msg/detail/wrench__functions.h"
// Member `speed_limits`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
ur_msgs__srv__SetForceMode_Request__init(ur_msgs__srv__SetForceMode_Request * msg)
{
  if (!msg) {
    return false;
  }
  // task_frame
  if (!geometry_msgs__msg__PoseStamped__init(&msg->task_frame)) {
    ur_msgs__srv__SetForceMode_Request__fini(msg);
    return false;
  }
  // selection_vector_x
  msg->selection_vector_x = false;
  // selection_vector_y
  msg->selection_vector_y = false;
  // selection_vector_z
  msg->selection_vector_z = false;
  // selection_vector_rx
  msg->selection_vector_rx = false;
  // selection_vector_ry
  msg->selection_vector_ry = false;
  // selection_vector_rz
  msg->selection_vector_rz = false;
  // wrench
  if (!geometry_msgs__msg__Wrench__init(&msg->wrench)) {
    ur_msgs__srv__SetForceMode_Request__fini(msg);
    return false;
  }
  // type
  msg->type = 2;
  // speed_limits
  if (!geometry_msgs__msg__Twist__init(&msg->speed_limits)) {
    ur_msgs__srv__SetForceMode_Request__fini(msg);
    return false;
  }
  // deviation_limits
  msg->deviation_limits[0] = 0.01f;
  msg->deviation_limits[1] = 0.01f;
  msg->deviation_limits[2] = 0.01f;
  msg->deviation_limits[3] = 0.01f;
  msg->deviation_limits[4] = 0.01f;
  msg->deviation_limits[5] = 0.01f;
  // damping_factor
  msg->damping_factor = 0.025f;
  // gain_scaling
  msg->gain_scaling = 0.5f;
  return true;
}

void
ur_msgs__srv__SetForceMode_Request__fini(ur_msgs__srv__SetForceMode_Request * msg)
{
  if (!msg) {
    return;
  }
  // task_frame
  geometry_msgs__msg__PoseStamped__fini(&msg->task_frame);
  // selection_vector_x
  // selection_vector_y
  // selection_vector_z
  // selection_vector_rx
  // selection_vector_ry
  // selection_vector_rz
  // wrench
  geometry_msgs__msg__Wrench__fini(&msg->wrench);
  // type
  // speed_limits
  geometry_msgs__msg__Twist__fini(&msg->speed_limits);
  // deviation_limits
  // damping_factor
  // gain_scaling
}

bool
ur_msgs__srv__SetForceMode_Request__are_equal(const ur_msgs__srv__SetForceMode_Request * lhs, const ur_msgs__srv__SetForceMode_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // task_frame
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->task_frame), &(rhs->task_frame)))
  {
    return false;
  }
  // selection_vector_x
  if (lhs->selection_vector_x != rhs->selection_vector_x) {
    return false;
  }
  // selection_vector_y
  if (lhs->selection_vector_y != rhs->selection_vector_y) {
    return false;
  }
  // selection_vector_z
  if (lhs->selection_vector_z != rhs->selection_vector_z) {
    return false;
  }
  // selection_vector_rx
  if (lhs->selection_vector_rx != rhs->selection_vector_rx) {
    return false;
  }
  // selection_vector_ry
  if (lhs->selection_vector_ry != rhs->selection_vector_ry) {
    return false;
  }
  // selection_vector_rz
  if (lhs->selection_vector_rz != rhs->selection_vector_rz) {
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->wrench), &(rhs->wrench)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // speed_limits
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->speed_limits), &(rhs->speed_limits)))
  {
    return false;
  }
  // deviation_limits
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->deviation_limits[i] != rhs->deviation_limits[i]) {
      return false;
    }
  }
  // damping_factor
  if (lhs->damping_factor != rhs->damping_factor) {
    return false;
  }
  // gain_scaling
  if (lhs->gain_scaling != rhs->gain_scaling) {
    return false;
  }
  return true;
}

bool
ur_msgs__srv__SetForceMode_Request__copy(
  const ur_msgs__srv__SetForceMode_Request * input,
  ur_msgs__srv__SetForceMode_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // task_frame
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->task_frame), &(output->task_frame)))
  {
    return false;
  }
  // selection_vector_x
  output->selection_vector_x = input->selection_vector_x;
  // selection_vector_y
  output->selection_vector_y = input->selection_vector_y;
  // selection_vector_z
  output->selection_vector_z = input->selection_vector_z;
  // selection_vector_rx
  output->selection_vector_rx = input->selection_vector_rx;
  // selection_vector_ry
  output->selection_vector_ry = input->selection_vector_ry;
  // selection_vector_rz
  output->selection_vector_rz = input->selection_vector_rz;
  // wrench
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->wrench), &(output->wrench)))
  {
    return false;
  }
  // type
  output->type = input->type;
  // speed_limits
  if (!geometry_msgs__msg__Twist__copy(
      &(input->speed_limits), &(output->speed_limits)))
  {
    return false;
  }
  // deviation_limits
  for (size_t i = 0; i < 6; ++i) {
    output->deviation_limits[i] = input->deviation_limits[i];
  }
  // damping_factor
  output->damping_factor = input->damping_factor;
  // gain_scaling
  output->gain_scaling = input->gain_scaling;
  return true;
}

ur_msgs__srv__SetForceMode_Request *
ur_msgs__srv__SetForceMode_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__srv__SetForceMode_Request * msg = (ur_msgs__srv__SetForceMode_Request *)allocator.allocate(sizeof(ur_msgs__srv__SetForceMode_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__srv__SetForceMode_Request));
  bool success = ur_msgs__srv__SetForceMode_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__srv__SetForceMode_Request__destroy(ur_msgs__srv__SetForceMode_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__srv__SetForceMode_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__srv__SetForceMode_Request__Sequence__init(ur_msgs__srv__SetForceMode_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__srv__SetForceMode_Request * data = NULL;

  if (size) {
    data = (ur_msgs__srv__SetForceMode_Request *)allocator.zero_allocate(size, sizeof(ur_msgs__srv__SetForceMode_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__srv__SetForceMode_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__srv__SetForceMode_Request__fini(&data[i - 1]);
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
ur_msgs__srv__SetForceMode_Request__Sequence__fini(ur_msgs__srv__SetForceMode_Request__Sequence * array)
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
      ur_msgs__srv__SetForceMode_Request__fini(&array->data[i]);
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

ur_msgs__srv__SetForceMode_Request__Sequence *
ur_msgs__srv__SetForceMode_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__srv__SetForceMode_Request__Sequence * array = (ur_msgs__srv__SetForceMode_Request__Sequence *)allocator.allocate(sizeof(ur_msgs__srv__SetForceMode_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__srv__SetForceMode_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__srv__SetForceMode_Request__Sequence__destroy(ur_msgs__srv__SetForceMode_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__srv__SetForceMode_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__srv__SetForceMode_Request__Sequence__are_equal(const ur_msgs__srv__SetForceMode_Request__Sequence * lhs, const ur_msgs__srv__SetForceMode_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__srv__SetForceMode_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__srv__SetForceMode_Request__Sequence__copy(
  const ur_msgs__srv__SetForceMode_Request__Sequence * input,
  ur_msgs__srv__SetForceMode_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__srv__SetForceMode_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__srv__SetForceMode_Request * data =
      (ur_msgs__srv__SetForceMode_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__srv__SetForceMode_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__srv__SetForceMode_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__srv__SetForceMode_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ur_msgs__srv__SetForceMode_Response__init(ur_msgs__srv__SetForceMode_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
ur_msgs__srv__SetForceMode_Response__fini(ur_msgs__srv__SetForceMode_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
ur_msgs__srv__SetForceMode_Response__are_equal(const ur_msgs__srv__SetForceMode_Response * lhs, const ur_msgs__srv__SetForceMode_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
ur_msgs__srv__SetForceMode_Response__copy(
  const ur_msgs__srv__SetForceMode_Response * input,
  ur_msgs__srv__SetForceMode_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

ur_msgs__srv__SetForceMode_Response *
ur_msgs__srv__SetForceMode_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__srv__SetForceMode_Response * msg = (ur_msgs__srv__SetForceMode_Response *)allocator.allocate(sizeof(ur_msgs__srv__SetForceMode_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ur_msgs__srv__SetForceMode_Response));
  bool success = ur_msgs__srv__SetForceMode_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ur_msgs__srv__SetForceMode_Response__destroy(ur_msgs__srv__SetForceMode_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ur_msgs__srv__SetForceMode_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ur_msgs__srv__SetForceMode_Response__Sequence__init(ur_msgs__srv__SetForceMode_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__srv__SetForceMode_Response * data = NULL;

  if (size) {
    data = (ur_msgs__srv__SetForceMode_Response *)allocator.zero_allocate(size, sizeof(ur_msgs__srv__SetForceMode_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ur_msgs__srv__SetForceMode_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ur_msgs__srv__SetForceMode_Response__fini(&data[i - 1]);
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
ur_msgs__srv__SetForceMode_Response__Sequence__fini(ur_msgs__srv__SetForceMode_Response__Sequence * array)
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
      ur_msgs__srv__SetForceMode_Response__fini(&array->data[i]);
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

ur_msgs__srv__SetForceMode_Response__Sequence *
ur_msgs__srv__SetForceMode_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ur_msgs__srv__SetForceMode_Response__Sequence * array = (ur_msgs__srv__SetForceMode_Response__Sequence *)allocator.allocate(sizeof(ur_msgs__srv__SetForceMode_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ur_msgs__srv__SetForceMode_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ur_msgs__srv__SetForceMode_Response__Sequence__destroy(ur_msgs__srv__SetForceMode_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ur_msgs__srv__SetForceMode_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ur_msgs__srv__SetForceMode_Response__Sequence__are_equal(const ur_msgs__srv__SetForceMode_Response__Sequence * lhs, const ur_msgs__srv__SetForceMode_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ur_msgs__srv__SetForceMode_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ur_msgs__srv__SetForceMode_Response__Sequence__copy(
  const ur_msgs__srv__SetForceMode_Response__Sequence * input,
  ur_msgs__srv__SetForceMode_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ur_msgs__srv__SetForceMode_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ur_msgs__srv__SetForceMode_Response * data =
      (ur_msgs__srv__SetForceMode_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ur_msgs__srv__SetForceMode_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ur_msgs__srv__SetForceMode_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ur_msgs__srv__SetForceMode_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
