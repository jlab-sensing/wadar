// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice
#include "wadar_interfaces/msg/detail/tag_relative_location__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
wadar_interfaces__msg__TagRelativeLocation__init(wadar_interfaces__msg__TagRelativeLocation * msg)
{
  if (!msg) {
    return false;
  }
  // relative_heading
  // alignment
  // distance
  return true;
}

void
wadar_interfaces__msg__TagRelativeLocation__fini(wadar_interfaces__msg__TagRelativeLocation * msg)
{
  if (!msg) {
    return;
  }
  // relative_heading
  // alignment
  // distance
}

bool
wadar_interfaces__msg__TagRelativeLocation__are_equal(const wadar_interfaces__msg__TagRelativeLocation * lhs, const wadar_interfaces__msg__TagRelativeLocation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // relative_heading
  if (lhs->relative_heading != rhs->relative_heading) {
    return false;
  }
  // alignment
  if (lhs->alignment != rhs->alignment) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
wadar_interfaces__msg__TagRelativeLocation__copy(
  const wadar_interfaces__msg__TagRelativeLocation * input,
  wadar_interfaces__msg__TagRelativeLocation * output)
{
  if (!input || !output) {
    return false;
  }
  // relative_heading
  output->relative_heading = input->relative_heading;
  // alignment
  output->alignment = input->alignment;
  // distance
  output->distance = input->distance;
  return true;
}

wadar_interfaces__msg__TagRelativeLocation *
wadar_interfaces__msg__TagRelativeLocation__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wadar_interfaces__msg__TagRelativeLocation * msg = (wadar_interfaces__msg__TagRelativeLocation *)allocator.allocate(sizeof(wadar_interfaces__msg__TagRelativeLocation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wadar_interfaces__msg__TagRelativeLocation));
  bool success = wadar_interfaces__msg__TagRelativeLocation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wadar_interfaces__msg__TagRelativeLocation__destroy(wadar_interfaces__msg__TagRelativeLocation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wadar_interfaces__msg__TagRelativeLocation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wadar_interfaces__msg__TagRelativeLocation__Sequence__init(wadar_interfaces__msg__TagRelativeLocation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wadar_interfaces__msg__TagRelativeLocation * data = NULL;

  if (size) {
    data = (wadar_interfaces__msg__TagRelativeLocation *)allocator.zero_allocate(size, sizeof(wadar_interfaces__msg__TagRelativeLocation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wadar_interfaces__msg__TagRelativeLocation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wadar_interfaces__msg__TagRelativeLocation__fini(&data[i - 1]);
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
wadar_interfaces__msg__TagRelativeLocation__Sequence__fini(wadar_interfaces__msg__TagRelativeLocation__Sequence * array)
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
      wadar_interfaces__msg__TagRelativeLocation__fini(&array->data[i]);
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

wadar_interfaces__msg__TagRelativeLocation__Sequence *
wadar_interfaces__msg__TagRelativeLocation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wadar_interfaces__msg__TagRelativeLocation__Sequence * array = (wadar_interfaces__msg__TagRelativeLocation__Sequence *)allocator.allocate(sizeof(wadar_interfaces__msg__TagRelativeLocation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wadar_interfaces__msg__TagRelativeLocation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wadar_interfaces__msg__TagRelativeLocation__Sequence__destroy(wadar_interfaces__msg__TagRelativeLocation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wadar_interfaces__msg__TagRelativeLocation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wadar_interfaces__msg__TagRelativeLocation__Sequence__are_equal(const wadar_interfaces__msg__TagRelativeLocation__Sequence * lhs, const wadar_interfaces__msg__TagRelativeLocation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wadar_interfaces__msg__TagRelativeLocation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wadar_interfaces__msg__TagRelativeLocation__Sequence__copy(
  const wadar_interfaces__msg__TagRelativeLocation__Sequence * input,
  wadar_interfaces__msg__TagRelativeLocation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wadar_interfaces__msg__TagRelativeLocation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wadar_interfaces__msg__TagRelativeLocation * data =
      (wadar_interfaces__msg__TagRelativeLocation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wadar_interfaces__msg__TagRelativeLocation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wadar_interfaces__msg__TagRelativeLocation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wadar_interfaces__msg__TagRelativeLocation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
