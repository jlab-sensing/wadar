// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice

#ifndef WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__STRUCT_H_
#define WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/TagRelativeLocation in the package wadar_interfaces.
typedef struct wadar_interfaces__msg__TagRelativeLocation
{
  float relative_heading;
  float alignment;
  float distance;
} wadar_interfaces__msg__TagRelativeLocation;

// Struct for a sequence of wadar_interfaces__msg__TagRelativeLocation.
typedef struct wadar_interfaces__msg__TagRelativeLocation__Sequence
{
  wadar_interfaces__msg__TagRelativeLocation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wadar_interfaces__msg__TagRelativeLocation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__STRUCT_H_
