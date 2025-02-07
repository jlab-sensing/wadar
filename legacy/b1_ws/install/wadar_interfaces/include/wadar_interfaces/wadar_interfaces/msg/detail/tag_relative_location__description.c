// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice

#include "wadar_interfaces/msg/detail/tag_relative_location__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_wadar_interfaces
const rosidl_type_hash_t *
wadar_interfaces__msg__TagRelativeLocation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6b, 0x04, 0x1a, 0xfd, 0xe4, 0x49, 0x15, 0x0e,
      0x6d, 0xd6, 0xcd, 0xc6, 0x76, 0xe0, 0xbe, 0x69,
      0x0b, 0xcd, 0xa4, 0x82, 0xf8, 0x99, 0x45, 0xfc,
      0xde, 0xfb, 0x9c, 0x75, 0xe0, 0xb7, 0x48, 0x36,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char wadar_interfaces__msg__TagRelativeLocation__TYPE_NAME[] = "wadar_interfaces/msg/TagRelativeLocation";

// Define type names, field names, and default values
static char wadar_interfaces__msg__TagRelativeLocation__FIELD_NAME__relative_heading[] = "relative_heading";
static char wadar_interfaces__msg__TagRelativeLocation__FIELD_NAME__alignment[] = "alignment";
static char wadar_interfaces__msg__TagRelativeLocation__FIELD_NAME__distance[] = "distance";

static rosidl_runtime_c__type_description__Field wadar_interfaces__msg__TagRelativeLocation__FIELDS[] = {
  {
    {wadar_interfaces__msg__TagRelativeLocation__FIELD_NAME__relative_heading, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {wadar_interfaces__msg__TagRelativeLocation__FIELD_NAME__alignment, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {wadar_interfaces__msg__TagRelativeLocation__FIELD_NAME__distance, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
wadar_interfaces__msg__TagRelativeLocation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {wadar_interfaces__msg__TagRelativeLocation__TYPE_NAME, 40, 40},
      {wadar_interfaces__msg__TagRelativeLocation__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 relative_heading\n"
  "float32 alignment\n"
  "float32 distance";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
wadar_interfaces__msg__TagRelativeLocation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {wadar_interfaces__msg__TagRelativeLocation__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 59, 59},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
wadar_interfaces__msg__TagRelativeLocation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *wadar_interfaces__msg__TagRelativeLocation__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
