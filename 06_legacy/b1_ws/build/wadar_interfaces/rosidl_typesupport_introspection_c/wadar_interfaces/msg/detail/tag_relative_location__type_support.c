// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wadar_interfaces/msg/detail/tag_relative_location__rosidl_typesupport_introspection_c.h"
#include "wadar_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wadar_interfaces/msg/detail/tag_relative_location__functions.h"
#include "wadar_interfaces/msg/detail/tag_relative_location__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wadar_interfaces__msg__TagRelativeLocation__init(message_memory);
}

void wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_fini_function(void * message_memory)
{
  wadar_interfaces__msg__TagRelativeLocation__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_message_member_array[3] = {
  {
    "relative_heading",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wadar_interfaces__msg__TagRelativeLocation, relative_heading),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alignment",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wadar_interfaces__msg__TagRelativeLocation, alignment),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wadar_interfaces__msg__TagRelativeLocation, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_message_members = {
  "wadar_interfaces__msg",  // message namespace
  "TagRelativeLocation",  // message name
  3,  // number of fields
  sizeof(wadar_interfaces__msg__TagRelativeLocation),
  wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_message_member_array,  // message members
  wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_init_function,  // function to initialize message memory (memory has to be allocated)
  wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_message_type_support_handle = {
  0,
  &wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_message_members,
  get_message_typesupport_handle_function,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_hash,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_description,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wadar_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wadar_interfaces, msg, TagRelativeLocation)() {
  if (!wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_message_type_support_handle.typesupport_identifier) {
    wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wadar_interfaces__msg__TagRelativeLocation__rosidl_typesupport_introspection_c__TagRelativeLocation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
