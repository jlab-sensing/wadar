// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "wadar_interfaces/msg/detail/tag_relative_location__functions.h"
#include "wadar_interfaces/msg/detail/tag_relative_location__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wadar_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TagRelativeLocation_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wadar_interfaces::msg::TagRelativeLocation(_init);
}

void TagRelativeLocation_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wadar_interfaces::msg::TagRelativeLocation *>(message_memory);
  typed_message->~TagRelativeLocation();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TagRelativeLocation_message_member_array[3] = {
  {
    "relative_heading",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wadar_interfaces::msg::TagRelativeLocation, relative_heading),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "alignment",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wadar_interfaces::msg::TagRelativeLocation, alignment),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "distance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wadar_interfaces::msg::TagRelativeLocation, distance),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TagRelativeLocation_message_members = {
  "wadar_interfaces::msg",  // message namespace
  "TagRelativeLocation",  // message name
  3,  // number of fields
  sizeof(wadar_interfaces::msg::TagRelativeLocation),
  TagRelativeLocation_message_member_array,  // message members
  TagRelativeLocation_init_function,  // function to initialize message memory (memory has to be allocated)
  TagRelativeLocation_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TagRelativeLocation_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TagRelativeLocation_message_members,
  get_message_typesupport_handle_function,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_hash,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_description,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace wadar_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wadar_interfaces::msg::TagRelativeLocation>()
{
  return &::wadar_interfaces::msg::rosidl_typesupport_introspection_cpp::TagRelativeLocation_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wadar_interfaces, msg, TagRelativeLocation)() {
  return &::wadar_interfaces::msg::rosidl_typesupport_introspection_cpp::TagRelativeLocation_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
