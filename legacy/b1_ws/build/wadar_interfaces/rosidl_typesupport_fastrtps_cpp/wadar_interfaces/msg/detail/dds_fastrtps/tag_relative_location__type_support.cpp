// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice
#include "wadar_interfaces/msg/detail/tag_relative_location__rosidl_typesupport_fastrtps_cpp.hpp"
#include "wadar_interfaces/msg/detail/tag_relative_location__functions.h"
#include "wadar_interfaces/msg/detail/tag_relative_location__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace wadar_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wadar_interfaces
cdr_serialize(
  const wadar_interfaces::msg::TagRelativeLocation & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: relative_heading
  cdr << ros_message.relative_heading;
  // Member: alignment
  cdr << ros_message.alignment;
  // Member: distance
  cdr << ros_message.distance;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wadar_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  wadar_interfaces::msg::TagRelativeLocation & ros_message)
{
  // Member: relative_heading
  cdr >> ros_message.relative_heading;

  // Member: alignment
  cdr >> ros_message.alignment;

  // Member: distance
  cdr >> ros_message.distance;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wadar_interfaces
get_serialized_size(
  const wadar_interfaces::msg::TagRelativeLocation & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: relative_heading
  {
    size_t item_size = sizeof(ros_message.relative_heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: alignment
  {
    size_t item_size = sizeof(ros_message.alignment);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance
  {
    size_t item_size = sizeof(ros_message.distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wadar_interfaces
max_serialized_size_TagRelativeLocation(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: relative_heading
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: alignment
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: distance
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = wadar_interfaces::msg::TagRelativeLocation;
    is_plain =
      (
      offsetof(DataType, distance) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _TagRelativeLocation__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const wadar_interfaces::msg::TagRelativeLocation *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TagRelativeLocation__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<wadar_interfaces::msg::TagRelativeLocation *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TagRelativeLocation__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const wadar_interfaces::msg::TagRelativeLocation *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TagRelativeLocation__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TagRelativeLocation(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TagRelativeLocation__callbacks = {
  "wadar_interfaces::msg",
  "TagRelativeLocation",
  _TagRelativeLocation__cdr_serialize,
  _TagRelativeLocation__cdr_deserialize,
  _TagRelativeLocation__get_serialized_size,
  _TagRelativeLocation__max_serialized_size
};

static rosidl_message_type_support_t _TagRelativeLocation__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TagRelativeLocation__callbacks,
  get_message_typesupport_handle_function,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_hash,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_description,
  &wadar_interfaces__msg__TagRelativeLocation__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace wadar_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_wadar_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<wadar_interfaces::msg::TagRelativeLocation>()
{
  return &wadar_interfaces::msg::typesupport_fastrtps_cpp::_TagRelativeLocation__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wadar_interfaces, msg, TagRelativeLocation)() {
  return &wadar_interfaces::msg::typesupport_fastrtps_cpp::_TagRelativeLocation__handle;
}

#ifdef __cplusplus
}
#endif
