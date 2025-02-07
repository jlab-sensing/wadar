// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from inertial_sense_ros2:msg/GNSSObsVec.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__struct.h"
#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "inertial_sense_ros2/msg/detail/g_time__functions.h"  // time
#include "inertial_sense_ros2/msg/detail/gnss_observation__functions.h"  // obs
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
size_t get_serialized_size_inertial_sense_ros2__msg__GNSSObservation(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_inertial_sense_ros2__msg__GNSSObservation(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GNSSObservation)();
size_t get_serialized_size_inertial_sense_ros2__msg__GTime(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_inertial_sense_ros2__msg__GTime(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_inertial_sense_ros2
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_inertial_sense_ros2
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _GNSSObsVec__ros_msg_type = inertial_sense_ros2__msg__GNSSObsVec;

static bool _GNSSObsVec__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GNSSObsVec__ros_msg_type * ros_message = static_cast<const _GNSSObsVec__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: time
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->time, cdr))
    {
      return false;
    }
  }

  // Field name: obs
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GNSSObservation
      )()->data);
    size_t size = ros_message->obs.size;
    auto array_ptr = ros_message->obs.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _GNSSObsVec__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GNSSObsVec__ros_msg_type * ros_message = static_cast<_GNSSObsVec__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: time
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->time))
    {
      return false;
    }
  }

  // Field name: obs
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GNSSObservation
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->obs.data) {
      inertial_sense_ros2__msg__GNSSObservation__Sequence__fini(&ros_message->obs);
    }
    if (!inertial_sense_ros2__msg__GNSSObservation__Sequence__init(&ros_message->obs, size)) {
      fprintf(stderr, "failed to create array for field 'obs'");
      return false;
    }
    auto array_ptr = ros_message->obs.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t get_serialized_size_inertial_sense_ros2__msg__GNSSObsVec(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GNSSObsVec__ros_msg_type * ros_message = static_cast<const _GNSSObsVec__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name time

  current_alignment += get_serialized_size_inertial_sense_ros2__msg__GTime(
    &(ros_message->time), current_alignment);
  // field.name obs
  {
    size_t array_size = ros_message->obs.size;
    auto array_ptr = ros_message->obs.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_inertial_sense_ros2__msg__GNSSObservation(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GNSSObsVec__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_inertial_sense_ros2__msg__GNSSObsVec(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t max_serialized_size_inertial_sense_ros2__msg__GNSSObsVec(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: time
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_inertial_sense_ros2__msg__GTime(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: obs
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_inertial_sense_ros2__msg__GNSSObservation(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = inertial_sense_ros2__msg__GNSSObsVec;
    is_plain =
      (
      offsetof(DataType, obs) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GNSSObsVec__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_inertial_sense_ros2__msg__GNSSObsVec(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GNSSObsVec = {
  "inertial_sense_ros2::msg",
  "GNSSObsVec",
  _GNSSObsVec__cdr_serialize,
  _GNSSObsVec__cdr_deserialize,
  _GNSSObsVec__get_serialized_size,
  _GNSSObsVec__max_serialized_size
};

static rosidl_message_type_support_t _GNSSObsVec__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GNSSObsVec,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GNSSObsVec__get_type_hash,
  &inertial_sense_ros2__msg__GNSSObsVec__get_type_description,
  &inertial_sense_ros2__msg__GNSSObsVec__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GNSSObsVec)() {
  return &_GNSSObsVec__type_support;
}

#if defined(__cplusplus)
}
#endif
