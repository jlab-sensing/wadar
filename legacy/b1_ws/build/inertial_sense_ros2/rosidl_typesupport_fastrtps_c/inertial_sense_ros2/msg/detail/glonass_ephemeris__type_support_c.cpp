// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__struct.h"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__functions.h"
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

#include "inertial_sense_ros2/msg/detail/g_time__functions.h"  // toe, tof

// forward declare type support functions
size_t get_serialized_size_inertial_sense_ros2__msg__GTime(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_inertial_sense_ros2__msg__GTime(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime)();


using _GlonassEphemeris__ros_msg_type = inertial_sense_ros2__msg__GlonassEphemeris;

static bool _GlonassEphemeris__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GlonassEphemeris__ros_msg_type * ros_message = static_cast<const _GlonassEphemeris__ros_msg_type *>(untyped_ros_message);
  // Field name: sat
  {
    cdr << ros_message->sat;
  }

  // Field name: iode
  {
    cdr << ros_message->iode;
  }

  // Field name: frq
  {
    cdr << ros_message->frq;
  }

  // Field name: svh
  {
    cdr << ros_message->svh;
  }

  // Field name: sva
  {
    cdr << ros_message->sva;
  }

  // Field name: age
  {
    cdr << ros_message->age;
  }

  // Field name: toe
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->toe, cdr))
    {
      return false;
    }
  }

  // Field name: tof
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->tof, cdr))
    {
      return false;
    }
  }

  // Field name: pos
  {
    size_t size = 3;
    auto array_ptr = ros_message->pos;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: vel
  {
    size_t size = 3;
    auto array_ptr = ros_message->vel;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: acc
  {
    size_t size = 3;
    auto array_ptr = ros_message->acc;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: taun
  {
    cdr << ros_message->taun;
  }

  // Field name: gamn
  {
    cdr << ros_message->gamn;
  }

  // Field name: dtaun
  {
    cdr << ros_message->dtaun;
  }

  return true;
}

static bool _GlonassEphemeris__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GlonassEphemeris__ros_msg_type * ros_message = static_cast<_GlonassEphemeris__ros_msg_type *>(untyped_ros_message);
  // Field name: sat
  {
    cdr >> ros_message->sat;
  }

  // Field name: iode
  {
    cdr >> ros_message->iode;
  }

  // Field name: frq
  {
    cdr >> ros_message->frq;
  }

  // Field name: svh
  {
    cdr >> ros_message->svh;
  }

  // Field name: sva
  {
    cdr >> ros_message->sva;
  }

  // Field name: age
  {
    cdr >> ros_message->age;
  }

  // Field name: toe
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->toe))
    {
      return false;
    }
  }

  // Field name: tof
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->tof))
    {
      return false;
    }
  }

  // Field name: pos
  {
    size_t size = 3;
    auto array_ptr = ros_message->pos;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: vel
  {
    size_t size = 3;
    auto array_ptr = ros_message->vel;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: acc
  {
    size_t size = 3;
    auto array_ptr = ros_message->acc;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: taun
  {
    cdr >> ros_message->taun;
  }

  // Field name: gamn
  {
    cdr >> ros_message->gamn;
  }

  // Field name: dtaun
  {
    cdr >> ros_message->dtaun;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t get_serialized_size_inertial_sense_ros2__msg__GlonassEphemeris(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GlonassEphemeris__ros_msg_type * ros_message = static_cast<const _GlonassEphemeris__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name sat
  {
    size_t item_size = sizeof(ros_message->sat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name iode
  {
    size_t item_size = sizeof(ros_message->iode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name frq
  {
    size_t item_size = sizeof(ros_message->frq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name svh
  {
    size_t item_size = sizeof(ros_message->svh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sva
  {
    size_t item_size = sizeof(ros_message->sva);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name age
  {
    size_t item_size = sizeof(ros_message->age);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name toe

  current_alignment += get_serialized_size_inertial_sense_ros2__msg__GTime(
    &(ros_message->toe), current_alignment);
  // field.name tof

  current_alignment += get_serialized_size_inertial_sense_ros2__msg__GTime(
    &(ros_message->tof), current_alignment);
  // field.name pos
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->pos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->vel;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acc
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->acc;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name taun
  {
    size_t item_size = sizeof(ros_message->taun);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gamn
  {
    size_t item_size = sizeof(ros_message->gamn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dtaun
  {
    size_t item_size = sizeof(ros_message->dtaun);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GlonassEphemeris__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_inertial_sense_ros2__msg__GlonassEphemeris(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t max_serialized_size_inertial_sense_ros2__msg__GlonassEphemeris(
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

  // member: sat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: iode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: frq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: svh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: sva
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: age
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: toe
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
  // member: tof
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
  // member: pos
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vel
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: acc
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: taun
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: gamn
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: dtaun
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = inertial_sense_ros2__msg__GlonassEphemeris;
    is_plain =
      (
      offsetof(DataType, dtaun) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GlonassEphemeris__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_inertial_sense_ros2__msg__GlonassEphemeris(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GlonassEphemeris = {
  "inertial_sense_ros2::msg",
  "GlonassEphemeris",
  _GlonassEphemeris__cdr_serialize,
  _GlonassEphemeris__cdr_deserialize,
  _GlonassEphemeris__get_serialized_size,
  _GlonassEphemeris__max_serialized_size
};

static rosidl_message_type_support_t _GlonassEphemeris__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GlonassEphemeris,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_hash,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_description,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GlonassEphemeris)() {
  return &_GlonassEphemeris__type_support;
}

#if defined(__cplusplus)
}
#endif
