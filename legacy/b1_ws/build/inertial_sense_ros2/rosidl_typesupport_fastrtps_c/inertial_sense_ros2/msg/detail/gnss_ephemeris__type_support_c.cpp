// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "inertial_sense_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__struct.h"
#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__functions.h"
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

#include "inertial_sense_ros2/msg/detail/g_time__functions.h"  // toc, toe, ttr
#include "std_msgs/msg/detail/header__functions.h"  // header

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


using _GNSSEphemeris__ros_msg_type = inertial_sense_ros2__msg__GNSSEphemeris;

static bool _GNSSEphemeris__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GNSSEphemeris__ros_msg_type * ros_message = static_cast<const _GNSSEphemeris__ros_msg_type *>(untyped_ros_message);
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

  // Field name: sat
  {
    cdr << ros_message->sat;
  }

  // Field name: iode
  {
    cdr << ros_message->iode;
  }

  // Field name: iodc
  {
    cdr << ros_message->iodc;
  }

  // Field name: sva
  {
    cdr << ros_message->sva;
  }

  // Field name: svh
  {
    cdr << ros_message->svh;
  }

  // Field name: week
  {
    cdr << ros_message->week;
  }

  // Field name: code
  {
    cdr << ros_message->code;
  }

  // Field name: flag
  {
    cdr << ros_message->flag;
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

  // Field name: toc
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->toc, cdr))
    {
      return false;
    }
  }

  // Field name: ttr
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->ttr, cdr))
    {
      return false;
    }
  }

  // Field name: a
  {
    cdr << ros_message->a;
  }

  // Field name: e
  {
    cdr << ros_message->e;
  }

  // Field name: i0
  {
    cdr << ros_message->i0;
  }

  // Field name: omg_0
  {
    cdr << ros_message->omg_0;
  }

  // Field name: omg
  {
    cdr << ros_message->omg;
  }

  // Field name: m_0
  {
    cdr << ros_message->m_0;
  }

  // Field name: deln
  {
    cdr << ros_message->deln;
  }

  // Field name: omg_d
  {
    cdr << ros_message->omg_d;
  }

  // Field name: idot
  {
    cdr << ros_message->idot;
  }

  // Field name: crc
  {
    cdr << ros_message->crc;
  }

  // Field name: crs
  {
    cdr << ros_message->crs;
  }

  // Field name: cuc
  {
    cdr << ros_message->cuc;
  }

  // Field name: cus
  {
    cdr << ros_message->cus;
  }

  // Field name: cic
  {
    cdr << ros_message->cic;
  }

  // Field name: cis
  {
    cdr << ros_message->cis;
  }

  // Field name: toes
  {
    cdr << ros_message->toes;
  }

  // Field name: fit
  {
    cdr << ros_message->fit;
  }

  // Field name: f0
  {
    cdr << ros_message->f0;
  }

  // Field name: f1
  {
    cdr << ros_message->f1;
  }

  // Field name: f2
  {
    cdr << ros_message->f2;
  }

  // Field name: tgd
  {
    size_t size = 4;
    auto array_ptr = ros_message->tgd;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: a_dot
  {
    cdr << ros_message->a_dot;
  }

  // Field name: ndot
  {
    cdr << ros_message->ndot;
  }

  return true;
}

static bool _GNSSEphemeris__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GNSSEphemeris__ros_msg_type * ros_message = static_cast<_GNSSEphemeris__ros_msg_type *>(untyped_ros_message);
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

  // Field name: sat
  {
    cdr >> ros_message->sat;
  }

  // Field name: iode
  {
    cdr >> ros_message->iode;
  }

  // Field name: iodc
  {
    cdr >> ros_message->iodc;
  }

  // Field name: sva
  {
    cdr >> ros_message->sva;
  }

  // Field name: svh
  {
    cdr >> ros_message->svh;
  }

  // Field name: week
  {
    cdr >> ros_message->week;
  }

  // Field name: code
  {
    cdr >> ros_message->code;
  }

  // Field name: flag
  {
    cdr >> ros_message->flag;
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

  // Field name: toc
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->toc))
    {
      return false;
    }
  }

  // Field name: ttr
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GTime
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->ttr))
    {
      return false;
    }
  }

  // Field name: a
  {
    cdr >> ros_message->a;
  }

  // Field name: e
  {
    cdr >> ros_message->e;
  }

  // Field name: i0
  {
    cdr >> ros_message->i0;
  }

  // Field name: omg_0
  {
    cdr >> ros_message->omg_0;
  }

  // Field name: omg
  {
    cdr >> ros_message->omg;
  }

  // Field name: m_0
  {
    cdr >> ros_message->m_0;
  }

  // Field name: deln
  {
    cdr >> ros_message->deln;
  }

  // Field name: omg_d
  {
    cdr >> ros_message->omg_d;
  }

  // Field name: idot
  {
    cdr >> ros_message->idot;
  }

  // Field name: crc
  {
    cdr >> ros_message->crc;
  }

  // Field name: crs
  {
    cdr >> ros_message->crs;
  }

  // Field name: cuc
  {
    cdr >> ros_message->cuc;
  }

  // Field name: cus
  {
    cdr >> ros_message->cus;
  }

  // Field name: cic
  {
    cdr >> ros_message->cic;
  }

  // Field name: cis
  {
    cdr >> ros_message->cis;
  }

  // Field name: toes
  {
    cdr >> ros_message->toes;
  }

  // Field name: fit
  {
    cdr >> ros_message->fit;
  }

  // Field name: f0
  {
    cdr >> ros_message->f0;
  }

  // Field name: f1
  {
    cdr >> ros_message->f1;
  }

  // Field name: f2
  {
    cdr >> ros_message->f2;
  }

  // Field name: tgd
  {
    size_t size = 4;
    auto array_ptr = ros_message->tgd;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: a_dot
  {
    cdr >> ros_message->a_dot;
  }

  // Field name: ndot
  {
    cdr >> ros_message->ndot;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t get_serialized_size_inertial_sense_ros2__msg__GNSSEphemeris(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GNSSEphemeris__ros_msg_type * ros_message = static_cast<const _GNSSEphemeris__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
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
  // field.name iodc
  {
    size_t item_size = sizeof(ros_message->iodc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sva
  {
    size_t item_size = sizeof(ros_message->sva);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name svh
  {
    size_t item_size = sizeof(ros_message->svh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name week
  {
    size_t item_size = sizeof(ros_message->week);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name code
  {
    size_t item_size = sizeof(ros_message->code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name flag
  {
    size_t item_size = sizeof(ros_message->flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name toe

  current_alignment += get_serialized_size_inertial_sense_ros2__msg__GTime(
    &(ros_message->toe), current_alignment);
  // field.name toc

  current_alignment += get_serialized_size_inertial_sense_ros2__msg__GTime(
    &(ros_message->toc), current_alignment);
  // field.name ttr

  current_alignment += get_serialized_size_inertial_sense_ros2__msg__GTime(
    &(ros_message->ttr), current_alignment);
  // field.name a
  {
    size_t item_size = sizeof(ros_message->a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name e
  {
    size_t item_size = sizeof(ros_message->e);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name i0
  {
    size_t item_size = sizeof(ros_message->i0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name omg_0
  {
    size_t item_size = sizeof(ros_message->omg_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name omg
  {
    size_t item_size = sizeof(ros_message->omg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name m_0
  {
    size_t item_size = sizeof(ros_message->m_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name deln
  {
    size_t item_size = sizeof(ros_message->deln);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name omg_d
  {
    size_t item_size = sizeof(ros_message->omg_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name idot
  {
    size_t item_size = sizeof(ros_message->idot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name crc
  {
    size_t item_size = sizeof(ros_message->crc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name crs
  {
    size_t item_size = sizeof(ros_message->crs);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cuc
  {
    size_t item_size = sizeof(ros_message->cuc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cus
  {
    size_t item_size = sizeof(ros_message->cus);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cic
  {
    size_t item_size = sizeof(ros_message->cic);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cis
  {
    size_t item_size = sizeof(ros_message->cis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name toes
  {
    size_t item_size = sizeof(ros_message->toes);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fit
  {
    size_t item_size = sizeof(ros_message->fit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name f0
  {
    size_t item_size = sizeof(ros_message->f0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name f1
  {
    size_t item_size = sizeof(ros_message->f1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name f2
  {
    size_t item_size = sizeof(ros_message->f2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tgd
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->tgd;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name a_dot
  {
    size_t item_size = sizeof(ros_message->a_dot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ndot
  {
    size_t item_size = sizeof(ros_message->ndot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GNSSEphemeris__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_inertial_sense_ros2__msg__GNSSEphemeris(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_inertial_sense_ros2
size_t max_serialized_size_inertial_sense_ros2__msg__GNSSEphemeris(
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
  // member: iodc
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
  // member: svh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: week
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: flag
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
  // member: toc
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
  // member: ttr
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
  // member: a
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: e
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: i0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: omg_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: omg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: m_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: deln
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: omg_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: idot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: crc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: crs
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: cuc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: cus
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: cic
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: cis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: toes
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: fit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: f0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: f1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: f2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tgd
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: a_dot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ndot
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
    using DataType = inertial_sense_ros2__msg__GNSSEphemeris;
    is_plain =
      (
      offsetof(DataType, ndot) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GNSSEphemeris__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_inertial_sense_ros2__msg__GNSSEphemeris(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GNSSEphemeris = {
  "inertial_sense_ros2::msg",
  "GNSSEphemeris",
  _GNSSEphemeris__cdr_serialize,
  _GNSSEphemeris__cdr_deserialize,
  _GNSSEphemeris__get_serialized_size,
  _GNSSEphemeris__max_serialized_size
};

static rosidl_message_type_support_t _GNSSEphemeris__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GNSSEphemeris,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GNSSEphemeris__get_type_hash,
  &inertial_sense_ros2__msg__GNSSEphemeris__get_type_description,
  &inertial_sense_ros2__msg__GNSSEphemeris__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, inertial_sense_ros2, msg, GNSSEphemeris)() {
  return &_GNSSEphemeris__type_support;
}

#if defined(__cplusplus)
}
#endif
