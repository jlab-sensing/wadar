// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__rosidl_typesupport_fastrtps_cpp.hpp"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__functions.h"
#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__struct.hpp"

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
namespace inertial_sense_ros2
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const inertial_sense_ros2::msg::GTime &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  inertial_sense_ros2::msg::GTime &);
size_t get_serialized_size(
  const inertial_sense_ros2::msg::GTime &,
  size_t current_alignment);
size_t
max_serialized_size_GTime(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace inertial_sense_ros2

// functions for inertial_sense_ros2::msg::GTime already declared above


namespace inertial_sense_ros2
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_inertial_sense_ros2
cdr_serialize(
  const inertial_sense_ros2::msg::GlonassEphemeris & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: sat
  cdr << ros_message.sat;
  // Member: iode
  cdr << ros_message.iode;
  // Member: frq
  cdr << ros_message.frq;
  // Member: svh
  cdr << ros_message.svh;
  // Member: sva
  cdr << ros_message.sva;
  // Member: age
  cdr << ros_message.age;
  // Member: toe
  inertial_sense_ros2::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.toe,
    cdr);
  // Member: tof
  inertial_sense_ros2::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.tof,
    cdr);
  // Member: pos
  {
    cdr << ros_message.pos;
  }
  // Member: vel
  {
    cdr << ros_message.vel;
  }
  // Member: acc
  {
    cdr << ros_message.acc;
  }
  // Member: taun
  cdr << ros_message.taun;
  // Member: gamn
  cdr << ros_message.gamn;
  // Member: dtaun
  cdr << ros_message.dtaun;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_inertial_sense_ros2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  inertial_sense_ros2::msg::GlonassEphemeris & ros_message)
{
  // Member: sat
  cdr >> ros_message.sat;

  // Member: iode
  cdr >> ros_message.iode;

  // Member: frq
  cdr >> ros_message.frq;

  // Member: svh
  cdr >> ros_message.svh;

  // Member: sva
  cdr >> ros_message.sva;

  // Member: age
  cdr >> ros_message.age;

  // Member: toe
  inertial_sense_ros2::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.toe);

  // Member: tof
  inertial_sense_ros2::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.tof);

  // Member: pos
  {
    cdr >> ros_message.pos;
  }

  // Member: vel
  {
    cdr >> ros_message.vel;
  }

  // Member: acc
  {
    cdr >> ros_message.acc;
  }

  // Member: taun
  cdr >> ros_message.taun;

  // Member: gamn
  cdr >> ros_message.gamn;

  // Member: dtaun
  cdr >> ros_message.dtaun;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_inertial_sense_ros2
get_serialized_size(
  const inertial_sense_ros2::msg::GlonassEphemeris & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: sat
  {
    size_t item_size = sizeof(ros_message.sat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: iode
  {
    size_t item_size = sizeof(ros_message.iode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: frq
  {
    size_t item_size = sizeof(ros_message.frq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: svh
  {
    size_t item_size = sizeof(ros_message.svh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sva
  {
    size_t item_size = sizeof(ros_message.sva);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: age
  {
    size_t item_size = sizeof(ros_message.age);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: toe

  current_alignment +=
    inertial_sense_ros2::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.toe, current_alignment);
  // Member: tof

  current_alignment +=
    inertial_sense_ros2::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.tof, current_alignment);
  // Member: pos
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.pos[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vel
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.vel[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acc
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.acc[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: taun
  {
    size_t item_size = sizeof(ros_message.taun);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gamn
  {
    size_t item_size = sizeof(ros_message.gamn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: dtaun
  {
    size_t item_size = sizeof(ros_message.dtaun);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_inertial_sense_ros2
max_serialized_size_GlonassEphemeris(
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


  // Member: sat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: iode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: frq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: svh
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: sva
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: age
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: toe
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        inertial_sense_ros2::msg::typesupport_fastrtps_cpp::max_serialized_size_GTime(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: tof
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        inertial_sense_ros2::msg::typesupport_fastrtps_cpp::max_serialized_size_GTime(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: pos
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: vel
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: acc
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: taun
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: gamn
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: dtaun
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
    using DataType = inertial_sense_ros2::msg::GlonassEphemeris;
    is_plain =
      (
      offsetof(DataType, dtaun) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GlonassEphemeris__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const inertial_sense_ros2::msg::GlonassEphemeris *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GlonassEphemeris__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<inertial_sense_ros2::msg::GlonassEphemeris *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GlonassEphemeris__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const inertial_sense_ros2::msg::GlonassEphemeris *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GlonassEphemeris__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GlonassEphemeris(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GlonassEphemeris__callbacks = {
  "inertial_sense_ros2::msg",
  "GlonassEphemeris",
  _GlonassEphemeris__cdr_serialize,
  _GlonassEphemeris__cdr_deserialize,
  _GlonassEphemeris__get_serialized_size,
  _GlonassEphemeris__max_serialized_size
};

static rosidl_message_type_support_t _GlonassEphemeris__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GlonassEphemeris__callbacks,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_hash,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_description,
  &inertial_sense_ros2__msg__GlonassEphemeris__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace inertial_sense_ros2

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::msg::GlonassEphemeris>()
{
  return &inertial_sense_ros2::msg::typesupport_fastrtps_cpp::_GlonassEphemeris__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, inertial_sense_ros2, msg, GlonassEphemeris)() {
  return &inertial_sense_ros2::msg::typesupport_fastrtps_cpp::_GlonassEphemeris__handle;
}

#ifdef __cplusplus
}
#endif
