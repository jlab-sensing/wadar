// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
// generated code does not contain a copyright notice
#include "inertial_sense_ros2/msg/detail/rtk_info__rosidl_typesupport_fastrtps_cpp.hpp"
#include "inertial_sense_ros2/msg/detail/rtk_info__functions.h"
#include "inertial_sense_ros2/msg/detail/rtk_info__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace inertial_sense_ros2
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_inertial_sense_ros2
cdr_serialize(
  const inertial_sense_ros2::msg::RTKInfo & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: base_lla
  {
    cdr << ros_message.base_lla;
  }
  // Member: cycle_slip_count
  cdr << ros_message.cycle_slip_count;
  // Member: rover_obs
  cdr << ros_message.rover_obs;
  // Member: base_obs
  cdr << ros_message.base_obs;
  // Member: rover_eph
  cdr << ros_message.rover_eph;
  // Member: base_eph
  cdr << ros_message.base_eph;
  // Member: base_ant_count
  cdr << ros_message.base_ant_count;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_inertial_sense_ros2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  inertial_sense_ros2::msg::RTKInfo & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: base_lla
  {
    cdr >> ros_message.base_lla;
  }

  // Member: cycle_slip_count
  cdr >> ros_message.cycle_slip_count;

  // Member: rover_obs
  cdr >> ros_message.rover_obs;

  // Member: base_obs
  cdr >> ros_message.base_obs;

  // Member: rover_eph
  cdr >> ros_message.rover_eph;

  // Member: base_eph
  cdr >> ros_message.base_eph;

  // Member: base_ant_count
  cdr >> ros_message.base_ant_count;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_inertial_sense_ros2
get_serialized_size(
  const inertial_sense_ros2::msg::RTKInfo & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: base_lla
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.base_lla[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cycle_slip_count
  {
    size_t item_size = sizeof(ros_message.cycle_slip_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rover_obs
  {
    size_t item_size = sizeof(ros_message.rover_obs);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: base_obs
  {
    size_t item_size = sizeof(ros_message.base_obs);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rover_eph
  {
    size_t item_size = sizeof(ros_message.rover_eph);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: base_eph
  {
    size_t item_size = sizeof(ros_message.base_eph);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: base_ant_count
  {
    size_t item_size = sizeof(ros_message.base_ant_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_inertial_sense_ros2
max_serialized_size_RTKInfo(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: base_lla
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: cycle_slip_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rover_obs
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: base_obs
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rover_eph
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: base_eph
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: base_ant_count
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
    using DataType = inertial_sense_ros2::msg::RTKInfo;
    is_plain =
      (
      offsetof(DataType, base_ant_count) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RTKInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const inertial_sense_ros2::msg::RTKInfo *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RTKInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<inertial_sense_ros2::msg::RTKInfo *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RTKInfo__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const inertial_sense_ros2::msg::RTKInfo *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RTKInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RTKInfo(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RTKInfo__callbacks = {
  "inertial_sense_ros2::msg",
  "RTKInfo",
  _RTKInfo__cdr_serialize,
  _RTKInfo__cdr_deserialize,
  _RTKInfo__get_serialized_size,
  _RTKInfo__max_serialized_size
};

static rosidl_message_type_support_t _RTKInfo__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RTKInfo__callbacks,
  get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__RTKInfo__get_type_hash,
  &inertial_sense_ros2__msg__RTKInfo__get_type_description,
  &inertial_sense_ros2__msg__RTKInfo__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace inertial_sense_ros2

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_inertial_sense_ros2
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::msg::RTKInfo>()
{
  return &inertial_sense_ros2::msg::typesupport_fastrtps_cpp::_RTKInfo__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, inertial_sense_ros2, msg, RTKInfo)() {
  return &inertial_sense_ros2::msg::typesupport_fastrtps_cpp::_RTKInfo__handle;
}

#ifdef __cplusplus
}
#endif
