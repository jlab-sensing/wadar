// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "inertial_sense_ros2/msg/detail/gnss_observation__functions.h"
#include "inertial_sense_ros2/msg/detail/gnss_observation__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace inertial_sense_ros2
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _GNSSObservation_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GNSSObservation_type_support_ids_t;

static const _GNSSObservation_type_support_ids_t _GNSSObservation_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GNSSObservation_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GNSSObservation_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GNSSObservation_type_support_symbol_names_t _GNSSObservation_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, inertial_sense_ros2, msg, GNSSObservation)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, msg, GNSSObservation)),
  }
};

typedef struct _GNSSObservation_type_support_data_t
{
  void * data[2];
} _GNSSObservation_type_support_data_t;

static _GNSSObservation_type_support_data_t _GNSSObservation_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GNSSObservation_message_typesupport_map = {
  2,
  "inertial_sense_ros2",
  &_GNSSObservation_message_typesupport_ids.typesupport_identifier[0],
  &_GNSSObservation_message_typesupport_symbol_names.symbol_name[0],
  &_GNSSObservation_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GNSSObservation_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GNSSObservation_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &inertial_sense_ros2__msg__GNSSObservation__get_type_hash,
  &inertial_sense_ros2__msg__GNSSObservation__get_type_description,
  &inertial_sense_ros2__msg__GNSSObservation__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace inertial_sense_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::msg::GNSSObservation>()
{
  return &::inertial_sense_ros2::msg::rosidl_typesupport_cpp::GNSSObservation_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, inertial_sense_ros2, msg, GNSSObservation)() {
  return get_message_type_support_handle<inertial_sense_ros2::msg::GNSSObservation>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
