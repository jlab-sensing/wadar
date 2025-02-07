// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from inertial_sense_ros2:srv/FirmwareUpdate.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "inertial_sense_ros2/srv/detail/firmware_update__functions.h"
#include "inertial_sense_ros2/srv/detail/firmware_update__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace inertial_sense_ros2
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _FirmwareUpdate_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FirmwareUpdate_Request_type_support_ids_t;

static const _FirmwareUpdate_Request_type_support_ids_t _FirmwareUpdate_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FirmwareUpdate_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FirmwareUpdate_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FirmwareUpdate_Request_type_support_symbol_names_t _FirmwareUpdate_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Request)),
  }
};

typedef struct _FirmwareUpdate_Request_type_support_data_t
{
  void * data[2];
} _FirmwareUpdate_Request_type_support_data_t;

static _FirmwareUpdate_Request_type_support_data_t _FirmwareUpdate_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FirmwareUpdate_Request_message_typesupport_map = {
  2,
  "inertial_sense_ros2",
  &_FirmwareUpdate_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FirmwareUpdate_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FirmwareUpdate_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FirmwareUpdate_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FirmwareUpdate_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_hash,
  &inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_description,
  &inertial_sense_ros2__srv__FirmwareUpdate_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace inertial_sense_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Request>()
{
  return &::inertial_sense_ros2::srv::rosidl_typesupport_cpp::FirmwareUpdate_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Request)() {
  return get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__functions.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace inertial_sense_ros2
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _FirmwareUpdate_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FirmwareUpdate_Response_type_support_ids_t;

static const _FirmwareUpdate_Response_type_support_ids_t _FirmwareUpdate_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FirmwareUpdate_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FirmwareUpdate_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FirmwareUpdate_Response_type_support_symbol_names_t _FirmwareUpdate_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Response)),
  }
};

typedef struct _FirmwareUpdate_Response_type_support_data_t
{
  void * data[2];
} _FirmwareUpdate_Response_type_support_data_t;

static _FirmwareUpdate_Response_type_support_data_t _FirmwareUpdate_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FirmwareUpdate_Response_message_typesupport_map = {
  2,
  "inertial_sense_ros2",
  &_FirmwareUpdate_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FirmwareUpdate_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FirmwareUpdate_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FirmwareUpdate_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FirmwareUpdate_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_hash,
  &inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_description,
  &inertial_sense_ros2__srv__FirmwareUpdate_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace inertial_sense_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Response>()
{
  return &::inertial_sense_ros2::srv::rosidl_typesupport_cpp::FirmwareUpdate_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Response)() {
  return get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__functions.h"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace inertial_sense_ros2
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _FirmwareUpdate_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FirmwareUpdate_Event_type_support_ids_t;

static const _FirmwareUpdate_Event_type_support_ids_t _FirmwareUpdate_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FirmwareUpdate_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FirmwareUpdate_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FirmwareUpdate_Event_type_support_symbol_names_t _FirmwareUpdate_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Event)),
  }
};

typedef struct _FirmwareUpdate_Event_type_support_data_t
{
  void * data[2];
} _FirmwareUpdate_Event_type_support_data_t;

static _FirmwareUpdate_Event_type_support_data_t _FirmwareUpdate_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FirmwareUpdate_Event_message_typesupport_map = {
  2,
  "inertial_sense_ros2",
  &_FirmwareUpdate_Event_message_typesupport_ids.typesupport_identifier[0],
  &_FirmwareUpdate_Event_message_typesupport_symbol_names.symbol_name[0],
  &_FirmwareUpdate_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FirmwareUpdate_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FirmwareUpdate_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_hash,
  &inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_description,
  &inertial_sense_ros2__srv__FirmwareUpdate_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace inertial_sense_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Event>()
{
  return &::inertial_sense_ros2::srv::rosidl_typesupport_cpp::FirmwareUpdate_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, inertial_sense_ros2, srv, FirmwareUpdate_Event)() {
  return get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "inertial_sense_ros2/srv/detail/firmware_update__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace inertial_sense_ros2
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _FirmwareUpdate_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FirmwareUpdate_type_support_ids_t;

static const _FirmwareUpdate_type_support_ids_t _FirmwareUpdate_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FirmwareUpdate_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FirmwareUpdate_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FirmwareUpdate_type_support_symbol_names_t _FirmwareUpdate_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, inertial_sense_ros2, srv, FirmwareUpdate)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, inertial_sense_ros2, srv, FirmwareUpdate)),
  }
};

typedef struct _FirmwareUpdate_type_support_data_t
{
  void * data[2];
} _FirmwareUpdate_type_support_data_t;

static _FirmwareUpdate_type_support_data_t _FirmwareUpdate_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FirmwareUpdate_service_typesupport_map = {
  2,
  "inertial_sense_ros2",
  &_FirmwareUpdate_service_typesupport_ids.typesupport_identifier[0],
  &_FirmwareUpdate_service_typesupport_symbol_names.symbol_name[0],
  &_FirmwareUpdate_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FirmwareUpdate_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FirmwareUpdate_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<inertial_sense_ros2::srv::FirmwareUpdate>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<inertial_sense_ros2::srv::FirmwareUpdate>,
  &inertial_sense_ros2__srv__FirmwareUpdate__get_type_hash,
  &inertial_sense_ros2__srv__FirmwareUpdate__get_type_description,
  &inertial_sense_ros2__srv__FirmwareUpdate__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace inertial_sense_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<inertial_sense_ros2::srv::FirmwareUpdate>()
{
  return &::inertial_sense_ros2::srv::rosidl_typesupport_cpp::FirmwareUpdate_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
