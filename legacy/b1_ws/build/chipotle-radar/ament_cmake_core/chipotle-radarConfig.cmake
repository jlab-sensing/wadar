# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_chipotle-radar_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED chipotle-radar_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(chipotle-radar_FOUND FALSE)
  elseif(NOT chipotle-radar_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(chipotle-radar_FOUND FALSE)
  endif()
  return()
endif()
set(_chipotle-radar_CONFIG_INCLUDED TRUE)

# output package information
if(NOT chipotle-radar_FIND_QUIETLY)
  message(STATUS "Found chipotle-radar: 0.0.0 (${chipotle-radar_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'chipotle-radar' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT chipotle-radar_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(chipotle-radar_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${chipotle-radar_DIR}/${_extra}")
endforeach()
