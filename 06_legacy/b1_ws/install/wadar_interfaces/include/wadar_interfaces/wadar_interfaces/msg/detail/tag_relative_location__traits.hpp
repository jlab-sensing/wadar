// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice

#ifndef WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__TRAITS_HPP_
#define WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wadar_interfaces/msg/detail/tag_relative_location__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace wadar_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TagRelativeLocation & msg,
  std::ostream & out)
{
  out << "{";
  // member: relative_heading
  {
    out << "relative_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.relative_heading, out);
    out << ", ";
  }

  // member: alignment
  {
    out << "alignment: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TagRelativeLocation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: relative_heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "relative_heading: ";
    rosidl_generator_traits::value_to_yaml(msg.relative_heading, out);
    out << "\n";
  }

  // member: alignment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TagRelativeLocation & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace wadar_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use wadar_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wadar_interfaces::msg::TagRelativeLocation & msg,
  std::ostream & out, size_t indentation = 0)
{
  wadar_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wadar_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const wadar_interfaces::msg::TagRelativeLocation & msg)
{
  return wadar_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wadar_interfaces::msg::TagRelativeLocation>()
{
  return "wadar_interfaces::msg::TagRelativeLocation";
}

template<>
inline const char * name<wadar_interfaces::msg::TagRelativeLocation>()
{
  return "wadar_interfaces/msg/TagRelativeLocation";
}

template<>
struct has_fixed_size<wadar_interfaces::msg::TagRelativeLocation>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<wadar_interfaces::msg::TagRelativeLocation>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<wadar_interfaces::msg::TagRelativeLocation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__TRAITS_HPP_
