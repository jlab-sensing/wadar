// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice

#ifndef WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__BUILDER_HPP_
#define WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wadar_interfaces/msg/detail/tag_relative_location__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wadar_interfaces
{

namespace msg
{

namespace builder
{

class Init_TagRelativeLocation_distance
{
public:
  explicit Init_TagRelativeLocation_distance(::wadar_interfaces::msg::TagRelativeLocation & msg)
  : msg_(msg)
  {}
  ::wadar_interfaces::msg::TagRelativeLocation distance(::wadar_interfaces::msg::TagRelativeLocation::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wadar_interfaces::msg::TagRelativeLocation msg_;
};

class Init_TagRelativeLocation_alignment
{
public:
  explicit Init_TagRelativeLocation_alignment(::wadar_interfaces::msg::TagRelativeLocation & msg)
  : msg_(msg)
  {}
  Init_TagRelativeLocation_distance alignment(::wadar_interfaces::msg::TagRelativeLocation::_alignment_type arg)
  {
    msg_.alignment = std::move(arg);
    return Init_TagRelativeLocation_distance(msg_);
  }

private:
  ::wadar_interfaces::msg::TagRelativeLocation msg_;
};

class Init_TagRelativeLocation_relative_heading
{
public:
  Init_TagRelativeLocation_relative_heading()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TagRelativeLocation_alignment relative_heading(::wadar_interfaces::msg::TagRelativeLocation::_relative_heading_type arg)
  {
    msg_.relative_heading = std::move(arg);
    return Init_TagRelativeLocation_alignment(msg_);
  }

private:
  ::wadar_interfaces::msg::TagRelativeLocation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wadar_interfaces::msg::TagRelativeLocation>()
{
  return wadar_interfaces::msg::builder::Init_TagRelativeLocation_relative_heading();
}

}  // namespace wadar_interfaces

#endif  // WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__BUILDER_HPP_
