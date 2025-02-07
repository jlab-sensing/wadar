// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
// generated code does not contain a copyright notice

#ifndef WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__STRUCT_HPP_
#define WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__wadar_interfaces__msg__TagRelativeLocation __attribute__((deprecated))
#else
# define DEPRECATED__wadar_interfaces__msg__TagRelativeLocation __declspec(deprecated)
#endif

namespace wadar_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TagRelativeLocation_
{
  using Type = TagRelativeLocation_<ContainerAllocator>;

  explicit TagRelativeLocation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->relative_heading = 0.0f;
      this->alignment = 0.0f;
      this->distance = 0.0f;
    }
  }

  explicit TagRelativeLocation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->relative_heading = 0.0f;
      this->alignment = 0.0f;
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _relative_heading_type =
    float;
  _relative_heading_type relative_heading;
  using _alignment_type =
    float;
  _alignment_type alignment;
  using _distance_type =
    float;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__relative_heading(
    const float & _arg)
  {
    this->relative_heading = _arg;
    return *this;
  }
  Type & set__alignment(
    const float & _arg)
  {
    this->alignment = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator> *;
  using ConstRawPtr =
    const wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wadar_interfaces__msg__TagRelativeLocation
    std::shared_ptr<wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wadar_interfaces__msg__TagRelativeLocation
    std::shared_ptr<wadar_interfaces::msg::TagRelativeLocation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TagRelativeLocation_ & other) const
  {
    if (this->relative_heading != other.relative_heading) {
      return false;
    }
    if (this->alignment != other.alignment) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const TagRelativeLocation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TagRelativeLocation_

// alias to use template instance with default allocator
using TagRelativeLocation =
  wadar_interfaces::msg::TagRelativeLocation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wadar_interfaces

#endif  // WADAR_INTERFACES__MSG__DETAIL__TAG_RELATIVE_LOCATION__STRUCT_HPP_
