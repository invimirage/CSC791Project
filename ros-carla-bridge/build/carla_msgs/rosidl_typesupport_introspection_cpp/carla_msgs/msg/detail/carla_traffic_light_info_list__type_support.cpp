// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from carla_msgs:msg/CarlaTrafficLightInfoList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "carla_msgs/msg/detail/carla_traffic_light_info_list__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace carla_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CarlaTrafficLightInfoList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) carla_msgs::msg::CarlaTrafficLightInfoList(_init);
}

void CarlaTrafficLightInfoList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<carla_msgs::msg::CarlaTrafficLightInfoList *>(message_memory);
  typed_message->~CarlaTrafficLightInfoList();
}

size_t size_function__CarlaTrafficLightInfoList__traffic_lights(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<carla_msgs::msg::CarlaTrafficLightInfo> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CarlaTrafficLightInfoList__traffic_lights(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<carla_msgs::msg::CarlaTrafficLightInfo> *>(untyped_member);
  return &member[index];
}

void * get_function__CarlaTrafficLightInfoList__traffic_lights(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<carla_msgs::msg::CarlaTrafficLightInfo> *>(untyped_member);
  return &member[index];
}

void fetch_function__CarlaTrafficLightInfoList__traffic_lights(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const carla_msgs::msg::CarlaTrafficLightInfo *>(
    get_const_function__CarlaTrafficLightInfoList__traffic_lights(untyped_member, index));
  auto & value = *reinterpret_cast<carla_msgs::msg::CarlaTrafficLightInfo *>(untyped_value);
  value = item;
}

void assign_function__CarlaTrafficLightInfoList__traffic_lights(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<carla_msgs::msg::CarlaTrafficLightInfo *>(
    get_function__CarlaTrafficLightInfoList__traffic_lights(untyped_member, index));
  const auto & value = *reinterpret_cast<const carla_msgs::msg::CarlaTrafficLightInfo *>(untyped_value);
  item = value;
}

void resize_function__CarlaTrafficLightInfoList__traffic_lights(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<carla_msgs::msg::CarlaTrafficLightInfo> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CarlaTrafficLightInfoList_message_member_array[1] = {
  {
    "traffic_lights",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<carla_msgs::msg::CarlaTrafficLightInfo>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs::msg::CarlaTrafficLightInfoList, traffic_lights),  // bytes offset in struct
    nullptr,  // default value
    size_function__CarlaTrafficLightInfoList__traffic_lights,  // size() function pointer
    get_const_function__CarlaTrafficLightInfoList__traffic_lights,  // get_const(index) function pointer
    get_function__CarlaTrafficLightInfoList__traffic_lights,  // get(index) function pointer
    fetch_function__CarlaTrafficLightInfoList__traffic_lights,  // fetch(index, &value) function pointer
    assign_function__CarlaTrafficLightInfoList__traffic_lights,  // assign(index, value) function pointer
    resize_function__CarlaTrafficLightInfoList__traffic_lights  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CarlaTrafficLightInfoList_message_members = {
  "carla_msgs::msg",  // message namespace
  "CarlaTrafficLightInfoList",  // message name
  1,  // number of fields
  sizeof(carla_msgs::msg::CarlaTrafficLightInfoList),
  CarlaTrafficLightInfoList_message_member_array,  // message members
  CarlaTrafficLightInfoList_init_function,  // function to initialize message memory (memory has to be allocated)
  CarlaTrafficLightInfoList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CarlaTrafficLightInfoList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CarlaTrafficLightInfoList_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace carla_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<carla_msgs::msg::CarlaTrafficLightInfoList>()
{
  return &::carla_msgs::msg::rosidl_typesupport_introspection_cpp::CarlaTrafficLightInfoList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, carla_msgs, msg, CarlaTrafficLightInfoList)() {
  return &::carla_msgs::msg::rosidl_typesupport_introspection_cpp::CarlaTrafficLightInfoList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
