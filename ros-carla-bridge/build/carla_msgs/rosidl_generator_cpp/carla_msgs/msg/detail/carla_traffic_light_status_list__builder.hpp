// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from carla_msgs:msg/CarlaTrafficLightStatusList.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_TRAFFIC_LIGHT_STATUS_LIST__BUILDER_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_TRAFFIC_LIGHT_STATUS_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "carla_msgs/msg/detail/carla_traffic_light_status_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace carla_msgs
{

namespace msg
{

namespace builder
{

class Init_CarlaTrafficLightStatusList_traffic_lights
{
public:
  Init_CarlaTrafficLightStatusList_traffic_lights()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::carla_msgs::msg::CarlaTrafficLightStatusList traffic_lights(::carla_msgs::msg::CarlaTrafficLightStatusList::_traffic_lights_type arg)
  {
    msg_.traffic_lights = std::move(arg);
    return std::move(msg_);
  }

private:
  ::carla_msgs::msg::CarlaTrafficLightStatusList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::carla_msgs::msg::CarlaTrafficLightStatusList>()
{
  return carla_msgs::msg::builder::Init_CarlaTrafficLightStatusList_traffic_lights();
}

}  // namespace carla_msgs

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_TRAFFIC_LIGHT_STATUS_LIST__BUILDER_HPP_
