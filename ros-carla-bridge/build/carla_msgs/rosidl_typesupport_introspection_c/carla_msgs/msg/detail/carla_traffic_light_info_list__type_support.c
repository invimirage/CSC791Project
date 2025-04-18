// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from carla_msgs:msg/CarlaTrafficLightInfoList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "carla_msgs/msg/detail/carla_traffic_light_info_list__rosidl_typesupport_introspection_c.h"
#include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "carla_msgs/msg/detail/carla_traffic_light_info_list__functions.h"
#include "carla_msgs/msg/detail/carla_traffic_light_info_list__struct.h"


// Include directives for member types
// Member `traffic_lights`
#include "carla_msgs/msg/carla_traffic_light_info.h"
// Member `traffic_lights`
#include "carla_msgs/msg/detail/carla_traffic_light_info__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_msgs__msg__CarlaTrafficLightInfoList__init(message_memory);
}

void carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_fini_function(void * message_memory)
{
  carla_msgs__msg__CarlaTrafficLightInfoList__fini(message_memory);
}

size_t carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__size_function__CarlaTrafficLightInfoList__traffic_lights(
  const void * untyped_member)
{
  const carla_msgs__msg__CarlaTrafficLightInfo__Sequence * member =
    (const carla_msgs__msg__CarlaTrafficLightInfo__Sequence *)(untyped_member);
  return member->size;
}

const void * carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__get_const_function__CarlaTrafficLightInfoList__traffic_lights(
  const void * untyped_member, size_t index)
{
  const carla_msgs__msg__CarlaTrafficLightInfo__Sequence * member =
    (const carla_msgs__msg__CarlaTrafficLightInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void * carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__get_function__CarlaTrafficLightInfoList__traffic_lights(
  void * untyped_member, size_t index)
{
  carla_msgs__msg__CarlaTrafficLightInfo__Sequence * member =
    (carla_msgs__msg__CarlaTrafficLightInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__fetch_function__CarlaTrafficLightInfoList__traffic_lights(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const carla_msgs__msg__CarlaTrafficLightInfo * item =
    ((const carla_msgs__msg__CarlaTrafficLightInfo *)
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__get_const_function__CarlaTrafficLightInfoList__traffic_lights(untyped_member, index));
  carla_msgs__msg__CarlaTrafficLightInfo * value =
    (carla_msgs__msg__CarlaTrafficLightInfo *)(untyped_value);
  *value = *item;
}

void carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__assign_function__CarlaTrafficLightInfoList__traffic_lights(
  void * untyped_member, size_t index, const void * untyped_value)
{
  carla_msgs__msg__CarlaTrafficLightInfo * item =
    ((carla_msgs__msg__CarlaTrafficLightInfo *)
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__get_function__CarlaTrafficLightInfoList__traffic_lights(untyped_member, index));
  const carla_msgs__msg__CarlaTrafficLightInfo * value =
    (const carla_msgs__msg__CarlaTrafficLightInfo *)(untyped_value);
  *item = *value;
}

bool carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__resize_function__CarlaTrafficLightInfoList__traffic_lights(
  void * untyped_member, size_t size)
{
  carla_msgs__msg__CarlaTrafficLightInfo__Sequence * member =
    (carla_msgs__msg__CarlaTrafficLightInfo__Sequence *)(untyped_member);
  carla_msgs__msg__CarlaTrafficLightInfo__Sequence__fini(member);
  return carla_msgs__msg__CarlaTrafficLightInfo__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_member_array[1] = {
  {
    "traffic_lights",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__msg__CarlaTrafficLightInfoList, traffic_lights),  // bytes offset in struct
    NULL,  // default value
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__size_function__CarlaTrafficLightInfoList__traffic_lights,  // size() function pointer
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__get_const_function__CarlaTrafficLightInfoList__traffic_lights,  // get_const(index) function pointer
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__get_function__CarlaTrafficLightInfoList__traffic_lights,  // get(index) function pointer
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__fetch_function__CarlaTrafficLightInfoList__traffic_lights,  // fetch(index, &value) function pointer
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__assign_function__CarlaTrafficLightInfoList__traffic_lights,  // assign(index, value) function pointer
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__resize_function__CarlaTrafficLightInfoList__traffic_lights  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_members = {
  "carla_msgs__msg",  // message namespace
  "CarlaTrafficLightInfoList",  // message name
  1,  // number of fields
  sizeof(carla_msgs__msg__CarlaTrafficLightInfoList),
  carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_member_array,  // message members
  carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_type_support_handle = {
  0,
  &carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, msg, CarlaTrafficLightInfoList)() {
  carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, msg, CarlaTrafficLightInfo)();
  if (!carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_type_support_handle.typesupport_identifier) {
    carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_msgs__msg__CarlaTrafficLightInfoList__rosidl_typesupport_introspection_c__CarlaTrafficLightInfoList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
