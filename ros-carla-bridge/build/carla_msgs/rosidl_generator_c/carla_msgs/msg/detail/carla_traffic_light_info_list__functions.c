// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from carla_msgs:msg/CarlaTrafficLightInfoList.idl
// generated code does not contain a copyright notice
#include "carla_msgs/msg/detail/carla_traffic_light_info_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `traffic_lights`
#include "carla_msgs/msg/detail/carla_traffic_light_info__functions.h"

bool
carla_msgs__msg__CarlaTrafficLightInfoList__init(carla_msgs__msg__CarlaTrafficLightInfoList * msg)
{
  if (!msg) {
    return false;
  }
  // traffic_lights
  if (!carla_msgs__msg__CarlaTrafficLightInfo__Sequence__init(&msg->traffic_lights, 0)) {
    carla_msgs__msg__CarlaTrafficLightInfoList__fini(msg);
    return false;
  }
  return true;
}

void
carla_msgs__msg__CarlaTrafficLightInfoList__fini(carla_msgs__msg__CarlaTrafficLightInfoList * msg)
{
  if (!msg) {
    return;
  }
  // traffic_lights
  carla_msgs__msg__CarlaTrafficLightInfo__Sequence__fini(&msg->traffic_lights);
}

bool
carla_msgs__msg__CarlaTrafficLightInfoList__are_equal(const carla_msgs__msg__CarlaTrafficLightInfoList * lhs, const carla_msgs__msg__CarlaTrafficLightInfoList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // traffic_lights
  if (!carla_msgs__msg__CarlaTrafficLightInfo__Sequence__are_equal(
      &(lhs->traffic_lights), &(rhs->traffic_lights)))
  {
    return false;
  }
  return true;
}

bool
carla_msgs__msg__CarlaTrafficLightInfoList__copy(
  const carla_msgs__msg__CarlaTrafficLightInfoList * input,
  carla_msgs__msg__CarlaTrafficLightInfoList * output)
{
  if (!input || !output) {
    return false;
  }
  // traffic_lights
  if (!carla_msgs__msg__CarlaTrafficLightInfo__Sequence__copy(
      &(input->traffic_lights), &(output->traffic_lights)))
  {
    return false;
  }
  return true;
}

carla_msgs__msg__CarlaTrafficLightInfoList *
carla_msgs__msg__CarlaTrafficLightInfoList__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carla_msgs__msg__CarlaTrafficLightInfoList * msg = (carla_msgs__msg__CarlaTrafficLightInfoList *)allocator.allocate(sizeof(carla_msgs__msg__CarlaTrafficLightInfoList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(carla_msgs__msg__CarlaTrafficLightInfoList));
  bool success = carla_msgs__msg__CarlaTrafficLightInfoList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
carla_msgs__msg__CarlaTrafficLightInfoList__destroy(carla_msgs__msg__CarlaTrafficLightInfoList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    carla_msgs__msg__CarlaTrafficLightInfoList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
carla_msgs__msg__CarlaTrafficLightInfoList__Sequence__init(carla_msgs__msg__CarlaTrafficLightInfoList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carla_msgs__msg__CarlaTrafficLightInfoList * data = NULL;

  if (size) {
    data = (carla_msgs__msg__CarlaTrafficLightInfoList *)allocator.zero_allocate(size, sizeof(carla_msgs__msg__CarlaTrafficLightInfoList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = carla_msgs__msg__CarlaTrafficLightInfoList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        carla_msgs__msg__CarlaTrafficLightInfoList__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
carla_msgs__msg__CarlaTrafficLightInfoList__Sequence__fini(carla_msgs__msg__CarlaTrafficLightInfoList__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      carla_msgs__msg__CarlaTrafficLightInfoList__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

carla_msgs__msg__CarlaTrafficLightInfoList__Sequence *
carla_msgs__msg__CarlaTrafficLightInfoList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  carla_msgs__msg__CarlaTrafficLightInfoList__Sequence * array = (carla_msgs__msg__CarlaTrafficLightInfoList__Sequence *)allocator.allocate(sizeof(carla_msgs__msg__CarlaTrafficLightInfoList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = carla_msgs__msg__CarlaTrafficLightInfoList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
carla_msgs__msg__CarlaTrafficLightInfoList__Sequence__destroy(carla_msgs__msg__CarlaTrafficLightInfoList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    carla_msgs__msg__CarlaTrafficLightInfoList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
carla_msgs__msg__CarlaTrafficLightInfoList__Sequence__are_equal(const carla_msgs__msg__CarlaTrafficLightInfoList__Sequence * lhs, const carla_msgs__msg__CarlaTrafficLightInfoList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!carla_msgs__msg__CarlaTrafficLightInfoList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
carla_msgs__msg__CarlaTrafficLightInfoList__Sequence__copy(
  const carla_msgs__msg__CarlaTrafficLightInfoList__Sequence * input,
  carla_msgs__msg__CarlaTrafficLightInfoList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(carla_msgs__msg__CarlaTrafficLightInfoList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    carla_msgs__msg__CarlaTrafficLightInfoList * data =
      (carla_msgs__msg__CarlaTrafficLightInfoList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!carla_msgs__msg__CarlaTrafficLightInfoList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          carla_msgs__msg__CarlaTrafficLightInfoList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!carla_msgs__msg__CarlaTrafficLightInfoList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
