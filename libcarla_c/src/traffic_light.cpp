#include "carla_c/traffic_light.h"
#include "carla/client/TrafficLight.h"
#include "carla/rpc/TrafficLightState.h"
#include "internal.h"
#include <memory>

// Convert C traffic light state to C++ traffic light state
carla::rpc::TrafficLightState
convert_to_cpp_state(carla_traffic_light_state_t c_state) {
  switch (c_state) {
  case CARLA_TRAFFIC_LIGHT_RED:
    return carla::rpc::TrafficLightState::Red;
  case CARLA_TRAFFIC_LIGHT_YELLOW:
    return carla::rpc::TrafficLightState::Yellow;
  case CARLA_TRAFFIC_LIGHT_GREEN:
    return carla::rpc::TrafficLightState::Green;
  case CARLA_TRAFFIC_LIGHT_OFF:
    return carla::rpc::TrafficLightState::Off;
  case CARLA_TRAFFIC_LIGHT_UNKNOWN:
  default:
    return carla::rpc::TrafficLightState::Unknown;
  }
}

// Convert C++ traffic light state to C traffic light state
carla_traffic_light_state_t
convert_to_c_state(carla::rpc::TrafficLightState cpp_state) {
  switch (cpp_state) {
  case carla::rpc::TrafficLightState::Red:
    return CARLA_TRAFFIC_LIGHT_RED;
  case carla::rpc::TrafficLightState::Yellow:
    return CARLA_TRAFFIC_LIGHT_YELLOW;
  case carla::rpc::TrafficLightState::Green:
    return CARLA_TRAFFIC_LIGHT_GREEN;
  case carla::rpc::TrafficLightState::Off:
    return CARLA_TRAFFIC_LIGHT_OFF;
  case carla::rpc::TrafficLightState::Unknown:
  default:
    return CARLA_TRAFFIC_LIGHT_UNKNOWN;
  }
}

carla_error_t carla_traffic_light_set_state(carla_actor_t *traffic_light,
                                            carla_traffic_light_state_t state) {
  if (!traffic_light || !traffic_light->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    auto cpp_state = convert_to_cpp_state(state);
    traffic_light_ptr->SetState(cpp_state);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_traffic_light_state_t
carla_traffic_light_get_state(const carla_actor_t *traffic_light) {
  if (!traffic_light || !traffic_light->actor) {
    return CARLA_TRAFFIC_LIGHT_UNKNOWN;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return CARLA_TRAFFIC_LIGHT_UNKNOWN;
    }

    auto cpp_state = traffic_light_ptr->GetState();
    return convert_to_c_state(cpp_state);
  } catch (const std::exception &e) {
    return CARLA_TRAFFIC_LIGHT_UNKNOWN;
  }
}

carla_error_t carla_traffic_light_set_green_time(carla_actor_t *traffic_light,
                                                 float green_time) {
  if (!traffic_light || !traffic_light->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    traffic_light_ptr->SetGreenTime(green_time);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

float carla_traffic_light_get_green_time(const carla_actor_t *traffic_light) {
  if (!traffic_light || !traffic_light->actor) {
    return 0.0f;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return 0.0f;
    }

    return traffic_light_ptr->GetGreenTime();
  } catch (const std::exception &e) {
    return 0.0f;
  }
}

carla_error_t carla_traffic_light_set_yellow_time(carla_actor_t *traffic_light,
                                                  float yellow_time) {
  if (!traffic_light || !traffic_light->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    traffic_light_ptr->SetYellowTime(yellow_time);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

float carla_traffic_light_get_yellow_time(const carla_actor_t *traffic_light) {
  if (!traffic_light || !traffic_light->actor) {
    return 0.0f;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return 0.0f;
    }

    return traffic_light_ptr->GetYellowTime();
  } catch (const std::exception &e) {
    return 0.0f;
  }
}

carla_error_t carla_traffic_light_set_red_time(carla_actor_t *traffic_light,
                                               float red_time) {
  if (!traffic_light || !traffic_light->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    traffic_light_ptr->SetRedTime(red_time);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

float carla_traffic_light_get_red_time(const carla_actor_t *traffic_light) {
  if (!traffic_light || !traffic_light->actor) {
    return 0.0f;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return 0.0f;
    }

    return traffic_light_ptr->GetRedTime();
  } catch (const std::exception &e) {
    return 0.0f;
  }
}

float carla_traffic_light_get_elapsed_time(const carla_actor_t *traffic_light) {
  if (!traffic_light || !traffic_light->actor) {
    return 0.0f;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return 0.0f;
    }

    return traffic_light_ptr->GetElapsedTime();
  } catch (const std::exception &e) {
    return 0.0f;
  }
}

carla_error_t carla_traffic_light_freeze(carla_actor_t *traffic_light,
                                         bool freeze) {
  if (!traffic_light || !traffic_light->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    traffic_light_ptr->Freeze(freeze);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

bool carla_traffic_light_is_frozen(const carla_actor_t *traffic_light) {
  if (!traffic_light || !traffic_light->actor) {
    return false;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return false;
    }

    return traffic_light_ptr->IsFrozen();
  } catch (const std::exception &e) {
    return false;
  }
}

uint32_t
carla_traffic_light_get_pole_index(const carla_actor_t *traffic_light) {
  if (!traffic_light || !traffic_light->actor) {
    return 0;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return 0;
    }

    return traffic_light_ptr->GetPoleIndex();
  } catch (const std::exception &e) {
    return 0;
  }
}

carla_error_t carla_traffic_light_reset_group(carla_actor_t *traffic_light) {
  if (!traffic_light || !traffic_light->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto traffic_light_ptr =
        std::dynamic_pointer_cast<carla::client::TrafficLight>(
            traffic_light->actor);
    if (!traffic_light_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    traffic_light_ptr->ResetGroup();
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Note: The following functions require additional type definitions that are
// not yet implemented in the C wrapper (carla_actor_list_t,
// carla_waypoint_list_t, carla_bounding_box_list_t) These will be implemented
// when those types are available
//
// carla_actor_list_t *carla_traffic_light_get_group_traffic_lights(const
// carla_actor_t *traffic_light); carla_waypoint_list_t
// *carla_traffic_light_get_affected_lane_waypoints(const carla_actor_t
// *traffic_light); carla_waypoint_list_t
// *carla_traffic_light_get_stop_waypoints(const carla_actor_t *traffic_light);
// carla_bounding_box_list_t *carla_traffic_light_get_light_boxes(const
// carla_actor_t *traffic_light);
