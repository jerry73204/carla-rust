#include "carla_c/traffic_manager.h"
#include "internal.h"

#include <carla/client/Client.h>
#include <carla/trafficmanager/SimpleWaypoint.h>
#include <carla/trafficmanager/TrafficManager.h>

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

using namespace carla::traffic_manager;
using namespace carla::client;

// Internal traffic manager structure
struct carla_traffic_manager {
  carla::traffic_manager::TrafficManager *tm;
  uint16_t port;
  bool is_running;
  carla_traffic_manager_stats_t stats;
  std::unordered_map<carla::ActorId, carla_traffic_manager_vehicle_config_t>
      vehicle_configs;

  explicit carla_traffic_manager(
      carla::traffic_manager::TrafficManager *traffic_manager, uint16_t p)
      : tm(traffic_manager), port(p), is_running(true), stats{} {}
};

// Global map to track traffic manager instances by port
static std::map<uint16_t, std::unique_ptr<carla_traffic_manager>>
    g_traffic_managers;

// Helper functions

// Convert C actor to CARLA actor pointer
static carla::SharedPtr<carla::client::Actor>
get_carla_actor(carla_actor_t *actor) {
  if (!actor)
    return nullptr;
  return actor->actor;
}

// Convert C actors array to CARLA actor vector
static std::vector<carla::SharedPtr<carla::client::Actor>>
get_carla_actors(carla_actor_t **actors, size_t count) {
  std::vector<carla::SharedPtr<carla::client::Actor>> carla_actors;
  carla_actors.reserve(count);

  for (size_t i = 0; i < count; ++i) {
    if (actors[i]) {
      carla_actors.push_back(actors[i]->actor);
    }
  }

  return carla_actors;
}

// Convert CARLA road option to C road option
static carla_road_option_t convert_road_option(RoadOption road_option) {
  switch (road_option) {
  case RoadOption::Void:
    return CARLA_ROAD_OPTION_VOID;
  case RoadOption::Left:
    return CARLA_ROAD_OPTION_LEFT;
  case RoadOption::Right:
    return CARLA_ROAD_OPTION_RIGHT;
  case RoadOption::Straight:
    return CARLA_ROAD_OPTION_STRAIGHT;
  case RoadOption::LaneFollow:
    return CARLA_ROAD_OPTION_LANE_FOLLOW;
  case RoadOption::ChangeLaneLeft:
    return CARLA_ROAD_OPTION_CHANGE_LANE_LEFT;
  case RoadOption::ChangeLaneRight:
    return CARLA_ROAD_OPTION_CHANGE_LANE_RIGHT;
  case RoadOption::RoadEnd:
    return CARLA_ROAD_OPTION_ROAD_END;
  default:
    return CARLA_ROAD_OPTION_VOID;
  }
}

// Convert C road option to CARLA road option
static RoadOption convert_road_option(carla_road_option_t road_option) {
  switch (road_option) {
  case CARLA_ROAD_OPTION_VOID:
    return RoadOption::Void;
  case CARLA_ROAD_OPTION_LEFT:
    return RoadOption::Left;
  case CARLA_ROAD_OPTION_RIGHT:
    return RoadOption::Right;
  case CARLA_ROAD_OPTION_STRAIGHT:
    return RoadOption::Straight;
  case CARLA_ROAD_OPTION_LANE_FOLLOW:
    return RoadOption::LaneFollow;
  case CARLA_ROAD_OPTION_CHANGE_LANE_LEFT:
    return RoadOption::ChangeLaneLeft;
  case CARLA_ROAD_OPTION_CHANGE_LANE_RIGHT:
    return RoadOption::ChangeLaneRight;
  case CARLA_ROAD_OPTION_ROAD_END:
    return RoadOption::RoadEnd;
  default:
    return RoadOption::Void;
  }
}

// Convert CARLA locations to C vector3d
static carla_vector3d_t
convert_location(const carla::geom::Location &location) {
  return {static_cast<float>(location.x), static_cast<float>(location.y),
          static_cast<float>(location.z)};
}

// Convert C vector3d to CARLA location
static carla::geom::Location
convert_location(const carla_vector3d_t &location) {
  return carla::geom::Location(location.x, location.y, location.z);
}

// Traffic Manager lifecycle management

carla_traffic_manager_t *
carla_traffic_manager_get_instance(carla_client_t *client, uint16_t port) {
  if (!client || !client->client) {
    return nullptr;
  }

  // Check if we already have this instance
  auto it = g_traffic_managers.find(port);
  if (it != g_traffic_managers.end()) {
    return it->second.get();
  }

  try {
    // Get traffic manager from CARLA client
    auto tm = client->client->GetInstanceTM(port);

    // Create our wrapper
    auto wrapper = std::make_unique<carla_traffic_manager>(&tm, port);
    auto *result = wrapper.get();

    // Store in global map
    g_traffic_managers[port] = std::move(wrapper);

    return result;
  } catch (const std::exception &) {
    return nullptr;
  }
}

carla_traffic_manager_t *
carla_traffic_manager_get_default(carla_client_t *client) {
  return carla_traffic_manager_get_instance(client, CARLA_TM_DEFAULT_PORT);
}

void carla_traffic_manager_release_all(void) {
  try {
    carla::traffic_manager::TrafficManager::Release();
    g_traffic_managers.clear();
  } catch (const std::exception &) {
    // Ignore exceptions during cleanup
  }
}

void carla_traffic_manager_reset_all(void) {
  try {
    carla::traffic_manager::TrafficManager::Reset();
    // Reset statistics for all instances
    for (auto &pair : g_traffic_managers) {
      pair.second->stats = {};
    }
  } catch (const std::exception &) {
    // Ignore exceptions during reset
  }
}

void carla_traffic_manager_shutdown(carla_traffic_manager_t *tm) {
  if (!tm)
    return;

  try {
    tm->tm->ShutDown();
    tm->is_running = false;
  } catch (const std::exception &) {
    // Ignore exceptions during shutdown
  }
}

carla_error_t
carla_traffic_manager_get_info(carla_traffic_manager_t *tm,
                               carla_traffic_manager_info_t *info) {
  if (!tm || !info) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  info->port = tm->port;
  info->is_running = tm->is_running;
  info->registered_vehicle_count = tm->vehicle_configs.size();
  // Note: config would need to be tracked separately in real implementation
  info->config = carla_traffic_manager_create_default_config(tm->port);

  return CARLA_ERROR_NONE;
}

// Vehicle registration and management

carla_error_t carla_traffic_manager_register_vehicles(
    carla_traffic_manager_t *tm, carla_actor_t **actors, size_t actor_count) {
  if (!tm || !actors) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actors = get_carla_actors(actors, actor_count);
    tm->tm->RegisterVehicles(carla_actors);

    // Track vehicle configs
    for (const auto &actor : carla_actors) {
      tm->vehicle_configs[actor->GetId()] =
          carla_traffic_manager_create_default_vehicle_config();
    }

    tm->stats.total_registered_vehicles += static_cast<uint32_t>(actor_count);
    tm->stats.active_vehicle_count =
        static_cast<uint32_t>(tm->vehicle_configs.size());

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_register_vehicle(carla_traffic_manager_t *tm,
                                       carla_actor_t *actor) {
  carla_actor_t *actors[] = {actor};
  return carla_traffic_manager_register_vehicles(tm, actors, 1);
}

carla_error_t carla_traffic_manager_unregister_vehicles(
    carla_traffic_manager_t *tm, carla_actor_t **actors, size_t actor_count) {
  if (!tm || !actors) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actors = get_carla_actors(actors, actor_count);
    tm->tm->UnregisterVehicles(carla_actors);

    // Remove vehicle configs
    for (const auto &actor : carla_actors) {
      tm->vehicle_configs.erase(actor->GetId());
    }

    tm->stats.active_vehicle_count =
        static_cast<uint32_t>(tm->vehicle_configs.size());

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_unregister_vehicle(carla_traffic_manager_t *tm,
                                         carla_actor_t *actor) {
  carla_actor_t *actors[] = {actor};
  return carla_traffic_manager_unregister_vehicles(tm, actors, 1);
}

// Synchronous execution control

carla_error_t
carla_traffic_manager_set_synchronous_mode(carla_traffic_manager_t *tm,
                                           bool mode) {
  if (!tm) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    tm->tm->SetSynchronousMode(mode);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

bool carla_traffic_manager_synchronous_tick(carla_traffic_manager_t *tm) {
  if (!tm) {
    return false;
  }

  try {
    bool result = tm->tm->SynchronousTick();
    if (result) {
      tm->stats.total_ticks++;
    }
    return result;
  } catch (const std::exception &) {
    return false;
  }
}

carla_error_t
carla_traffic_manager_set_synchronous_timeout(carla_traffic_manager_t *tm,
                                              double timeout_ms) {
  if (!tm) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    tm->tm->SetSynchronousModeTimeOutInMiliSecond(timeout_ms);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Global configuration

carla_error_t
carla_traffic_manager_set_global_speed_percentage(carla_traffic_manager_t *tm,
                                                  float percentage) {
  if (!tm) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    tm->tm->SetGlobalPercentageSpeedDifference(percentage);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_set_global_lane_offset(carla_traffic_manager_t *tm,
                                             float offset) {
  if (!tm) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    tm->tm->SetGlobalLaneOffset(offset);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_set_global_distance_to_leading_vehicle(
    carla_traffic_manager_t *tm, float distance) {
  if (!tm) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    tm->tm->SetGlobalDistanceToLeadingVehicle(distance);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_set_random_device_seed(carla_traffic_manager_t *tm,
                                             uint64_t seed) {
  if (!tm) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    tm->tm->SetRandomDeviceSeed(seed);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_set_osm_mode(carla_traffic_manager_t *tm,
                                                 bool osm_mode) {
  if (!tm) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    tm->tm->SetOSMMode(osm_mode);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Per-vehicle speed control

carla_error_t carla_traffic_manager_set_vehicle_speed_percentage(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetPercentageSpeedDifference(carla_actor, percentage);

    // Update tracked config
    auto &config = tm->vehicle_configs[carla_actor->GetId()];
    config.speed_percentage_difference = percentage;
    config.desired_speed = 0.0f; // Reset desired speed when using percentage

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_set_vehicle_desired_speed(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float speed) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetDesiredSpeed(carla_actor, speed);

    // Update tracked config
    auto &config = tm->vehicle_configs[carla_actor->GetId()];
    config.desired_speed = speed;
    config.speed_percentage_difference =
        0.0f; // Reset percentage when using absolute speed

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Per-vehicle lane behavior

carla_error_t carla_traffic_manager_set_vehicle_lane_offset(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float offset) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetLaneOffset(carla_actor, offset);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].lane_offset = offset;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_set_vehicle_auto_lane_change(
    carla_traffic_manager_t *tm, carla_actor_t *actor, bool enable) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetAutoLaneChange(carla_actor, enable);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].auto_lane_change = enable;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_force_vehicle_lane_change(
    carla_traffic_manager_t *tm, carla_actor_t *actor, bool direction) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetForceLaneChange(carla_actor, direction);

    // Update tracked config
    auto &config = tm->vehicle_configs[carla_actor->GetId()];
    config.force_lane_change_direction = direction;
    config.force_lane_change_active = true;

    if (direction) {
      tm->stats.lane_change_count++;
    }

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_set_vehicle_keep_right_percentage(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetKeepRightPercentage(carla_actor, percentage);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].keep_right_percentage =
        percentage;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_set_vehicle_random_left_lane_change_percentage(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetRandomLeftLaneChangePercentage(carla_actor, percentage);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()]
        .random_left_lane_change_percentage = percentage;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_set_vehicle_random_right_lane_change_percentage(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetRandomRightLaneChangePercentage(carla_actor, percentage);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()]
        .random_right_lane_change_percentage = percentage;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Per-vehicle following behavior

carla_error_t carla_traffic_manager_set_vehicle_distance_to_leading_vehicle(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float distance) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetDistanceToLeadingVehicle(carla_actor, distance);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].distance_to_leading_vehicle =
        distance;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Traffic rule compliance

carla_error_t carla_traffic_manager_set_vehicle_percentage_running_light(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetPercentageRunningLight(carla_actor, percentage);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].percentage_running_light =
        percentage;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_set_vehicle_percentage_running_sign(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetPercentageRunningSign(carla_actor, percentage);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].percentage_running_sign =
        percentage;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_set_vehicle_percentage_ignore_walkers(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetPercentageIgnoreWalkers(carla_actor, percentage);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].percentage_ignore_walkers =
        percentage;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_traffic_manager_set_vehicle_percentage_ignore_vehicles(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetPercentageIgnoreVehicles(carla_actor, percentage);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].percentage_ignore_vehicles =
        percentage;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Collision detection and safety

carla_error_t carla_traffic_manager_set_collision_detection(
    carla_traffic_manager_t *tm, carla_actor_t *reference_actor,
    carla_actor_t *other_actor, bool detect_collision) {
  if (!tm || !reference_actor || !other_actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_ref_actor = get_carla_actor(reference_actor);
    auto carla_other_actor = get_carla_actor(other_actor);
    tm->tm->SetCollisionDetection(carla_ref_actor, carla_other_actor,
                                  detect_collision);

    // Update tracked config for reference actor
    tm->vehicle_configs[carla_ref_actor->GetId()].collision_detection_enabled =
        detect_collision;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Vehicle lights and visual behavior

carla_error_t carla_traffic_manager_set_vehicle_update_lights(
    carla_traffic_manager_t *tm, carla_actor_t *actor, bool update_lights) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->SetUpdateVehicleLights(carla_actor, update_lights);

    // Update tracked config
    tm->vehicle_configs[carla_actor->GetId()].update_vehicle_lights =
        update_lights;

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Utility functions

carla_traffic_manager_config_t
carla_traffic_manager_create_default_config(uint16_t port) {
  carla_traffic_manager_config_t config = {};

  config.global_speed_percentage_difference = 0.0f;
  config.global_lane_offset = 0.0f;
  config.global_distance_to_leading_vehicle = 5.0f;
  config.synchronous_mode = false;
  config.synchronous_mode_timeout_ms = CARLA_TM_DEFAULT_SYNC_TIMEOUT_MS;
  config.hybrid_physics_mode = false;
  config.hybrid_physics_radius = 50.0f;
  config.respawn_dormant_vehicles = false;
  config.respawn_lower_bound = -30.0f;
  config.respawn_upper_bound = 30.0f;
  config.random_device_seed = 0;
  config.osm_mode = false;
  config.port = port;

  return config;
}

carla_traffic_manager_vehicle_config_t
carla_traffic_manager_create_default_vehicle_config(void) {
  carla_traffic_manager_vehicle_config_t config = {};

  config.speed_percentage_difference = 0.0f;
  config.desired_speed = 0.0f;
  config.lane_offset = 0.0f;
  config.distance_to_leading_vehicle = 5.0f;
  config.auto_lane_change = true;
  config.force_lane_change_direction = false;
  config.force_lane_change_active = false;
  config.keep_right_percentage = 0.0f;
  config.random_left_lane_change_percentage = 0.0f;
  config.random_right_lane_change_percentage = 0.0f;
  config.percentage_running_light = 0.0f;
  config.percentage_running_sign = 0.0f;
  config.percentage_ignore_walkers = 0.0f;
  config.percentage_ignore_vehicles = 0.0f;
  config.update_vehicle_lights = true;
  config.collision_detection_enabled = true;

  return config;
}

// String conversion functions

const char *carla_road_option_to_string(carla_road_option_t road_option) {
  switch (road_option) {
  case CARLA_ROAD_OPTION_VOID:
    return "Void";
  case CARLA_ROAD_OPTION_LEFT:
    return "Left";
  case CARLA_ROAD_OPTION_RIGHT:
    return "Right";
  case CARLA_ROAD_OPTION_STRAIGHT:
    return "Straight";
  case CARLA_ROAD_OPTION_LANE_FOLLOW:
    return "LaneFollow";
  case CARLA_ROAD_OPTION_CHANGE_LANE_LEFT:
    return "ChangeLaneLeft";
  case CARLA_ROAD_OPTION_CHANGE_LANE_RIGHT:
    return "ChangeLaneRight";
  case CARLA_ROAD_OPTION_ROAD_END:
    return "RoadEnd";
  default:
    return "Unknown";
  }
}

carla_road_option_t carla_road_option_from_string(const char *road_option_str) {
  if (!road_option_str)
    return CARLA_ROAD_OPTION_VOID;

  if (strcmp(road_option_str, "Void") == 0)
    return CARLA_ROAD_OPTION_VOID;
  if (strcmp(road_option_str, "Left") == 0)
    return CARLA_ROAD_OPTION_LEFT;
  if (strcmp(road_option_str, "Right") == 0)
    return CARLA_ROAD_OPTION_RIGHT;
  if (strcmp(road_option_str, "Straight") == 0)
    return CARLA_ROAD_OPTION_STRAIGHT;
  if (strcmp(road_option_str, "LaneFollow") == 0)
    return CARLA_ROAD_OPTION_LANE_FOLLOW;
  if (strcmp(road_option_str, "ChangeLaneLeft") == 0)
    return CARLA_ROAD_OPTION_CHANGE_LANE_LEFT;
  if (strcmp(road_option_str, "ChangeLaneRight") == 0)
    return CARLA_ROAD_OPTION_CHANGE_LANE_RIGHT;
  if (strcmp(road_option_str, "RoadEnd") == 0)
    return CARLA_ROAD_OPTION_ROAD_END;

  return CARLA_ROAD_OPTION_VOID;
}

// Stub implementations for complex functions that would require full
// implementation

carla_error_t carla_traffic_manager_set_vehicle_custom_path(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_path_t *path) {
  if (!tm || !actor || !path) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);

    // Convert path to CARLA format
    std::vector<carla::geom::Location> carla_path;
    carla_path.reserve(path->location_count);

    for (size_t i = 0; i < path->location_count; ++i) {
      carla_path.push_back(convert_location(path->locations[i]));
    }

    tm->tm->SetCustomPath(carla_actor, carla_path, path->empty_buffer);

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_remove_vehicle_custom_path(carla_traffic_manager_t *tm,
                                                 carla_actor_t *actor) {
  if (!tm || !actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = get_carla_actor(actor);
    tm->tm->RemoveUploadPath(carla_actor->GetId(), true);

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Placeholder implementations for functions requiring more complex integration

carla_error_t
carla_traffic_manager_set_hybrid_physics_mode(carla_traffic_manager_t *tm,
                                              bool mode) {
  if (!tm)
    return CARLA_ERROR_INVALID_ARGUMENT;

  try {
    tm->tm->SetHybridPhysicsMode(mode);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_set_hybrid_physics_radius(carla_traffic_manager_t *tm,
                                                float radius) {
  if (!tm)
    return CARLA_ERROR_INVALID_ARGUMENT;

  try {
    tm->tm->SetHybridPhysicsRadius(radius);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_set_respawn_dormant_vehicles(carla_traffic_manager_t *tm,
                                                   bool enable) {
  if (!tm)
    return CARLA_ERROR_INVALID_ARGUMENT;

  try {
    tm->tm->SetRespawnDormantVehicles(enable);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_traffic_manager_get_stats(carla_traffic_manager_t *tm,
                                carla_traffic_manager_stats_t *stats) {
  if (!tm || !stats) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  *stats = tm->stats;
  return CARLA_ERROR_NONE;
}

carla_error_t carla_traffic_manager_reset_stats(carla_traffic_manager_t *tm) {
  if (!tm) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  tm->stats = {};
  return CARLA_ERROR_NONE;
}

bool carla_traffic_manager_is_vehicle_registered(carla_traffic_manager_t *tm,
                                                 carla_actor_t *actor) {
  if (!tm || !actor) {
    return false;
  }

  auto carla_actor = get_carla_actor(actor);
  return tm->vehicle_configs.find(carla_actor->GetId()) !=
         tm->vehicle_configs.end();
}

// Memory management stubs

void carla_traffic_manager_path_destroy(carla_traffic_manager_path_t *path) {
  if (!path)
    return;
  delete[] path->locations;
  path->locations = nullptr;
  path->location_count = 0;
}

void carla_traffic_manager_route_destroy(carla_traffic_manager_route_t *route) {
  if (!route)
    return;
  delete[] route->road_options;
  route->road_options = nullptr;
  route->option_count = 0;
}

void carla_traffic_manager_action_buffer_destroy(
    carla_traffic_manager_action_buffer_t *buffer) {
  if (!buffer)
    return;
  delete[] buffer->actions;
  buffer->actions = nullptr;
  buffer->action_count = 0;
  buffer->capacity = 0;
}

void carla_traffic_manager_free_actor_array(carla_actor_t **actors) {
  delete[] actors;
}

// Placeholder implementations for functions requiring significant additional
// work
carla_error_t carla_traffic_manager_update_vehicle_custom_path(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_path_t *path) {
  (void)tm;
  (void)actor;
  (void)path;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_traffic_manager_set_vehicle_imported_route(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_route_t *route) {
  (void)tm;
  (void)actor;
  (void)route;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t
carla_traffic_manager_remove_vehicle_imported_route(carla_traffic_manager_t *tm,
                                                    carla_actor_t *actor) {
  (void)tm;
  (void)actor;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_traffic_manager_update_vehicle_imported_route(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_route_t *route) {
  (void)tm;
  (void)actor;
  (void)route;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_traffic_manager_get_vehicle_next_action(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    carla_traffic_manager_action_t *action) {
  (void)tm;
  (void)actor;
  (void)action;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_traffic_manager_get_vehicle_action_buffer(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    carla_traffic_manager_action_buffer_t *buffer) {
  (void)tm;
  (void)actor;
  (void)buffer;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_traffic_manager_set_respawn_boundaries(
    carla_traffic_manager_t *tm, float lower_bound, float upper_bound) {
  (void)tm;
  (void)lower_bound;
  (void)upper_bound;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t
carla_traffic_manager_set_max_boundaries(carla_traffic_manager_t *tm,
                                         float lower, float upper) {
  (void)tm;
  (void)lower;
  (void)upper;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t
carla_traffic_manager_get_config(carla_traffic_manager_t *tm,
                                 carla_traffic_manager_config_t *config) {
  (void)tm;
  (void)config;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t
carla_traffic_manager_set_config(carla_traffic_manager_t *tm,
                                 const carla_traffic_manager_config_t *config) {
  (void)tm;
  (void)config;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_traffic_manager_get_vehicle_config(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    carla_traffic_manager_vehicle_config_t *config) {
  (void)tm;
  (void)actor;
  (void)config;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_traffic_manager_set_vehicle_config(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_vehicle_config_t *config) {
  (void)tm;
  (void)actor;
  (void)config;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t carla_traffic_manager_get_registered_vehicles(
    carla_traffic_manager_t *tm, carla_actor_t ***actors, size_t *actor_count) {
  (void)tm;
  (void)actors;
  (void)actor_count;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t
carla_traffic_manager_create_path(const carla_vector3d_t *locations,
                                  size_t location_count, bool empty_buffer,
                                  carla_traffic_manager_path_t *path) {
  (void)locations;
  (void)location_count;
  (void)empty_buffer;
  (void)path;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}

carla_error_t
carla_traffic_manager_create_route(const carla_road_option_t *road_options,
                                   size_t option_count, bool empty_buffer,
                                   carla_traffic_manager_route_t *route) {
  (void)road_options;
  (void)option_count;
  (void)empty_buffer;
  (void)route;
  return CARLA_ERROR_NOT_FOUND; // Not implemented
}
