#ifndef CARLA_C_INTERNAL_H
#define CARLA_C_INTERNAL_H

#include "carla/client/Actor.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/ActorList.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Client.h"
#include "carla/client/Map.h"
#include "carla/client/World.h"
#include "carla/sensor/SensorData.h"
#include "carla_c/types.h"
#include <memory>
#include <vector>

// Internal structure definitions shared across all C wrapper files

// Client structure
struct carla_client {
  std::unique_ptr<carla::client::Client> client;
  std::shared_ptr<carla::client::World> current_world;
};

// World structure
struct carla_world {
  std::shared_ptr<carla::client::World> world;
};

// Map structure
struct carla_map {
  std::shared_ptr<carla::client::Map> map;
};

// Actor structure
struct carla_actor {
  std::shared_ptr<carla::client::Actor> actor;
};

// Actor list structure
struct carla_actor_list {
  carla::SharedPtr<carla::client::ActorList> list;
};

// Blueprint library structure
struct carla_blueprint_library {
  std::shared_ptr<carla::client::BlueprintLibrary> library;
};

// Actor blueprint structure (defined per-file as needed)

// Transform list structure
struct carla_transform_list {
  std::vector<carla::geom::Transform> transforms;
};

// String list structure
struct carla_string_list {
  char **strings;
  size_t count;
};

// Forward declaration for sensor data type enumeration function
carla_sensor_data_type_t
IdentifySensorDataType(const carla::sensor::SensorData &data);

// Sensor data structure that wraps the C++ sensor data
struct carla_sensor_data {
  carla::SharedPtr<carla::sensor::SensorData> cpp_data;
  carla_sensor_data_type_t type;

  // Cached data for C access
  mutable carla_image_data_t image_data;
  mutable carla_lidar_data_t lidar_data;
  mutable carla_semantic_lidar_data_t semantic_lidar_data;
  mutable carla_radar_data_t radar_data;
  mutable carla_imu_data_t imu_data;
  mutable carla_gnss_data_t gnss_data;
  mutable carla_collision_data_t collision_data;
  mutable carla_lane_invasion_data_t lane_invasion_data;
  mutable carla_obstacle_detection_data_t obstacle_detection_data;
  mutable carla_dvs_event_array_data_t dvs_event_array_data;

  // Flags to track what data has been cached
  mutable bool image_cached = false;
  mutable bool lidar_cached = false;
  mutable bool semantic_lidar_cached = false;
  mutable bool radar_cached = false;
  mutable bool imu_cached = false;
  mutable bool gnss_cached = false;
  mutable bool collision_cached = false;
  mutable bool lane_invasion_cached = false;
  mutable bool obstacle_detection_cached = false;
  mutable bool dvs_event_array_cached = false;

  explicit carla_sensor_data(carla::SharedPtr<carla::sensor::SensorData> data)
      : cpp_data(std::move(data)), type(IdentifySensorDataType(*cpp_data)) {}
};

#endif // CARLA_C_INTERNAL_H
