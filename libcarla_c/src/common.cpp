#include "carla/sensor/data/CollisionEvent.h"
#include "carla/sensor/data/DVSEventArray.h"
#include "carla/sensor/data/GnssMeasurement.h"
#include "carla/sensor/data/IMUMeasurement.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/data/LaneInvasionEvent.h"
#include "carla/sensor/data/LidarMeasurement.h"
#include "carla/sensor/data/ObstacleDetectionEvent.h"
#include "carla/sensor/data/RadarMeasurement.h"
#include "carla/sensor/data/SemanticLidarMeasurement.h"
#include "carla_c/types.h"
#include "internal.h"
#include <cstdlib>
#include <cstring>
#include <typeinfo>

const char *carla_error_to_string(carla_error_t error) {
  switch (error) {
  case CARLA_ERROR_NONE:
    return "No error";
  case CARLA_ERROR_CONNECTION_FAILED:
    return "Connection failed";
  case CARLA_ERROR_TIMEOUT:
    return "Operation timed out";
  case CARLA_ERROR_INVALID_ARGUMENT:
    return "Invalid argument";
  case CARLA_ERROR_NOT_FOUND:
    return "Not found";
  case CARLA_ERROR_UNKNOWN:
  default:
    return "Unknown error";
  }
}

void carla_free_string(char *str) {
  if (str) {
    free(str);
  }
}

// Sensor data type identification function
carla_sensor_data_type_t
IdentifySensorDataType(const carla::sensor::SensorData &data) {
  // Use typeid to identify the concrete type
  const std::type_info &type = typeid(data);

  if (type == typeid(carla::sensor::data::Image)) {
    // For now, we'll identify optical flow at runtime in the accessor functions
    // since we can't easily distinguish the image type here without metadata
    return CARLA_SENSOR_DATA_IMAGE;
  } else if (type == typeid(carla::sensor::data::LidarMeasurement)) {
    return CARLA_SENSOR_DATA_LIDAR;
  } else if (type == typeid(carla::sensor::data::SemanticLidarMeasurement)) {
    return CARLA_SENSOR_DATA_SEMANTIC_LIDAR;
  } else if (type == typeid(carla::sensor::data::RadarMeasurement)) {
    return CARLA_SENSOR_DATA_RADAR;
  } else if (type == typeid(carla::sensor::data::IMUMeasurement)) {
    return CARLA_SENSOR_DATA_IMU;
  } else if (type == typeid(carla::sensor::data::GnssMeasurement)) {
    return CARLA_SENSOR_DATA_GNSS;
  } else if (type == typeid(carla::sensor::data::CollisionEvent)) {
    return CARLA_SENSOR_DATA_COLLISION;
  } else if (type == typeid(carla::sensor::data::LaneInvasionEvent)) {
    return CARLA_SENSOR_DATA_LANE_INVASION;
  } else if (type == typeid(carla::sensor::data::ObstacleDetectionEvent)) {
    return CARLA_SENSOR_DATA_OBSTACLE_DETECTION;
  } else if (type == typeid(carla::sensor::data::DVSEventArray)) {
    return CARLA_SENSOR_DATA_DVS_EVENT_ARRAY;
  }

  return CARLA_SENSOR_DATA_UNKNOWN;
}

// String list implementation (using definition from internal.h)

void carla_free_string_list(carla_string_list_t *list) {
  if (list) {
    for (size_t i = 0; i < list->count; ++i) {
      free(list->strings[i]);
    }
    free(list->strings);
    delete list;
  }
}
