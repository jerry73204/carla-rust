#include "carla_c/sensor.h"
#include <memory>
#include <mutex>
#include <unordered_map>

#include "carla/client/Sensor.h"
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
#include "internal.h"

namespace {
// Callback storage structure
struct CallbackData {
  carla_sensor_callback_t callback;
  void *user_data;
  std::shared_ptr<carla::client::Sensor> sensor;
};

// Global storage for active callbacks
std::unordered_map<uintptr_t, std::unique_ptr<CallbackData>> g_active_callbacks;
std::mutex g_callback_mutex;

// Helper to get Sensor from Actor (using carla_actor_t since sensor is a
// typedef of carla_actor_t)
std::shared_ptr<carla::client::Sensor> GetSensor(carla_actor_t *actor) {
  if (!actor || !actor->actor) {
    return nullptr;
  }
  return std::dynamic_pointer_cast<carla::client::Sensor>(actor->actor);
}

std::shared_ptr<const carla::client::Sensor>
GetSensor(const carla_actor_t *actor) {
  if (!actor || !actor->actor) {
    return nullptr;
  }
  return std::dynamic_pointer_cast<const carla::client::Sensor>(actor->actor);
}

// Type identification functions moved outside anonymous namespace to be
// accessible from other files

// Convert transform from CARLA to C
carla_transform_t ConvertTransform(const carla::geom::Transform &transform) {
  carla_transform_t result;
  result.location.x = transform.location.x;
  result.location.y = transform.location.y;
  result.location.z = transform.location.z;
  result.rotation.pitch = transform.rotation.pitch;
  result.rotation.yaw = transform.rotation.yaw;
  result.rotation.roll = transform.rotation.roll;
  return result;
}

// Convert Vector3D from CARLA to C
carla_vector3d_t ConvertVector3D(const carla::geom::Vector3D &vector) {
  carla_vector3d_t result;
  result.x = vector.x;
  result.y = vector.y;
  result.z = vector.z;
  return result;
}
} // namespace

// Type identification function is now implemented in common.cpp to avoid boost
// conflicts

// Sensor data structure definition moved to internal.h

extern "C" {

// Sensor lifecycle and callback management
carla_error_t carla_sensor_listen(carla_sensor_t *sensor,
                                  carla_sensor_callback_t callback,
                                  void *user_data) {
  if (!sensor || !callback) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto cpp_sensor = GetSensor(sensor);
    if (!cpp_sensor) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    // Create callback data
    auto callback_data = std::make_unique<CallbackData>();
    callback_data->callback = callback;
    callback_data->user_data = user_data;
    callback_data->sensor = cpp_sensor;

    uintptr_t sensor_id = reinterpret_cast<uintptr_t>(sensor);

    // Create C++ callback wrapper
    auto cpp_callback = [callback, user_data](
                            carla::SharedPtr<carla::sensor::SensorData> data) {
      // Create C sensor data wrapper
      auto *c_data = new carla_sensor_data(data);

      // Call the C callback
      callback(c_data, user_data);

      // Note: C code is responsible for calling carla_sensor_data_destroy()
      // to free the sensor data when done with it
    };

    // Register the callback with the sensor
    cpp_sensor->Listen(cpp_callback);

    // Store callback data for cleanup
    {
      std::lock_guard<std::mutex> lock(g_callback_mutex);
      g_active_callbacks[sensor_id] = std::move(callback_data);
    }

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_sensor_stop(carla_sensor_t *sensor) {
  if (!sensor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto cpp_sensor = GetSensor(sensor);
    if (!cpp_sensor) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    // Stop the sensor
    cpp_sensor->Stop();

    // Remove callback data
    uintptr_t sensor_id = reinterpret_cast<uintptr_t>(sensor);
    {
      std::lock_guard<std::mutex> lock(g_callback_mutex);
      g_active_callbacks.erase(sensor_id);
    }

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

bool carla_sensor_is_listening(const carla_sensor_t *sensor) {
  if (!sensor) {
    return false;
  }

  try {
    auto cpp_sensor = GetSensor(sensor);
    if (cpp_sensor) {
      return cpp_sensor->IsListening();
    }
  } catch (const std::exception &) {
    // Return false on error
  }

  return false;
}

// Sensor data introspection
carla_sensor_data_type_t
carla_sensor_data_get_type(const carla_sensor_data_t *data) {
  if (!data) {
    return CARLA_SENSOR_DATA_UNKNOWN;
  }
  return data->type;
}

carla_sensor_data_info_t
carla_sensor_data_get_info(const carla_sensor_data_t *data) {
  carla_sensor_data_info_t info = {0};

  if (!data || !data->cpp_data) {
    return info;
  }

  try {
    info.frame = data->cpp_data->GetFrame();
    info.timestamp = data->cpp_data->GetTimestamp(); // Already in seconds
    info.sensor_transform =
        ConvertTransform(data->cpp_data->GetSensorTransform());
  } catch (const std::exception &) {
    // Return zero-initialized info on error
  }

  return info;
}

// Common sensor data access
uint64_t carla_sensor_data_get_frame(const carla_sensor_data_t *data) {
  if (!data || !data->cpp_data) {
    return 0;
  }

  try {
    return data->cpp_data->GetFrame();
  } catch (const std::exception &) {
    return 0;
  }
}

double carla_sensor_data_get_timestamp(const carla_sensor_data_t *data) {
  if (!data || !data->cpp_data) {
    return 0.0;
  }

  try {
    return data->cpp_data->GetTimestamp(); // Already in seconds
  } catch (const std::exception &) {
    return 0.0;
  }
}

carla_transform_t
carla_sensor_data_get_transform(const carla_sensor_data_t *data) {
  carla_transform_t transform = {0};

  if (!data || !data->cpp_data) {
    return transform;
  }

  try {
    return ConvertTransform(data->cpp_data->GetSensorTransform());
  } catch (const std::exception &) {
    return transform;
  }
}

// Image data access
carla_image_data_t
carla_sensor_data_get_image(const carla_sensor_data_t *data) {
  carla_image_data_t image_data = {0};

  if (!data || !data->cpp_data || data->type != CARLA_SENSOR_DATA_IMAGE) {
    return image_data;
  }

  if (!data->image_cached) {
    try {
      auto image = std::static_pointer_cast<const carla::sensor::data::Image>(
          data->cpp_data);
      data->image_data.width = image->GetWidth();
      data->image_data.height = image->GetHeight();
      data->image_data.fov = image->GetFOVAngle();
      data->image_data.raw_data =
          reinterpret_cast<const uint8_t *>(image->data());
      data->image_data.raw_data_size =
          image->size() * sizeof(carla::sensor::data::Color);
      data->image_cached = true;
    } catch (const std::exception &) {
      // Return zero-initialized data on error
    }
  }

  return data->image_data;
}

uint32_t carla_image_data_get_width(const carla_sensor_data_t *data) {
  auto image_data = carla_sensor_data_get_image(data);
  return image_data.width;
}

uint32_t carla_image_data_get_height(const carla_sensor_data_t *data) {
  auto image_data = carla_sensor_data_get_image(data);
  return image_data.height;
}

uint32_t carla_image_data_get_fov(const carla_sensor_data_t *data) {
  auto image_data = carla_sensor_data_get_image(data);
  return image_data.fov;
}

const uint8_t *carla_image_data_get_raw_data(const carla_sensor_data_t *data) {
  auto image_data = carla_sensor_data_get_image(data);
  return image_data.raw_data;
}

size_t carla_image_data_get_raw_data_size(const carla_sensor_data_t *data) {
  auto image_data = carla_sensor_data_get_image(data);
  return image_data.raw_data_size;
}

carla_error_t carla_image_data_copy_to_buffer(const carla_sensor_data_t *data,
                                              uint8_t *buffer,
                                              size_t buffer_size) {
  if (!data || !buffer || buffer_size == 0) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  auto image_data = carla_sensor_data_get_image(data);
  if (!image_data.raw_data || image_data.raw_data_size == 0) {
    return CARLA_ERROR_NOT_FOUND;
  }

  if (buffer_size < image_data.raw_data_size) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    std::memcpy(buffer, image_data.raw_data, image_data.raw_data_size);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// LiDAR data access
carla_lidar_data_t
carla_sensor_data_get_lidar(const carla_sensor_data_t *data) {
  carla_lidar_data_t lidar_data = {0};

  if (!data || !data->cpp_data || data->type != CARLA_SENSOR_DATA_LIDAR) {
    return lidar_data;
  }

  if (!data->lidar_cached) {
    try {
      auto lidar =
          std::static_pointer_cast<const carla::sensor::data::LidarMeasurement>(
              data->cpp_data);
      data->lidar_data.points =
          reinterpret_cast<const carla_lidar_detection_t *>(lidar->data());
      data->lidar_data.point_count = lidar->size();
      data->lidar_data.horizontal_angle = lidar->GetHorizontalAngle();
      data->lidar_data.channels = lidar->GetChannelCount();
      data->lidar_cached = true;
    } catch (const std::exception &) {
      // Return zero-initialized data on error
    }
  }

  return data->lidar_data;
}

size_t carla_lidar_data_get_point_count(const carla_sensor_data_t *data) {
  auto lidar_data = carla_sensor_data_get_lidar(data);
  return lidar_data.point_count;
}

const carla_lidar_detection_t *
carla_lidar_data_get_points(const carla_sensor_data_t *data) {
  auto lidar_data = carla_sensor_data_get_lidar(data);
  return lidar_data.points;
}

uint32_t
carla_lidar_data_get_horizontal_angle(const carla_sensor_data_t *data) {
  auto lidar_data = carla_sensor_data_get_lidar(data);
  return lidar_data.horizontal_angle;
}

uint32_t carla_lidar_data_get_channels(const carla_sensor_data_t *data) {
  auto lidar_data = carla_sensor_data_get_lidar(data);
  return lidar_data.channels;
}

carla_error_t
carla_lidar_data_copy_points_to_buffer(const carla_sensor_data_t *data,
                                       carla_lidar_detection_t *buffer,
                                       size_t buffer_size) {
  if (!data || !buffer || buffer_size == 0) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  auto lidar_data = carla_sensor_data_get_lidar(data);
  if (!lidar_data.points || lidar_data.point_count == 0) {
    return CARLA_ERROR_NOT_FOUND;
  }

  size_t required_size =
      lidar_data.point_count * sizeof(carla_lidar_detection_t);
  if (buffer_size < required_size) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    std::memcpy(buffer, lidar_data.points, required_size);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Utility functions
bool carla_actor_is_sensor(const carla_actor_t *actor) {
  if (!actor || !actor->actor) {
    return false;
  }

  try {
    auto sensor =
        std::dynamic_pointer_cast<const carla::client::Sensor>(actor->actor);
    return sensor != nullptr;
  } catch (const std::exception &) {
    return false;
  }
}

carla_sensor_t *carla_actor_as_sensor(carla_actor_t *actor) {
  if (!carla_actor_is_sensor(actor)) {
    return nullptr;
  }

  // Since carla_sensor_t is just a typedef of carla_actor_t,
  // we can safely cast if it's a sensor
  return reinterpret_cast<carla_sensor_t *>(actor);
}

// IMU data access
carla_imu_data_t carla_sensor_data_get_imu(const carla_sensor_data_t *data) {
  carla_imu_data_t imu_data = {0};

  if (!data || !data->cpp_data || data->type != CARLA_SENSOR_DATA_IMU) {
    return imu_data;
  }

  if (!data->imu_cached) {
    try {
      auto imu =
          std::static_pointer_cast<const carla::sensor::data::IMUMeasurement>(
              data->cpp_data);
      data->imu_data.accelerometer = ConvertVector3D(imu->GetAccelerometer());
      data->imu_data.gyroscope = ConvertVector3D(imu->GetGyroscope());
      data->imu_data.compass = imu->GetCompass();
      data->imu_cached = true;
    } catch (const std::exception &) {
      // Return zero-initialized data on error
    }
  }

  return data->imu_data;
}

carla_vector3d_t
carla_imu_data_get_accelerometer(const carla_sensor_data_t *data) {
  auto imu_data = carla_sensor_data_get_imu(data);
  return imu_data.accelerometer;
}

carla_vector3d_t carla_imu_data_get_gyroscope(const carla_sensor_data_t *data) {
  auto imu_data = carla_sensor_data_get_imu(data);
  return imu_data.gyroscope;
}

float carla_imu_data_get_compass(const carla_sensor_data_t *data) {
  auto imu_data = carla_sensor_data_get_imu(data);
  return imu_data.compass;
}

// GNSS data access
carla_gnss_data_t carla_sensor_data_get_gnss(const carla_sensor_data_t *data) {
  carla_gnss_data_t gnss_data = {0};

  if (!data || !data->cpp_data || data->type != CARLA_SENSOR_DATA_GNSS) {
    return gnss_data;
  }

  if (!data->gnss_cached) {
    try {
      auto gnss =
          std::static_pointer_cast<const carla::sensor::data::GnssMeasurement>(
              data->cpp_data);
      data->gnss_data.latitude = gnss->GetLatitude();
      data->gnss_data.longitude = gnss->GetLongitude();
      data->gnss_data.altitude = gnss->GetAltitude();
      data->gnss_cached = true;
    } catch (const std::exception &) {
      // Return zero-initialized data on error
    }
  }

  return data->gnss_data;
}

double carla_gnss_data_get_latitude(const carla_sensor_data_t *data) {
  auto gnss_data = carla_sensor_data_get_gnss(data);
  return gnss_data.latitude;
}

double carla_gnss_data_get_longitude(const carla_sensor_data_t *data) {
  auto gnss_data = carla_sensor_data_get_gnss(data);
  return gnss_data.longitude;
}

double carla_gnss_data_get_altitude(const carla_sensor_data_t *data) {
  auto gnss_data = carla_sensor_data_get_gnss(data);
  return gnss_data.altitude;
}

// Collision event data access
carla_collision_data_t
carla_sensor_data_get_collision(const carla_sensor_data_t *data) {
  carla_collision_data_t collision_data = {0};

  if (!data || !data->cpp_data || data->type != CARLA_SENSOR_DATA_COLLISION) {
    return collision_data;
  }

  if (!data->collision_cached) {
    try {
      auto collision =
          std::static_pointer_cast<const carla::sensor::data::CollisionEvent>(
              data->cpp_data);

      // Create actor wrappers - note: these create new actor handles
      // The caller is responsible for managing their lifetime
      if (collision->GetActor()) {
        auto *actor_wrapper = new carla_actor_t;
        actor_wrapper->actor = collision->GetActor();
        data->collision_data.actor = actor_wrapper;
      }

      if (collision->GetOtherActor()) {
        auto *other_actor_wrapper = new carla_actor_t;
        other_actor_wrapper->actor = collision->GetOtherActor();
        data->collision_data.other_actor = other_actor_wrapper;
      }

      data->collision_data.normal_impulse =
          ConvertVector3D(collision->GetNormalImpulse());
      data->collision_cached = true;
    } catch (const std::exception &) {
      // Return zero-initialized data on error
    }
  }

  return data->collision_data;
}

carla_actor_t *carla_collision_data_get_actor(const carla_sensor_data_t *data) {
  auto collision_data = carla_sensor_data_get_collision(data);
  return collision_data.actor;
}

carla_actor_t *
carla_collision_data_get_other_actor(const carla_sensor_data_t *data) {
  auto collision_data = carla_sensor_data_get_collision(data);
  return collision_data.other_actor;
}

carla_vector3d_t
carla_collision_data_get_normal_impulse(const carla_sensor_data_t *data) {
  auto collision_data = carla_sensor_data_get_collision(data);
  return collision_data.normal_impulse;
}

// Stub implementations for remaining sensor types (to be completed)
carla_semantic_lidar_data_t
carla_sensor_data_get_semantic_lidar(const carla_sensor_data_t *data) {
  carla_semantic_lidar_data_t semantic_lidar_data = {0};
  // TODO: Implement semantic LiDAR data access
  return semantic_lidar_data;
}

size_t
carla_semantic_lidar_data_get_point_count(const carla_sensor_data_t *data) {
  return 0; // TODO: Implement
}

const carla_semantic_lidar_detection_t *
carla_semantic_lidar_data_get_points(const carla_sensor_data_t *data) {
  return nullptr; // TODO: Implement
}

carla_radar_data_t
carla_sensor_data_get_radar(const carla_sensor_data_t *data) {
  carla_radar_data_t radar_data = {0};
  // TODO: Implement radar data access
  return radar_data;
}

size_t carla_radar_data_get_detection_count(const carla_sensor_data_t *data) {
  return 0; // TODO: Implement
}

const carla_radar_detection_t *
carla_radar_data_get_detections(const carla_sensor_data_t *data) {
  return nullptr; // TODO: Implement
}

carla_lane_invasion_data_t
carla_sensor_data_get_lane_invasion(const carla_sensor_data_t *data) {
  carla_lane_invasion_data_t lane_invasion_data = {0};
  // TODO: Implement lane invasion data access
  return lane_invasion_data;
}

carla_actor_t *
carla_lane_invasion_data_get_actor(const carla_sensor_data_t *data) {
  return nullptr; // TODO: Implement
}

size_t carla_lane_invasion_data_get_crossed_lane_markings_count(
    const carla_sensor_data_t *data) {
  return 0; // TODO: Implement
}

carla_obstacle_detection_data_t
carla_sensor_data_get_obstacle_detection(const carla_sensor_data_t *data) {
  carla_obstacle_detection_data_t obstacle_detection_data = {0};
  // TODO: Implement obstacle detection data access
  return obstacle_detection_data;
}

carla_actor_t *
carla_obstacle_detection_data_get_actor(const carla_sensor_data_t *data) {
  return nullptr; // TODO: Implement
}

carla_actor_t *
carla_obstacle_detection_data_get_other_actor(const carla_sensor_data_t *data) {
  return nullptr; // TODO: Implement
}

float carla_obstacle_detection_data_get_distance(
    const carla_sensor_data_t *data) {
  return 0.0f; // TODO: Implement
}

carla_dvs_event_array_data_t
carla_sensor_data_get_dvs_event_array(const carla_sensor_data_t *data) {
  carla_dvs_event_array_data_t dvs_event_array_data = {0};

  if (!data || data->type != CARLA_SENSOR_DATA_DVS_EVENT_ARRAY) {
    return dvs_event_array_data;
  }

  // Cache DVS data if not already cached
  if (!data->dvs_event_array_cached) {
    auto dvs_data =
        std::dynamic_pointer_cast<carla::sensor::data::DVSEventArray>(
            data->cpp_data);
    if (dvs_data) {
      data->dvs_event_array_data.events =
          reinterpret_cast<const carla_dvs_event_t *>(dvs_data->data());
      data->dvs_event_array_data.event_count = dvs_data->size();
      data->dvs_event_array_data.width = dvs_data->GetWidth();
      data->dvs_event_array_data.height = dvs_data->GetHeight();
      data->dvs_event_array_data.fov_angle =
          static_cast<uint64_t>(dvs_data->GetFOVAngle());
      data->dvs_event_array_cached = true;
    }
  }

  return data->dvs_event_array_data;
}

size_t
carla_dvs_event_array_data_get_event_count(const carla_sensor_data_t *data) {
  if (!data || data->type != CARLA_SENSOR_DATA_DVS_EVENT_ARRAY) {
    return 0;
  }

  auto dvs_data = std::dynamic_pointer_cast<carla::sensor::data::DVSEventArray>(
      data->cpp_data);
  return dvs_data ? dvs_data->size() : 0;
}

const carla_dvs_event_t *
carla_dvs_event_array_data_get_events(const carla_sensor_data_t *data) {
  if (!data || data->type != CARLA_SENSOR_DATA_DVS_EVENT_ARRAY) {
    return nullptr;
  }

  auto dvs_data = std::dynamic_pointer_cast<carla::sensor::data::DVSEventArray>(
      data->cpp_data);
  if (!dvs_data) {
    return nullptr;
  }

  return reinterpret_cast<const carla_dvs_event_t *>(dvs_data->data());
}

uint32_t carla_dvs_event_array_data_get_width(const carla_sensor_data_t *data) {
  if (!data || data->type != CARLA_SENSOR_DATA_DVS_EVENT_ARRAY) {
    return 0;
  }

  auto dvs_data = std::dynamic_pointer_cast<carla::sensor::data::DVSEventArray>(
      data->cpp_data);
  return dvs_data ? dvs_data->GetWidth() : 0;
}

uint32_t
carla_dvs_event_array_data_get_height(const carla_sensor_data_t *data) {
  if (!data || data->type != CARLA_SENSOR_DATA_DVS_EVENT_ARRAY) {
    return 0;
  }

  auto dvs_data = std::dynamic_pointer_cast<carla::sensor::data::DVSEventArray>(
      data->cpp_data);
  return dvs_data ? dvs_data->GetHeight() : 0;
}

// Optical flow data access functions

carla_optical_flow_data_t
carla_sensor_data_get_optical_flow(const carla_sensor_data_t *data) {
  carla_optical_flow_data_t optical_flow_data = {0};

  if (!data || data->type != CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE) {
    return optical_flow_data;
  }

  // Cache optical flow data if not already cached
  if (!data->optical_flow_cached) {
    auto image =
        std::dynamic_pointer_cast<carla::sensor::data::Image>(data->cpp_data);
    if (image) {
      data->optical_flow_data.width = image->GetWidth();
      data->optical_flow_data.height = image->GetHeight();
      data->optical_flow_data.fov = static_cast<uint32_t>(image->GetFOVAngle());
      data->optical_flow_data.flow_data =
          reinterpret_cast<const carla_optical_flow_pixel_t *>(image->data());
      data->optical_flow_data.flow_data_size =
          image->GetWidth() * image->GetHeight();
      data->optical_flow_cached = true;
    }
  }

  return data->optical_flow_data;
}

uint32_t carla_optical_flow_data_get_width(const carla_sensor_data_t *data) {
  if (!data || data->type != CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE) {
    return 0;
  }

  auto image =
      std::dynamic_pointer_cast<carla::sensor::data::Image>(data->cpp_data);
  return image ? image->GetWidth() : 0;
}

uint32_t carla_optical_flow_data_get_height(const carla_sensor_data_t *data) {
  if (!data || data->type != CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE) {
    return 0;
  }

  auto image =
      std::dynamic_pointer_cast<carla::sensor::data::Image>(data->cpp_data);
  return image ? image->GetHeight() : 0;
}

const carla_optical_flow_pixel_t *
carla_optical_flow_data_get_pixels(const carla_sensor_data_t *data) {
  if (!data || data->type != CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE) {
    return nullptr;
  }

  auto image =
      std::dynamic_pointer_cast<carla::sensor::data::Image>(data->cpp_data);
  if (!image) {
    return nullptr;
  }

  return reinterpret_cast<const carla_optical_flow_pixel_t *>(image->data());
}

// Sensor attribute management
carla_error_t carla_sensor_set_attribute(carla_sensor_t *sensor,
                                         const char *key, const char *value) {
  if (!sensor || !key || !value) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto cpp_sensor = GetSensor(sensor);
    if (!cpp_sensor) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    // Note: GetActorDescription() is protected in CARLA 0.10.0
    // We cannot modify sensor attributes after creation in this version
    // This functionality was available in older versions but is no longer
    // accessible
    return CARLA_ERROR_NOT_FOUND;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

const char *carla_sensor_get_attribute(const carla_sensor_t *sensor,
                                       const char *key) {
  if (!sensor || !key) {
    return nullptr;
  }

  try {
    auto cpp_sensor = GetSensor(sensor);
    if (!cpp_sensor) {
      return nullptr;
    }

    // Note: GetActorDescription() is protected in CARLA 0.10.0
    // We cannot access sensor attributes after creation in this version
    // This functionality was available in older versions but is no longer
    // accessible
    return nullptr;
  } catch (const std::exception &) {
    return nullptr;
  }
}

void *carla_sensor_get_calibration_data(const carla_sensor_t *sensor) {
  if (!sensor) {
    return nullptr;
  }

  try {
    auto cpp_sensor = GetSensor(sensor);
    if (!cpp_sensor) {
      return nullptr;
    }

    // Check if this is a camera sensor that supports calibration
    auto camera_sensor =
        std::dynamic_pointer_cast<const carla::client::Sensor>(cpp_sensor);
    if (!camera_sensor) {
      return nullptr;
    }

    // For now, return a placeholder indicating calibration data is not yet
    // implemented
    // TODO: Implement proper calibration data extraction when CARLA 0.10.0 API
    // is available
    return nullptr;
  } catch (const std::exception &) {
    return nullptr;
  }
}

void carla_sensor_data_destroy(carla_sensor_data_t *data) {
  if (data) {
    // Clean up any allocated actor wrappers
    if (data->collision_cached) {
      delete data->collision_data.actor;
      delete data->collision_data.other_actor;
    }

    delete data;
  }
}

} // extern "C"
