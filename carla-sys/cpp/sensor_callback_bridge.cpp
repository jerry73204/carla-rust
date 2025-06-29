// Enhanced sensor callback implementation
#include "sensor_callback_bridge.h"
#include <carla/sensor/data/CollisionEvent.h>
#include <carla/sensor/data/DVSEventArray.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LaneInvasionEvent.h>
#include <carla/sensor/data/LidarData.h>
#include <carla/sensor/data/ObstacleDetectionEvent.h>
#include <carla/sensor/data/RadarData.h>
#include <carla/sensor/data/SemanticLidarData.h>
#include <cstring>

namespace carla_sys {

uint64_t SensorCallbackManager::registerCallback(uint32_t sensor_id,
                                                 SensorDataCallback callback,
                                                 uint8_t *user_data) {
  std::lock_guard<std::mutex> lock(mutex_);

  uint64_t handle = next_handle_++;

  CallbackInfo info;
  info.callback = callback;
  info.user_data = user_data;
  info.handle = handle;

  callbacks_[sensor_id].push_back(info);
  handle_to_sensor_[handle] = sensor_id;

  return handle;
}

bool SensorCallbackManager::unregisterCallback(uint64_t handle) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = handle_to_sensor_.find(handle);
  if (it == handle_to_sensor_.end()) {
    return false;
  }

  uint32_t sensor_id = it->second;
  handle_to_sensor_.erase(it);

  auto &sensor_callbacks = callbacks_[sensor_id];
  sensor_callbacks.erase(std::remove_if(sensor_callbacks.begin(),
                                        sensor_callbacks.end(),
                                        [handle](const CallbackInfo &info) {
                                          return info.handle == handle;
                                        }),
                         sensor_callbacks.end());

  if (sensor_callbacks.empty()) {
    callbacks_.erase(sensor_id);
  }

  return true;
}

void SensorCallbackManager::processSensorData(
    uint32_t sensor_id, std::shared_ptr<carla::sensor::SensorData> data) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = callbacks_.find(sensor_id);
  if (it == callbacks_.end()) {
    return;
  }

  // Serialize the data once
  SensorDataType data_type;
  std::vector<uint8_t> serialized = serializeSensorData(data, data_type);

  // Call all registered callbacks
  for (const auto &callback_info : it->second) {
    callback_info.callback(sensor_id, serialized.data(), serialized.size(),
                           callback_info.user_data);
  }
}

void SensorCallbackManager::cleanupSensor(uint32_t sensor_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = callbacks_.find(sensor_id);
  if (it != callbacks_.end()) {
    // Remove all handles for this sensor
    for (const auto &callback_info : it->second) {
      handle_to_sensor_.erase(callback_info.handle);
    }
    callbacks_.erase(it);
  }
}

std::vector<uint8_t> SensorCallbackManager::serializeSensorData(
    std::shared_ptr<carla::sensor::SensorData> data, SensorDataType &out_type) {

  std::vector<uint8_t> result;

  // Create header
  SensorDataHeader header;
  header.frame = data->GetFrame();
  header.timestamp = data->GetTimestamp();

  // Extract transform from sensor data
  auto transform = data->GetSensorTransform();
  header.transform.location.x = static_cast<float>(transform.location.x);
  header.transform.location.y = static_cast<float>(transform.location.y);
  header.transform.location.z = static_cast<float>(transform.location.z);
  header.transform.rotation.pitch =
      static_cast<float>(transform.rotation.pitch);
  header.transform.rotation.yaw = static_cast<float>(transform.rotation.yaw);
  header.transform.rotation.roll = static_cast<float>(transform.rotation.roll);

  // Try to cast to each sensor data type and serialize
  if (auto image = std::dynamic_pointer_cast<
          carla::sensor::data::ImageTmpl<carla::sensor::data::Color>>(data)) {
    out_type = SensorDataType::Image;
    header.data_type = out_type;

    // Add header
    result.resize(sizeof(header) +
                  image->size() * sizeof(carla::sensor::data::Color));
    memcpy(result.data(), &header, sizeof(header));

    // Add image data
    memcpy(result.data() + sizeof(header), image->data(),
           image->size() * sizeof(carla::sensor::data::Color));
  } else if (auto lidar =
                 std::dynamic_pointer_cast<carla::sensor::data::LidarData>(
                     data)) {
    out_type = SensorDataType::LiDAR;
    header.data_type = out_type;

    // Add header
    size_t point_count = lidar->size();
    result.resize(sizeof(header) + sizeof(uint32_t) +
                  point_count * sizeof(float) * 4);
    memcpy(result.data(), &header, sizeof(header));

    // Add point count
    uint32_t count = static_cast<uint32_t>(point_count);
    memcpy(result.data() + sizeof(header), &count, sizeof(count));

    // Add point data (x, y, z, intensity)
    size_t offset = sizeof(header) + sizeof(count);
    for (size_t i = 0; i < point_count; ++i) {
      auto point = (*lidar)[i];
      float point_data[4] = {point.x, point.y, point.z, point.intensity};
      memcpy(result.data() + offset, point_data, sizeof(point_data));
      offset += sizeof(point_data);
    }
  } else if (auto radar =
                 std::dynamic_pointer_cast<carla::sensor::data::RadarData>(
                     data)) {
    out_type = SensorDataType::Radar;
    header.data_type = out_type;

    // Add header and radar detections
    size_t detection_count = radar->GetDetectionCount();
    result.resize(sizeof(header) + sizeof(uint32_t) +
                  detection_count *
                      sizeof(carla::sensor::data::RadarDetection));
    memcpy(result.data(), &header, sizeof(header));

    uint32_t count = static_cast<uint32_t>(detection_count);
    memcpy(result.data() + sizeof(header), &count, sizeof(count));

    // Copy radar detections
    if (detection_count > 0) {
      memcpy(result.data() + sizeof(header) + sizeof(count),
             radar->GetDetections(),
             detection_count * sizeof(carla::sensor::data::RadarDetection));
    }
  } else if (auto imu =
                 std::dynamic_pointer_cast<carla::sensor::data::IMUMeasurement>(
                     data)) {
    out_type = SensorDataType::IMU;
    header.data_type = out_type;

    // IMU data: accelerometer + gyroscope + compass
    result.resize(sizeof(header) +
                  sizeof(float) * 7); // 3 accel + 3 gyro + 1 compass
    memcpy(result.data(), &header, sizeof(header));

    float imu_data[7] = {static_cast<float>(imu->GetAccelerometer().x),
                         static_cast<float>(imu->GetAccelerometer().y),
                         static_cast<float>(imu->GetAccelerometer().z),
                         static_cast<float>(imu->GetGyroscope().x),
                         static_cast<float>(imu->GetGyroscope().y),
                         static_cast<float>(imu->GetGyroscope().z),
                         static_cast<float>(imu->GetCompass())};
    memcpy(result.data() + sizeof(header), imu_data, sizeof(imu_data));
  } else if (auto gnss = std::dynamic_pointer_cast<
                 carla::sensor::data::GnssMeasurement>(data)) {
    out_type = SensorDataType::GNSS;
    header.data_type = out_type;

    // GNSS data: latitude, longitude, altitude
    result.resize(sizeof(header) + sizeof(double) * 3);
    memcpy(result.data(), &header, sizeof(header));

    double gnss_data[3] = {gnss->GetLatitude(), gnss->GetLongitude(),
                           gnss->GetAltitude()};
    memcpy(result.data() + sizeof(header), gnss_data, sizeof(gnss_data));
  } else if (auto collision =
                 std::dynamic_pointer_cast<carla::sensor::data::CollisionEvent>(
                     data)) {
    out_type = SensorDataType::Collision;
    header.data_type = out_type;

    // Collision data: normal impulse + actor ID
    result.resize(sizeof(header) + sizeof(float) * 3 + sizeof(uint32_t));
    memcpy(result.data(), &header, sizeof(header));

    auto normal = collision->GetNormalImpulse();
    float normal_data[3] = {normal.x, normal.y, normal.z};
    memcpy(result.data() + sizeof(header), normal_data, sizeof(normal_data));

    uint32_t other_actor =
        collision->GetOtherActor() ? collision->GetOtherActor()->GetId() : 0;
    memcpy(result.data() + sizeof(header) + sizeof(normal_data), &other_actor,
           sizeof(other_actor));
  } else if (auto lane_invasion = std::dynamic_pointer_cast<
                 carla::sensor::data::LaneInvasionEvent>(data)) {
    out_type = SensorDataType::LaneInvasion;
    header.data_type = out_type;

    // Lane invasion: crossed lane markings count + types
    auto crossed_markings = lane_invasion->GetCrossedLaneMarkings();
    result.resize(sizeof(header) + sizeof(uint32_t) +
                  crossed_markings.size() * sizeof(int32_t));
    memcpy(result.data(), &header, sizeof(header));

    uint32_t count = static_cast<uint32_t>(crossed_markings.size());
    memcpy(result.data() + sizeof(header), &count, sizeof(count));

    // Copy lane marking types
    size_t offset = sizeof(header) + sizeof(count);
    for (const auto &marking : crossed_markings) {
      int32_t type = static_cast<int32_t>(marking);
      memcpy(result.data() + offset, &type, sizeof(type));
      offset += sizeof(type);
    }
  } else {
    // Unknown sensor type - return empty data with header only
    out_type = SensorDataType::Image; // Default
    header.data_type = out_type;
    result.resize(sizeof(header));
    memcpy(result.data(), &header, sizeof(header));
  }

  return result;
}

} // namespace carla_sys
