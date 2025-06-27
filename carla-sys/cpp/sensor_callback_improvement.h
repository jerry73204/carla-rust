// Improved sensor callback implementation with per-sensor data storage
// This header defines the improved polling-based sensor data collection

#pragma once

#include <carla/sensor/SensorData.h>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace carla_sys {

// Thread-safe sensor data storage
class SensorDataStore {
public:
  static SensorDataStore &getInstance() {
    static SensorDataStore instance;
    return instance;
  }

  // Store sensor data for a specific sensor
  void storeSensorData(uint32_t sensor_id,
                       std::shared_ptr<carla::sensor::SensorData> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor_data_[sensor_id] = data;
    has_new_data_[sensor_id] = true;
  }

  // Check if sensor has new data
  bool hasNewData(uint32_t sensor_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = has_new_data_.find(sensor_id);
    if (it != has_new_data_.end()) {
      bool result = it->second;
      it->second = false; // Reset flag after checking
      return result;
    }
    return false;
  }

  // Get latest sensor data
  std::shared_ptr<carla::sensor::SensorData> getSensorData(uint32_t sensor_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensor_data_.find(sensor_id);
    if (it != sensor_data_.end()) {
      return it->second;
    }
    return nullptr;
  }

  // Clear data for a sensor (when it stops listening)
  void clearSensorData(uint32_t sensor_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor_data_.erase(sensor_id);
    has_new_data_.erase(sensor_id);
  }

private:
  SensorDataStore() = default;
  ~SensorDataStore() = default;
  SensorDataStore(const SensorDataStore &) = delete;
  SensorDataStore &operator=(const SensorDataStore &) = delete;

  std::unordered_map<uint32_t, std::shared_ptr<carla::sensor::SensorData>>
      sensor_data_;
  std::unordered_map<uint32_t, bool> has_new_data_;
  std::mutex mutex_;
};

} // namespace carla_sys
