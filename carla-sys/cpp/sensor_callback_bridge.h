// Enhanced sensor callback implementation with true callbacks
#pragma once

#include <carla/sensor/SensorData.h>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace carla_sys {

// FFI-safe callback function type that matches Rust
typedef void (*SensorDataCallback)(uint32_t sensor_id, const uint8_t *data,
                                   size_t size, uint8_t *user_data);

// Sensor data type enum matching Rust
enum class SensorDataType : uint8_t {
  Image = 0,
  LiDAR = 1,
  Radar = 2,
  IMU = 3,
  GNSS = 4,
  Collision = 5,
  LaneInvasion = 6,
  ObstacleDetection = 7,
  DVS = 8,
  SemanticLiDAR = 9,
  RSS = 10,
};

// Sensor data header matching Rust
struct SensorDataHeader {
  SensorDataType data_type;
  uint64_t frame;
  double timestamp;
  // Transform will be added here
};

// Callback manager for sensor data
class SensorCallbackManager {
public:
  static SensorCallbackManager &getInstance() {
    static SensorCallbackManager instance;
    return instance;
  }

  // Register a callback for a sensor
  uint64_t registerCallback(uint32_t sensor_id, SensorDataCallback callback,
                            uint8_t *user_data);

  // Unregister a callback
  bool unregisterCallback(uint64_t handle);

  // Process sensor data (called from C++ sensor callback)
  void processSensorData(uint32_t sensor_id,
                         std::shared_ptr<carla::sensor::SensorData> data);

  // Clean up callbacks for a sensor
  void cleanupSensor(uint32_t sensor_id);

private:
  SensorCallbackManager() = default;
  ~SensorCallbackManager() = default;
  SensorCallbackManager(const SensorCallbackManager &) = delete;
  SensorCallbackManager &operator=(const SensorCallbackManager &) = delete;

  struct CallbackInfo {
    SensorDataCallback callback;
    uint8_t *user_data;
    uint64_t handle;
  };

  std::unordered_map<uint32_t, std::vector<CallbackInfo>> callbacks_;
  std::unordered_map<uint64_t, uint32_t> handle_to_sensor_;
  std::mutex mutex_;
  uint64_t next_handle_ = 1;

  // Helper to serialize sensor data
  std::vector<uint8_t>
  serializeSensorData(std::shared_ptr<carla::sensor::SensorData> data,
                      SensorDataType &out_type);
};

} // namespace carla_sys
