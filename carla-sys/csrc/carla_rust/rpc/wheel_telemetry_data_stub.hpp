#pragma once

// Stub header for WheelTelemetryData in CARLA versions < 0.9.16
// This provides a compatible stub type for autocxx when the actual type doesn't exist

#ifndef CARLA_VERSION_0916

#include <cstdint>

namespace carla {
namespace rpc {

// Stub struct for WheelTelemetryData
// Never actually used - just allows autocxx generate_pod! to succeed
struct WheelTelemetryData {
    float tire_friction;
    float lat_slip;
    float long_slip;
    float omega;
    float tire_load;
    float normalized_tire_load;
    float torque;
    float long_force;
    float lat_force;
    float normalized_long_force;
    float normalized_lat_force;
};

}  // namespace rpc
}  // namespace carla

#endif  // !CARLA_VERSION_0916
