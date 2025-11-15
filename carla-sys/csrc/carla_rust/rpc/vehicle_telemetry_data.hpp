#pragma once

#include <vector>
#include <cstdint>

#ifdef CARLA_VERSION_0916
#include "carla/rpc/VehicleTelemetryData.h"
#include "carla/rpc/WheelTelemetryData.h"
#else
#include "carla_rust/rpc/wheel_telemetry_data_stub.hpp"
#endif

namespace carla_rust {
namespace rpc {

#ifdef CARLA_VERSION_0916

using carla::rpc::VehicleTelemetryData;
using carla::rpc::WheelTelemetryData;

// Wrapper for VehicleTelemetryData to provide field access
class FfiVehicleTelemetryData {
public:
    FfiVehicleTelemetryData(const VehicleTelemetryData& data) : data_(data) {}

    float speed() const { return data_.speed; }
    float steer() const { return data_.steer; }
    float throttle() const { return data_.throttle; }
    float brake() const { return data_.brake; }
    float engine_rpm() const { return data_.engine_rpm; }
    int32_t gear() const { return data_.gear; }
    float drag() const { return data_.drag; }

    const std::vector<WheelTelemetryData>& wheels() const { return data_.wheels; }

private:
    VehicleTelemetryData data_;
};

#else  // Not CARLA_VERSION_0916 - create stub types for autocxx

// Stub type for versions < 0.9.16
// This allows autocxx to see the type name without failing,
// but the type won't be used since it's not exposed in the API
struct FfiVehicleTelemetryData {
    // Empty stub - never instantiated
    uint8_t _unused;
};

#endif  // CARLA_VERSION_0916

}  // namespace rpc
}  // namespace carla_rust
