#pragma once

#ifdef CARLA_VERSION_0916

#include "carla/rpc/VehicleTelemetryData.h"
#include "carla/rpc/WheelTelemetryData.h"
#include <vector>

namespace carla_rust {
namespace rpc {

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

}  // namespace rpc
}  // namespace carla_rust

#endif  // CARLA_VERSION_0916
