#pragma once

#include <cstdint>
#include <cstring>
#include <string>

#include "carla/Memory.h"
#ifdef CARLA_VERSION_0916
#include "carla/sensor/data/V2XEvent.h"
#endif

namespace carla_rust {
namespace sensor {
namespace data {

#ifdef CARLA_VERSION_0916
using carla::SharedPtr;
using carla::sensor::data::CustomV2XEvent;

/// The messages one `sensor.other.v2x_custom` received in one frame: per
/// message the received power (dBm), the sender's ITS station id and the
/// text it sent (at most 100 bytes, as `carla.Sensor.send` takes it).
class FfiCustomV2XEvent {
public:
    FfiCustomV2XEvent(SharedPtr<CustomV2XEvent>&& base) : inner_(std::move(base)) {}

    size_t size() const { return inner_->size(); }

    float power_at(size_t index) const { return inner_->at(index).Power; }

    int64_t station_id_at(size_t index) const {
        return static_cast<int64_t>(inner_->at(index).Message.header.stationID);
    }

    std::string message_at(size_t index) const {
        const auto& msg = inner_->at(index).Message.message;
        return std::string(msg, strnlen(msg, sizeof(msg)));
    }

private:
    SharedPtr<CustomV2XEvent> inner_;
};
#else
/// Stub for CARLA versions without the V2X sensors (before 0.9.16); never
/// constructed (`FfiSensorData::to_custom_v2x_event` returns null).
class FfiCustomV2XEvent {
public:
    size_t size() const { return 0; }
    float power_at(size_t) const { return 0.0f; }
    int64_t station_id_at(size_t) const { return 0; }
    std::string message_at(size_t) const { return std::string(); }
};
#endif

}  // namespace data
}  // namespace sensor
}  // namespace carla_rust
