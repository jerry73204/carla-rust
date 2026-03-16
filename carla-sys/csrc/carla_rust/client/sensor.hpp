#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla_rust/compat.hpp"
#include "carla/client/Sensor.h"
#include "carla/client/ServerSideSensor.h"
#include "carla/sensor/SensorData.h"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Sensor;
using carla::client::ServerSideSensor;
using carla::sensor::SensorData;

// Sensor
class FfiSensor {
public:
    FfiSensor(SharedPtr<Sensor>&& base) : inner_(std::move(base)) {}

    void Listen(void* caller, void* fn, void* delete_fn, FfiError& error) const {
        ffi_call_void(error, [&]() {
            auto container = std::make_shared<ListenCallback>(caller, fn, delete_fn);
            auto callback = [container = std::move(container)](SharedPtr<SensorData> data) {
                auto ffi_data = std::make_shared<FfiSensorData>(std::move(data));
                (*container)(ffi_data);
            };
            inner_->Listen(std::move(callback));
        });
    }

    void Stop(FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->Stop(); });
    }

    bool IsListening(FfiError& error) const {
        return ffi_call(error, false, [&]() { return inner_->IsListening(); });
    }

    // GBuffer methods (available in all versions)

    bool IsListeningGBuffer(uint32_t id, FfiError& error) const {
        return ffi_call(error, false, [&]() {
            auto ss = carla_dynamic_pointer_cast<ServerSideSensor>(inner_);
            if (ss == nullptr)
                return false;
            return ss->IsListeningGBuffer(id);
        });
    }

    void StopGBuffer(uint32_t id, FfiError& error) const {
        ffi_call_void(error, [&]() {
            auto ss = carla_dynamic_pointer_cast<ServerSideSensor>(inner_);
            if (ss != nullptr) {
                ss->StopGBuffer(id);
            }
        });
    }

    // ROS bridge methods (0.9.15+ only)

#ifdef CARLA_VERSION_0915_PLUS
    void EnableForROS(FfiError& error) const {
        ffi_call_void(error, [&]() {
            auto ss = carla_dynamic_pointer_cast<ServerSideSensor>(inner_);
            if (ss != nullptr) {
                ss->EnableForROS();
            }
        });
    }

    void DisableForROS(FfiError& error) const {
        ffi_call_void(error, [&]() {
            auto ss = carla_dynamic_pointer_cast<ServerSideSensor>(inner_);
            if (ss != nullptr) {
                ss->DisableForROS();
            }
        });
    }

    bool IsEnabledForROS(FfiError& error) const {
        return ffi_call(error, false, [&]() {
            auto ss = carla_dynamic_pointer_cast<ServerSideSensor>(inner_);
            if (ss == nullptr)
                return false;
            return ss->IsEnabledForROS();
        });
    }
#endif

    std::shared_ptr<FfiActor> to_actor() const {
        SharedPtr<Actor> ptr = carla_static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<Sensor> inner_;
};
}  // namespace client
}  // namespace carla_rust
