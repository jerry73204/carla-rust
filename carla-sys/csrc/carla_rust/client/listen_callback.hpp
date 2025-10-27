#pragma once

#include <memory>
#include "carla/sensor/SensorData.h"
#include "carla_rust/sensor/sensor_data.hpp"

namespace carla_rust {
namespace client {
using carla_rust::sensor::FfiSensorData;

class ListenCallback {
public:
    ListenCallback(void* caller, void* fn, void* delete_fn) {
        auto caller_ptr =
            reinterpret_cast<void (*)(void* ctx, std::shared_ptr<FfiSensorData>* data)>(caller);
        auto delete_fn_ptr = reinterpret_cast<void (*)(void* ctx)>(delete_fn);

        caller_ = caller_ptr;
        fn_ = fn;
        delete_fn_ = delete_fn_ptr;
    }

    ListenCallback(ListenCallback&&) = default;

    ~ListenCallback() { (delete_fn_)(fn_); }

    ListenCallback& operator=(ListenCallback&&) = default;

    void operator()(std::shared_ptr<FfiSensorData> data) const { (caller_)(fn_, &data); }

private:
    void (*caller_)(void* fn, std::shared_ptr<FfiSensorData>* data);
    void* fn_;
    void (*delete_fn_)(void* fn);
};

}  // namespace client
}  // namespace carla_rust
