#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Sensor.h"
#include "carla/sensor/SensorData.h"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Sensor;
        using carla::sensor::SensorData;

        // Sensor
        class FfiSensor {
        public:
            FfiSensor(SharedPtr<Sensor> &&base) : inner_(std::move(base)) {}

            void Listen(void *caller, void *fn, void *delete_fn) const {
                auto container = std::make_shared<ListenCallback>(caller, fn, delete_fn);
                auto callback =
                    [container = std::move(container)]
                    (SharedPtr<SensorData> data)
                    {
                        auto ffi_data = std::make_shared<FfiSensorData>(std::move(data));
                        (*container)(ffi_data);
                    };
                inner_->Listen(std::move(callback));
            }

            void Stop() const {
                inner_->Stop();
            }

            bool IsListening() const {
                return inner_->IsListening();
            }

            std::shared_ptr<FfiActor> to_actor() const {
                SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
                return std::make_shared<FfiActor>(std::move(ptr));
            }

        private:
            SharedPtr<Sensor> inner_;
        };
    }
} // namespace carla_rust
