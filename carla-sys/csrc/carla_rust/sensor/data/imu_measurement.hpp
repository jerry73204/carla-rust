#pragma once

#include "carla/Memory.h"
#include "carla/sensor/data/IMUMeasurement.h"
#include "carla/geom/Vector3D.h"

namespace carla_rust
{
    namespace sensor {
        namespace data {
            using carla::SharedPtr;
            using carla::geom::Vector3D;
            using carla::sensor::data::IMUMeasurement;

            class FfiImuMeasurement {
            public:
                FfiImuMeasurement(SharedPtr<IMUMeasurement> &&base)
                    :
                    inner_(std::move(base))
                {}

                Vector3D GetAccelerometer() const {
                    return inner_->GetAccelerometer();
                }

                Vector3D GetGyroscope() const {
                    return inner_->GetGyroscope();
                }

                float GetCompass() const {
                    return inner_->GetCompass();
                }

            private:
                SharedPtr<IMUMeasurement> inner_;
            };
       }
    }
}
