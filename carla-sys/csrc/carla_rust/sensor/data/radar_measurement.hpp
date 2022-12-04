#pragma once

#include "carla/Memory.h"
#include "carla/sensor/data/RadarData.h"
#include "carla/sensor/data/RadarMeasurement.h"

namespace carla_rust
{
    namespace sensor {
        namespace data {
            using carla::SharedPtr;
            using carla::sensor::data::RadarDetection;
            using carla::sensor::data::RadarMeasurement;

            class FfiRadarMeasurement {
            public:
                FfiRadarMeasurement(SharedPtr<RadarMeasurement> &&base)
                    :
                    inner_(std::move(base))
                {}

                size_t GetDetectionAmount() const {
                    return inner_->GetDetectionAmount();
                }

                size_t size() const {
                    return inner_->size();
                }

                const RadarDetection* data() const {
                    return inner_->data();
                }

                const RadarDetection& at(size_t pos) const {
                    return inner_->at(pos);
                }

            private:
                SharedPtr<RadarMeasurement> inner_;
            };
       }
    }
}
