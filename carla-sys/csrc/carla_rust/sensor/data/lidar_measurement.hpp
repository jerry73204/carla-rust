#pragma once

#include "carla/Memory.h"
#include "carla/sensor/data/LidarData.h"
#include "carla/sensor/data/LidarMeasurement.h"
#include "carla_rust/sensor/data/lidar_detection.hpp"

namespace carla_rust
{
    namespace sensor {
        namespace data {
            using carla::SharedPtr;
            using carla::sensor::data::LidarDetection;
            using carla::sensor::data::LidarMeasurement;
            using carla_rust::sensor::data::FfiLidarDetection;

            class FfiLidarMeasurement {
            public:
                FfiLidarMeasurement(SharedPtr<LidarMeasurement> &&base)
                    :
                    inner_(std::move(base))
                {}

                float GetHorizontalAngle() const {
                    return inner_->GetHorizontalAngle();
                }

                uint32_t GetChannelCount() const {
                    return inner_->GetHorizontalAngle();
                }

                uint32_t GetPointCount(size_t channel) const {
                    return inner_->GetPointCount(channel);
                }

                size_t size() const {
                    return inner_->size();
                }

                const FfiLidarDetection* data() const {
                    auto orig = inner_->data();
                    auto new_ = reinterpret_cast<FfiLidarDetection*>(orig);
                    return new_;
                }

                const FfiLidarDetection& at(size_t pos) const {
                    const LidarDetection& orig = inner_->at(pos);
                    const FfiLidarDetection& new_ = reinterpret_cast<const FfiLidarDetection&>(orig);
                    return new_;
                }

            private:
                SharedPtr<LidarMeasurement> inner_;
            };
       }
    }
}
