#pragma once

#include "carla/Memory.h"
#include "carla/sensor/data/SemanticLidarData.h"
#include "carla/sensor/data/SemanticLidarMeasurement.h"
#include "carla_rust/sensor/data/semantic_lidar_detection.hpp"

namespace carla_rust
{
    namespace sensor {
        namespace data {
            using carla::SharedPtr;
            using carla::sensor::data::SemanticLidarDetection;
            using carla::sensor::data::SemanticLidarMeasurement;
            using carla_rust::sensor::data::FfiSemanticLidarDetection;

            // Image
            class FfiSemanticLidarMeasurement {
            public:
                FfiSemanticLidarMeasurement(SharedPtr<SemanticLidarMeasurement> &&base)
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

                const FfiSemanticLidarDetection* data() const {
                    auto orig = inner_->data();
                    auto new_ = reinterpret_cast<FfiSemanticLidarDetection*>(orig);
                    return new_;
                }

                const FfiSemanticLidarDetection& at(size_t pos) const {
                    const SemanticLidarDetection& orig = inner_->at(pos);
                    const FfiSemanticLidarDetection& new_ = reinterpret_cast<const FfiSemanticLidarDetection&>(orig);
                    return new_;
                }

            private:
                SharedPtr<SemanticLidarMeasurement> inner_;
            };
       }
    }
}
