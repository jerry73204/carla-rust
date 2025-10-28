#pragma once

#include "carla/sensor/data/Image.h"
#include <cstdint>

namespace carla_rust {
namespace sensor {
namespace data {

using carla::SharedPtr;
using carla::sensor::data::OpticalFlowImage;
using carla::sensor::data::OpticalFlowPixel;

// FFI wrapper for OpticalFlowPixel (POD type with x, y float components)
struct FfiOpticalFlowPixel {
    float x;
    float y;
};

// FFI wrapper for OpticalFlowImage
class FfiOpticalFlowImage {
public:
    FfiOpticalFlowImage(SharedPtr<OpticalFlowImage>&& base) : inner_(std::move(base)) {}

    size_t GetWidth() const { return inner_->GetWidth(); }

    size_t GetHeight() const { return inner_->GetHeight(); }

    float GetFOVAngle() const { return inner_->GetFOVAngle(); }

    size_t size() const { return inner_->size(); }

    const FfiOpticalFlowPixel* data() const {
        auto orig = inner_->data();
        // OpticalFlowPixel and FfiOpticalFlowPixel have the same layout
        return reinterpret_cast<const FfiOpticalFlowPixel*>(orig);
    }

    const FfiOpticalFlowPixel& at(size_t pos) const {
        const OpticalFlowPixel& orig = inner_->at(pos);
        return reinterpret_cast<const FfiOpticalFlowPixel&>(orig);
    }

private:
    SharedPtr<OpticalFlowImage> inner_;
};

}  // namespace data
}  // namespace sensor
}  // namespace carla_rust
