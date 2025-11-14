#pragma once

#include "carla/sensor/data/DVSEventArray.h"
#include "color.hpp"
#include <cstdint>

namespace carla_rust {
namespace sensor {
namespace data {

using carla::SharedPtr;
using carla::sensor::data::DVSEvent;
using carla::sensor::data::DVSEventArray;

// FFI wrapper for DVSEventArray
class FfiDVSEventArray {
public:
    FfiDVSEventArray(SharedPtr<DVSEventArray>&& base) : inner_(std::move(base)) {}

    size_t GetWidth() const { return inner_->GetWidth(); }

    size_t GetHeight() const { return inner_->GetHeight(); }

    float GetFOVAngle() const { return inner_->GetFOVAngle(); }

    size_t size() const { return inner_->size(); }

    const DVSEvent* data() const { return inner_->data(); }

    const DVSEvent& at(size_t pos) const { return inner_->at(pos); }

    // Convert to image for visualization (blue for positive, red for negative)
    std::vector<FfiColor> ToImage() const {
        auto img = inner_->ToImage();
        // Convert Color to FfiColor (same memory layout)
        // Use move + reserve to avoid reinterpret_cast strict-aliasing issues
        std::vector<FfiColor> result;
        result.reserve(img.size());
        for (auto& color : img) {
            result.push_back(*reinterpret_cast<FfiColor*>(&color));
        }
        return result;
    }

private:
    SharedPtr<DVSEventArray> inner_;
};

}  // namespace data
}  // namespace sensor
}  // namespace carla_rust
