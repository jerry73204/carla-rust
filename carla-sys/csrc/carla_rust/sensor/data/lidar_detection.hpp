#pragma once

#include "carla/sensor/data/LidarData.h"
#include "carla_rust/geom.hpp"

namespace carla_rust {
namespace sensor {
namespace data {
using carla::sensor::data::LidarDetection;
using carla_rust::geom::FfiLocation;

// LidarDetection
class FfiLidarDetection {
public:
    FfiLocation point;
    float intensity;

    FfiLidarDetection(LidarDetection&& base)
        : point(reinterpret_cast<FfiLocation&&>(std::move(base.point))),
          intensity(std::move(base.intensity)) {}

    LidarDetection& as_builtin() { return reinterpret_cast<LidarDetection&>(*this); }
};
static_assert(sizeof(FfiLidarDetection) == sizeof(LidarDetection),
              "FfiLidarDetection and LidarDetection size mismatch");
}  // namespace data
}  // namespace sensor
}  // namespace carla_rust
