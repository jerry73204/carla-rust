#pragma once

#include "carla/Memory.h"
#include "carla/sensor/data/GnssMeasurement.h"
#include "carla/geom/GeoLocation.h"

namespace carla_rust {
namespace sensor {
namespace data {
using carla::SharedPtr;
using carla::geom::GeoLocation;
using carla::sensor::data::GnssMeasurement;

class FfiGnssMeasurement {
public:
    FfiGnssMeasurement(SharedPtr<GnssMeasurement>&& base) : inner_(std::move(base)) {}

    GeoLocation GetGeoLocation() const { return inner_->GetGeoLocation(); }

    double GetLongitude() const { return inner_->GetLongitude(); }

    double GetLatitude() const { return inner_->GetLatitude(); }

    double GetAltitude() const { return inner_->GetAltitude(); }

private:
    SharedPtr<GnssMeasurement> inner_;
};
}  // namespace data
}  // namespace sensor
}  // namespace carla_rust
