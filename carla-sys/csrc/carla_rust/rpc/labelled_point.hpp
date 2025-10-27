#pragma once

#include "carla/rpc/ObjectLabel.h"
#include "carla/rpc/LabelledPoint.h"
#include "carla_rust/geom.hpp"

namespace carla_rust {
namespace rpc {
using carla::rpc::CityObjectLabel;
using carla::rpc::LabelledPoint;
using carla_rust::geom::FfiLocation;

struct FfiLabelledPoint {
public:
    FfiLocation location;
    CityObjectLabel label;

    FfiLabelledPoint(LabelledPoint&& orig)
        : location(std::move(orig._location)), label(std::move(orig._label)) {}
};

static_assert(sizeof(FfiLabelledPoint) == sizeof(LabelledPoint),
              "FfiLabelledPoint size is incorrect");
}  // namespace rpc
}  // namespace carla_rust
