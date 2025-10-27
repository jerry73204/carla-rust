#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/rpc/LabelledPoint.h"

namespace carla_rust {
namespace client {
using carla::rpc::LabelledPoint;
using carla_rust::rpc::FfiLabelledPoint;

class FfiLabelledPointList {
public:
    FfiLabelledPointList(std::vector<LabelledPoint>&& vec) : inner_(std::move(vec)) {}

    size_t len() const { return inner_.size(); }

    const FfiLabelledPoint* data() const {
        return reinterpret_cast<const FfiLabelledPoint*>(inner_.data());
    }

private:
    std::vector<LabelledPoint> inner_;
};
}  // namespace client
}  // namespace carla_rust
