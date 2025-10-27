#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/client/Light.h"
#include "carla_rust/client/light.hpp"

namespace carla_rust {
namespace client {
using carla::client::Light;
using carla_rust::client::FfiLightRef;

class FfiLightList {
public:
    FfiLightList(std::vector<Light>&& base) : inner_(std::move(base)) {}

    size_t size() const { return inner_.size(); }

    FfiLightRef at(size_t index) {
        Light& orig = inner_.at(index);
        return FfiLightRef(orig);
    }

    std::vector<Light>& inner() { return inner_; }

private:
    std::vector<Light> inner_;
};
}  // namespace client
}  // namespace carla_rust
