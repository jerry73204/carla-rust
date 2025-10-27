#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/client/Actor.h"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Actor;

class FfiActorVec {
public:
    FfiActorVec(std::vector<SharedPtr<Actor>>&& vec) : inner_(std::move(vec)) {}

    size_t len() const { return inner_.size(); }

    std::shared_ptr<FfiActor> get(size_t index) const {
        auto orig = inner_.at(index);
        return std::make_shared<FfiActor>(std::move(orig));
    }

private:
    std::vector<SharedPtr<Actor>> inner_;
};
}  // namespace client
}  // namespace carla_rust
