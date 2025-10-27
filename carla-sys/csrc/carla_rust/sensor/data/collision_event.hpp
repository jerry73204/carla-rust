#pragma once

#include "carla/Memory.h"
#include "carla/sensor/data/CollisionEvent.h"
#include "carla/geom/Vector3D.h"
#include "carla_rust/client/actor.hpp"

namespace carla_rust {
namespace sensor {
namespace data {
using carla::SharedPtr;
using carla::geom::Vector3D;
using carla::sensor::data::CollisionEvent;
using carla_rust::client::FfiActor;

// CollisionEvent
class FfiCollisionEvent {
public:
    FfiCollisionEvent(SharedPtr<CollisionEvent>&& base) : inner_(std::move(base)) {}

    std::shared_ptr<FfiActor> GetActor() const {
        return std::make_shared<FfiActor>(std::move(inner_->GetActor()));
    }

    std::shared_ptr<FfiActor> GetOtherActor() const {
        auto ptr = inner_->GetOtherActor();
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiActor>(std::move(ptr));
        }
    }

    const Vector3D& GetNormalImpulse() const { return inner_->GetNormalImpulse(); }

private:
    SharedPtr<CollisionEvent> inner_;
};
}  // namespace data
}  // namespace sensor
}  // namespace carla_rust
