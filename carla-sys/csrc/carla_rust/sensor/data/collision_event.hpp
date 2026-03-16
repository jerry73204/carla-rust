#pragma once

#include "carla/Memory.h"
#include "carla/sensor/data/CollisionEvent.h"
#include "carla/geom/Vector3D.h"
#include "carla_rust/client/actor.hpp"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace sensor {
namespace data {
using carla::SharedPtr;
using carla::geom::Vector3D;
using carla::sensor::data::CollisionEvent;
using carla_rust::client::ffi_call;
using carla_rust::client::FfiActor;
using carla_rust::client::FfiError;

// CollisionEvent
class FfiCollisionEvent {
public:
    FfiCollisionEvent(SharedPtr<CollisionEvent>&& base) : inner_(std::move(base)) {}

    std::shared_ptr<FfiActor> GetActor(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() {
            return std::make_shared<FfiActor>(std::move(inner_->GetActor()));
        });
    }

    std::shared_ptr<FfiActor> GetOtherActor(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            auto ptr = inner_->GetOtherActor();
            if (ptr == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(ptr));
            }
        });
    }

    const Vector3D& GetNormalImpulse() const { return inner_->GetNormalImpulse(); }

private:
    SharedPtr<CollisionEvent> inner_;
};
}  // namespace data
}  // namespace sensor
}  // namespace carla_rust
