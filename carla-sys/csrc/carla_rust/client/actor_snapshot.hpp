#pragma once

#include "carla/client/ActorSnapshot.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Vector3D.h"
#include "carla_rust/rpc/actor_id.hpp"
#include "carla_rust/geom.hpp"

namespace carla_rust {
namespace client {
using carla::client::ActorSnapshot;
using carla::geom::Transform;
using carla::geom::Vector3D;
using carla_rust::geom::FfiTransform;
using carla_rust::rpc::FfiActorId;

/// FFI wrapper for carla::client::ActorSnapshot
///
/// ActorSnapshot is a simple struct containing actor state at a moment in time.
/// This wrapper provides safe accessor methods for Rust FFI.
class FfiActorSnapshot {
public:
    FfiActorSnapshot(const ActorSnapshot& snapshot) : inner_(snapshot) {}

    /// Get the actor's ID
    FfiActorId GetId() const { return inner_.id; }

    /// Get the actor's transform (position and rotation)
    FfiTransform GetTransform() const { return FfiTransform(inner_.transform); }

    /// Get the actor's velocity vector (m/s)
    Vector3D GetVelocity() const { return inner_.velocity; }

    /// Get the actor's angular velocity vector (rad/s)
    Vector3D GetAngularVelocity() const { return inner_.angular_velocity; }

    /// Get the actor's acceleration vector (m/s²)
    Vector3D GetAcceleration() const { return inner_.acceleration; }

private:
    ActorSnapshot inner_;
};

}  // namespace client
}  // namespace carla_rust
