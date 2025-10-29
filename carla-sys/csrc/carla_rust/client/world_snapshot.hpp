#pragma once

#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/client/WorldSnapshot.h"
#include "carla/client/ActorSnapshot.h"
#include "carla/client/Timestamp.h"
#include "carla_rust/rpc/actor_id.hpp"
#include "carla_rust/client/actor_snapshot.hpp"
#include "carla_rust/client/actor_snapshot_list.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::ActorSnapshot;
using carla::client::Map;
using carla::client::Timestamp;
using carla::client::WorldSnapshot;
using carla_rust::rpc::FfiActorId;

/// FFI wrapper for carla::client::WorldSnapshot
class FfiWorldSnapshot {
public:
    FfiWorldSnapshot(WorldSnapshot&& base) : inner_(std::move(base)) {}

    uint64_t GetId() const { return inner_.GetId(); }

    size_t GetFrame() const { return inner_.GetFrame(); }

    const Timestamp& GetTimestamp() const { return inner_.GetTimestamp(); }

    bool Contains(FfiActorId actor_id) const { return inner_.Contains(actor_id); }

    /// Find an actor snapshot by actor ID.
    /// Returns a pointer to FfiActorSnapshot if found, nullptr otherwise.
    std::unique_ptr<FfiActorSnapshot> Find(FfiActorId actor_id) const {
        auto opt_snapshot = inner_.Find(actor_id);
        if (opt_snapshot) {
            return std::make_unique<FfiActorSnapshot>(*opt_snapshot);
        }
        return nullptr;
    }

    /// Get all actor snapshots as a list.
    /// This is used for iteration from Rust.
    std::unique_ptr<FfiActorSnapshotList> GetActorSnapshots() const {
        std::vector<ActorSnapshot> snapshots;
        snapshots.reserve(inner_.size());
        for (const auto& snapshot : inner_) {
            snapshots.push_back(snapshot);
        }
        return std::make_unique<FfiActorSnapshotList>(std::move(snapshots));
    }

    size_t size() const { return inner_.size(); }

private:
    WorldSnapshot inner_;
};
}  // namespace client
}  // namespace carla_rust
