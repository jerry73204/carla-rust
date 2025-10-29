#pragma once

#include <vector>
#include "carla/client/ActorSnapshot.h"
#include "carla_rust/client/actor_snapshot.hpp"

namespace carla_rust {
namespace client {
using carla::client::ActorSnapshot;

/// FFI wrapper for a list of ActorSnapshots
///
/// Provides indexed access to actor snapshots for Rust iteration.
class FfiActorSnapshotList {
public:
    FfiActorSnapshotList(std::vector<ActorSnapshot>&& snapshots)
        : snapshots_(std::move(snapshots)) {}

    /// Get the number of actor snapshots in the list
    size_t size() const { return snapshots_.size(); }

    /// Get an actor snapshot by index
    /// Returns nullptr if index is out of bounds
    std::unique_ptr<FfiActorSnapshot> get(size_t index) const {
        if (index >= snapshots_.size()) {
            return nullptr;
        }
        return std::make_unique<FfiActorSnapshot>(snapshots_[index]);
    }

private:
    std::vector<ActorSnapshot> snapshots_;
};

}  // namespace client
}  // namespace carla_rust
