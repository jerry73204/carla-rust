#pragma once

#include "carla/rpc/ActorId.h"

namespace carla_rust {
namespace rpc {
using carla::rpc::ActorId;

using FfiActorId = uint32_t;
static_assert(sizeof(FfiActorId) == sizeof(ActorId), "FfiActorId has incorrect size");
}  // namespace rpc
}  // namespace carla_rust
