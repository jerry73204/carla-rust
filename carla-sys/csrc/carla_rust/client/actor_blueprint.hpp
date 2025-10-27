#pragma once

#include <memory>

namespace carla_rust {
namespace client {
// functions
ActorBlueprint copy_actor_blueprint(const ActorBlueprint& ref) {
    return ref;
}
}  // namespace client
}  // namespace carla_rust
