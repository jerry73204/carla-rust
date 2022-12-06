#pragma once

#include <memory>

namespace carla_rust
{
    namespace client {
        // functions
        ActorBlueprint copy_actor_blueprint(const ActorBlueprint &ref) {
            return ref;
        }
    }
} // namespace carla_rust
