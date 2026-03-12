#pragma once

// Compatibility header: GearPhysicsControl was removed in CARLA 0.10.0
// For 0.9.x, include the real header. For 0.10.0, provide a stub so
// autocxx can generate bindings without failing.

#ifdef CARLA_VERSION_0100

#include "carla/MsgPack.h"

namespace carla {
namespace rpc {

// Stub type for autocxx binding generation.
// Not used at runtime in 0.10.0 (gear ratios are flat vectors instead).
struct GearPhysicsControl {
    float ratio = 1.0f;
    float down_ratio = 0.5f;
    float up_ratio = 0.65f;

    bool operator==(const GearPhysicsControl&) const = default;
    bool operator!=(const GearPhysicsControl&) const = default;

    MSGPACK_DEFINE_ARRAY(ratio, down_ratio, up_ratio)
};

}  // namespace rpc
}  // namespace carla

#else

#include "carla/rpc/GearPhysicsControl.h"

#endif
