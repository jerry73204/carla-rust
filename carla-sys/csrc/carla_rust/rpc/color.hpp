#pragma once

#include <cstdint>
#include "carla/rpc/Color.h"

namespace carla_rust {
namespace rpc {
using carla::rpc::Color;

struct FfiRpcColor {
public:
    uint8_t r;
    uint8_t g;
    uint8_t b;

    FfiRpcColor(Color&& base) : r(base.r), g(base.g), b(base.b) {}

    Color to_native() const { return Color(r, g, b); }
};

static_assert(sizeof(FfiRpcColor) == sizeof(Color), "FfiRpcColor size is incorrect");
}  // namespace rpc
}  // namespace carla_rust
