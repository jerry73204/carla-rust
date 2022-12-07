#pragma once

#include <cstdint>
#include "carla/sensor/data/Color.h"

namespace carla_rust
{
    namespace sensor {
        namespace data {
            using carla::sensor::data::Color;

            // Color
            class FfiColor {
            public:
                uint8_t b;
                uint8_t g;
                uint8_t r;
                uint8_t a;

                FfiColor(Color &&base)
                    :
                    b(std::move(base.b)),
                    g(std::move(base.g)),
                    r(std::move(base.r)),
                    a(std::move(base.a))
                {}

                Color& as_builtin() {
                    return reinterpret_cast<Color&>(*this);
                }
            };
            static_assert(sizeof(FfiColor) == sizeof(Color), "Invalid FfiColor size!");
       }
    }
}
