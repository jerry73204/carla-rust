#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/client/LightState.h"
#include "carla/rpc/LightState.h"
#include "carla_rust/sensor/data/color.hpp"

namespace carla_rust
{
    namespace client {
        using carla::client::LightState;
        using carla_rust::sensor::data::FfiColor;

        class FfiClientLightState {
        public:
            float intensity;
            FfiColor color;
            carla::rpc::LightState::LightGroup group;
            bool active;

            FfiClientLightState(LightState &&base) :
                intensity(std::move(base._intensity)),
                color(std::move(base._color)),
                group(std::move(base._group)),
                active(std::move(base._active))
            {}

            const LightState& as_builtin() const {
                return reinterpret_cast<const LightState&>(*this);
            }
        };

        static_assert(sizeof(FfiClientLightState) == sizeof(LightState), "FfiClientLightState size is not correct");
    }
}
