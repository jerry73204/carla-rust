#pragma once

#include <cstdint>
#include "carla/rpc/LightState.h"
#include "carla/rpc/Color.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/rpc/color.hpp"

namespace carla_rust
{
    namespace rpc {
        using carla::rpc::Color;
        using carla::rpc::LightId;
        using carla::rpc::LightState;
        using carla_rust::geom::FfiLocation;
        using carla_rust::rpc::FfiRpcColor;


        using FfiLightId = uint32_t;

        class FfiRpcLightState {
        public:
            FfiLocation location;
            float intensity;
            FfiLightId id;
            LightState::LightGroup group;
            FfiRpcColor color;
            bool active;

            FfiRpcLightState(LightState &&base) :
                location(std::move(base._location)),
                intensity(std::move(base._intensity)),
                id(std::move(base._id)),
                group(static_cast<LightState::LightGroup>(base._group)),
                color(std::move(base._color)),
                active(std::move(base._active))
            {}

            const LightState& as_native() const {
                return reinterpret_cast<const LightState&>(*this);
            }
        };

        static_assert(sizeof(FfiRpcLightState) == sizeof(LightState), "FfiRpcLightState size is incorrect");
    }
}
