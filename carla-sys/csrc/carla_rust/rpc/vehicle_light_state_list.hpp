#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/VehicleLightStateList.h"
#include "carla/rpc/ActorId.h"
#include "carla_rust/rpc/actor_id.hpp"

namespace carla_rust {
namespace rpc {
using carla::rpc::ActorId;
using carla::rpc::VehicleLightState;
using carla::rpc::VehicleLightStateList;
using carla_rust::rpc::FfiActorId;

class FfiVehicleLightStateElementRef {
public:
    FfiVehicleLightStateElementRef(const std::pair<ActorId, VehicleLightState::flag_type>& elem)
        : inner_(elem) {}

    FfiActorId id() const { return std::get<0>(inner_); }

    VehicleLightState::LightState light_state() const {
        auto flag = std::get<1>(inner_);
        VehicleLightState state(flag);
        return state.GetLightStateEnum();
    }

private:
    const std::pair<ActorId, VehicleLightState::flag_type>& inner_;
};

class FfiVehicleLightStateList {
public:
    FfiVehicleLightStateList(VehicleLightStateList&& vec) : inner_(std::move(vec)) {}

    size_t len() const { return inner_.size(); }

    std::unique_ptr<FfiVehicleLightStateElementRef> get(size_t index) const {
        auto orig = inner_.at(index);
        return std::make_unique<FfiVehicleLightStateElementRef>(orig);
    }

private:
    VehicleLightStateList inner_;
};
}  // namespace rpc
}  // namespace carla_rust
