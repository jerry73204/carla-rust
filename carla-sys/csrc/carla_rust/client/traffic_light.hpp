#pragma once

#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla_rust/compat.hpp"
#include "carla/road/RoadTypes.h"
#include "carla/geom/BoundingBox.h"
#include "carla/client/TrafficLight.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/client/waypoint_list.hpp"
#include "carla_rust/client/traffic_light_list.hpp"
#include "carla_rust/client/bounding_box_list.hpp"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::TrafficLight;
using carla::geom::BoundingBox;
using carla::road::SignId;
using carla::rpc::TrafficLightState;
using carla_rust::client::FfiBoundingBoxList;
using carla_rust::client::FfiTrafficLightList;
using carla_rust::client::FfiWaypointList;
using carla_rust::geom::FfiBoundingBox;

// TrafficLight
class FfiTrafficLight {
public:
    FfiTrafficLight(SharedPtr<TrafficLight>&& base) noexcept : inner_(std::move(base)) {}

    FfiBoundingBox GetTriggerVolume(FfiError& error) const {
        return ffi_call(error, FfiBoundingBox(), [&]() {
            const BoundingBox& orig = inner_->GetTriggerVolume();
            return reinterpret_cast<const FfiBoundingBox&>(orig);
        });
    }

    std::unique_ptr<std::string> GetSignId(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<std::string>(nullptr),
                        [&]() { return std::make_unique<std::string>(inner_->GetSignId()); });
    }

    void SetState(TrafficLightState state, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetState(state); });
    }

    TrafficLightState GetState(FfiError& error) const {
        return ffi_call(error, TrafficLightState::Off, [&]() { return inner_->GetState(); });
    }

    void SetGreenTime(float green_time, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetGreenTime(green_time); });
    }

    float GetGreenTime(FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_->GetGreenTime(); });
    }

    void SetYellowTime(float yellow_time, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetYellowTime(yellow_time); });
    }

    float GetYellowTime(FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_->GetYellowTime(); });
    }

    void SetRedTime(float red_time, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetRedTime(red_time); });
    }

    float GetRedTime(FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_->GetRedTime(); });
    }

    float GetElapsedTime(FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_->GetElapsedTime(); });
    }

    void Freeze(bool freeze, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->Freeze(freeze); });
    }

    bool IsFrozen(FfiError& error) const {
        return ffi_call(error, false, [&]() { return inner_->IsFrozen(); });
    }

    uint32_t GetPoleIndex(FfiError& error) const {
        return ffi_call(error, uint32_t(0), [&]() { return inner_->GetPoleIndex(); });
    }

    std::unique_ptr<FfiTrafficLightList> GetGroupTrafficLights(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiTrafficLightList>(nullptr), [&]() {
            auto orig = inner_->GetGroupTrafficLights();
            return std::make_unique<FfiTrafficLightList>(std::move(orig));
        });
    }

    void ResetGroup(FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->ResetGroup(); });
    }

    std::unique_ptr<FfiWaypointList> GetAffectedLaneWaypoints(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWaypointList>(nullptr), [&]() {
            auto orig = inner_->GetAffectedLaneWaypoints();
            return std::make_unique<FfiWaypointList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiBoundingBoxList> GetLightBoxes(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiBoundingBoxList>(nullptr), [&]() {
            auto orig = inner_->GetLightBoxes();
            return std::make_unique<FfiBoundingBoxList>(std::move(orig));
        });
    }

    std::unique_ptr<std::string> GetOpenDRIVEID(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<std::string>(nullptr),
                        [&]() { return std::make_unique<std::string>(inner_->GetOpenDRIVEID()); });
    }

    std::unique_ptr<FfiWaypointList> GetStopWaypoints(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWaypointList>(nullptr), [&]() {
            auto orig = inner_->GetStopWaypoints();
            return std::make_unique<FfiWaypointList>(std::move(orig));
        });
    }

    std::shared_ptr<FfiActor> to_actor() const noexcept {
        SharedPtr<Actor> ptr = carla_static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<TrafficLight> inner_;
};
}  // namespace client
}  // namespace carla_rust
