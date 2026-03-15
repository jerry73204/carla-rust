#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/client/LightManager.h"
#include "carla/rpc/LightState.h"
#include "carla_rust/client/light_state.hpp"
#include "carla_rust/client/light_list.hpp"
#include "carla_rust/client/result.hpp"
#include "carla_rust/sensor/data/color.hpp"
#include "light.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::LightId;
using carla::client::LightManager;
using carla::client::LightState;
using carla::rpc::EnvironmentObject;
using carla_rust::client::FfiClientLightState;
using carla_rust::rpc::FfiRpcLightGroup;
using carla_rust::sensor::data::FfiColor;

class FfiLightManager {
public:
    using LightGroup = carla::rpc::LightState::LightGroup;

    FfiLightManager(SharedPtr<LightManager>&& base) : inner_(std::move(base)) {}

    std::unique_ptr<FfiLightList> GetAllLights(FfiRpcLightGroup type, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLightList>(nullptr), [&]() {
            auto type_ = static_cast<LightGroup>(type);
            auto orig = inner_->GetAllLights(type_);
            return std::make_unique<FfiLightList>(std::move(orig));
        });
    }

    void TurnOnList(FfiLightList& lights, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->TurnOn(lights.inner()); });
    }

    void TurnOffList(FfiLightList& lights, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->TurnOff(lights.inner()); });
    }

    void SetActiveList(FfiLightList& lights, std::vector<bool>& active, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetActive(lights.inner(), active); });
    }

    std::vector<bool> IsActiveList(FfiLightList& lights, FfiError& error) const {
        return ffi_call(error, std::vector<bool>(),
                        [&]() { return inner_->IsActive(lights.inner()); });
    }

    std::unique_ptr<FfiLightList> GetTurnedOnLights(FfiRpcLightGroup type, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLightList>(nullptr), [&]() {
            auto type_ = static_cast<LightGroup>(type);
            auto orig = inner_->GetTurnedOnLights(type_);
            return std::make_unique<FfiLightList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiLightList> GetTurnedOffLights(FfiRpcLightGroup type, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLightList>(nullptr), [&]() {
            auto type_ = static_cast<LightGroup>(type);
            auto orig = inner_->GetTurnedOffLights(type_);
            return std::make_unique<FfiLightList>(std::move(orig));
        });
    }

    void SetColorList1(FfiLightList& lights, FfiColor color, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetColor(lights.inner(), color.as_builtin()); });
    }

    void SetColorList2(FfiLightList& lights, std::vector<Color>& colors, FfiError& error) {
        ffi_call_void(error, [&]() { inner_->SetColor(lights.inner(), colors); });
    }

    std::vector<Color> GetColorList(std::vector<Light>& lights, FfiError& error) const {
        return ffi_call(error, std::vector<Color>(), [&]() { return inner_->GetColor(lights); });
    }

    void SetIntensityList1(FfiLightList& lights, float intensity, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetIntensity(lights.inner(), intensity); });
    }

    void SetIntensityList2(FfiLightList& lights, std::vector<float>& intensities,
                           FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetIntensity(lights.inner(), intensities); });
    }

    std::vector<float> GetIntensityList(FfiLightList& lights, FfiError& error) const {
        return ffi_call(error, std::vector<float>(),
                        [&]() { return inner_->GetIntensity(lights.inner()); });
    }

    void SetLightGroupList1(FfiLightList& lights, FfiRpcLightGroup group, FfiError& error) const {
        ffi_call_void(error, [&]() {
            auto group_ = static_cast<LightGroup>(group);
            inner_->SetLightGroup(lights.inner(), group_);
        });
    }

    void SetLightGroupList2(FfiLightList& lights, std::vector<FfiRpcLightGroup>& groups,
                            FfiError& error) const {
        ffi_call_void(error, [&]() {
            std::vector<LightGroup> groups_;

            for (auto g : groups) {
                groups_.push_back(static_cast<LightGroup>(g));
            }

            inner_->SetLightGroup(lights.inner(), groups_);
        });
    }

    std::vector<FfiRpcLightGroup> GetLightGroupList(FfiLightList& lights, FfiError& error) const {
        return ffi_call(error, std::vector<FfiRpcLightGroup>(), [&]() {
            auto groups = inner_->GetLightGroup(lights.inner());

            std::vector<FfiRpcLightGroup> groups_;

            for (auto g : groups) {
                groups_.push_back(static_cast<FfiRpcLightGroup>(g));
            }

            return groups_;
        });
    }

    void SetLightStateList1(FfiLightList& lights, FfiClientLightState state,
                            FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetLightState(lights.inner(), state.as_builtin()); });
    }

    void SetLightStateList2(FfiLightList& lights, std::vector<FfiClientLightState>& states,
                            FfiError& error) {
        ffi_call_void(error, [&]() {
            std::vector<LightState>& casted = reinterpret_cast<std::vector<LightState>&>(states);
            inner_->SetLightState(lights.inner(), casted);
        });
    }

    std::vector<FfiClientLightState> GetLightStateList(FfiLightList& lights,
                                                       FfiError& error) const {
        return ffi_call(error, std::vector<FfiClientLightState>(), [&]() {
            auto orig = inner_->GetLightState(lights.inner());
            std::vector<FfiClientLightState> vec;

            for (auto&& item : orig) {
                vec.push_back(std::move(item));
            }

            return vec;
        });
    }

    FfiColor GetColor(uint32_t id, FfiError& error) const {
        return ffi_call(error, FfiColor(Color(0, 0, 0, 0)), [&]() {
            auto orig = inner_->GetColor(id);
            return FfiColor(std::move(orig));
        });
    }

    float GetIntensity(uint32_t id, FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_->GetIntensity(id); });
    }

    FfiClientLightState GetLightState(uint32_t id, FfiError& error) const {
        return ffi_call(error, FfiClientLightState(LightState()), [&]() {
            auto orig = inner_->GetLightState(id);
            return FfiClientLightState(std::move(orig));
        });
    }

    FfiRpcLightGroup GetLightGroup(uint32_t id, FfiError& error) const {
        return ffi_call(error, static_cast<FfiRpcLightGroup>(0), [&]() {
            auto group = inner_->GetLightGroup(id);
            return static_cast<FfiRpcLightGroup>(group);
        });
    }

    bool IsActive(uint32_t id, FfiError& error) const {
        return ffi_call(error, false, [&]() { return inner_->IsActive(id); });
    }

    void SetActive(uint32_t id, bool active, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetActive(id, active); });
    }

    void SetColor(uint32_t id, FfiColor color, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetColor(id, color.as_builtin()); });
    }

    void SetIntensity(uint32_t id, float intensity, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetIntensity(id, intensity); });
    }

    void SetLightState(uint32_t id, const FfiClientLightState& new_state, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetLightState(id, new_state.as_builtin()); });
    }

    void SetLightGroup(uint32_t id, FfiRpcLightGroup group, FfiError& error) const {
        ffi_call_void(error, [&]() {
            auto group_ = static_cast<LightGroup>(group);
            inner_->SetLightGroup(id, group_);
        });
    }

private:
    SharedPtr<LightManager> inner_;
};
}  // namespace client
}  // namespace carla_rust
