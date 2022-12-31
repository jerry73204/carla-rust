#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/client/LightManager.h"
#include "carla/rpc/LightState.h"
#include "carla_rust/client/light_state.hpp"
#include "carla_rust/client/light_list.hpp"
#include "carla_rust/sensor/data/color.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::rpc::EnvironmentObject;
        using carla::client::LightId;
        using carla::client::LightManager;
        using carla::client::LightState;
        using carla_rust::client::FfiLightState;
        using carla_rust::sensor::data::FfiColor;

        class FfiLightManager {
        public:
            using LightGroup = carla::rpc::LightState::LightGroup;

            FfiLightManager(SharedPtr<LightManager> &&base)
                : inner_(std::move(base))
            {}

            FfiLightList GetAllLights(carla::rpc::LightState::LightGroup type) const {
                auto orig = inner_->GetAllLights(type);
                return FfiLightList(std::move(orig));
            }

            void TurnOnList(FfiLightList& lights) const {
                inner_->TurnOn(lights.inner());
            }

            void TurnOffList(FfiLightList& lights) const {
                inner_->TurnOff(lights.inner());
            }

            void SetActiveList(FfiLightList& lights, std::vector<bool>& active) const {
                inner_->SetActive(lights.inner(), active);
            }

            std::vector<bool> IsActiveList(FfiLightList& lights) const {
                return inner_->IsActive(lights.inner());
            }

            FfiLightList GetTurnedOnLights(carla::rpc::LightState::LightGroup type) const {
                auto orig = inner_->GetTurnedOnLights(type);
                return FfiLightList(std::move(orig));
            }

            FfiLightList GetTurnedOffLights(carla::rpc::LightState::LightGroup type) const {
                auto orig = inner_->GetTurnedOffLights(type);
                return FfiLightList(std::move(orig));
            }

            void SetColorList1(FfiLightList& lights, FfiColor color) const {
                inner_->SetColor(lights.inner(), color.as_builtin());
            }

            void SetColorList2(FfiLightList& lights, std::vector<Color>& colors) {
                inner_->SetColor(lights.inner(), colors);
            }

            std::vector<Color> GetColor(std::vector<Light>& lights) const {
                return inner_->GetColor(lights);
            }

            void SetIntensityList1(FfiLightList& lights, float intensity) const {
                inner_->SetIntensity(lights.inner(), intensity);
            }

            void SetIntensityList2(FfiLightList& lights, std::vector<float>& intensities) const {
                inner_->SetIntensity(lights.inner(), intensities);
            }

            std::vector<float> GetIntensityList(FfiLightList& lights) const {
                return inner_->GetIntensity(lights.inner());
            }

            void SetLightGroupList1(FfiLightList& lights, carla::rpc::LightState::LightGroup group) const {
                inner_->SetLightGroup(lights.inner(), group);
            }

            void SetLightGroupList2(FfiLightList& lights, std::vector<carla::rpc::LightState::LightGroup>& groups) const {
                inner_->SetLightGroup(lights.inner(), groups);
            }

            std::vector<carla::rpc::LightState::LightGroup> GetLightGroupList(FfiLightList& lights) const {
                inner_->GetLightGroup(lights.inner());
            }

            void SetLightStateList1(FfiLightList& lights, FfiLightState state) const {
                inner_->SetLightState(lights.inner(), state.as_builtin());
            }

            void SetLightStateList2(FfiLightList& lights, std::vector<FfiLightState>& states) {
                std::vector<LightState>& casted = reinterpret_cast<std::vector<LightState>&>(states);
                inner_->SetLightState(lights.inner(), casted);
            }

            std::vector<FfiLightState> GetLightStateList(FfiLightList& lights) const {
                auto orig = inner_->GetLightState(lights.inner());
                std::vector<FfiLightState> vec;

                for (auto&& item: orig) {
                    vec.push_back(std::move(item));
                }

                return vec;
            }

            FfiColor GetColor(uint32_t id) const {
                auto orig = inner_->GetColor(id);
                return FfiColor(std::move(orig));
            }

            float GetIntensity(uint32_t id) const {
                return inner_->GetIntensity(id);
            }

            FfiLightState GetLightState(uint32_t id) const {
                auto orig = inner_->GetLightState(id);
                return FfiLightState(std::move(orig));
            }

            carla::rpc::LightState::LightGroup GetLightGroup(uint32_t id) const {
                return inner_->GetLightGroup(id);
            }

            bool IsActive(uint32_t id) const {
                return inner_->IsActive(id);
            }

            void SetActive(uint32_t id, bool active) const {
                inner_->SetActive(id, active);
            }

            void SetColor(uint32_t id, FfiColor color) const {
                inner_->SetColor(id, color.as_builtin());
            }

            void SetIntensity(uint32_t id, float intensity) const {
                inner_->SetIntensity(id, intensity);
            }

            void SetLightState(uint32_t id, const FfiLightState& new_state) const {
                inner_->SetLightState(id, new_state.as_builtin());
            }

            void SetLightGroup(uint32_t id, carla::rpc::LightState::LightGroup group) const {
                inner_->SetLightGroup(id, group);
            }


        private:
            SharedPtr<LightManager> inner_;
        };
    }
}
