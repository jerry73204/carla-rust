#pragma once

#include <cstdint>
#include <memory>
#include "carla/Memory.h"
#include "carla/client/Light.h"
#include "carla/client/LightState.h"
#include "carla_rust/client/light_state.hpp"
#include "carla_rust/sensor/data/color.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Light;
        using LightId = carla::client::LightId;
        using carla::client::LightState;
        using carla_rust::client::FfiClientLightState;
        using carla_rust::sensor::data::FfiColor;

        class FfiLightRef {
        public:
            FfiLightRef(Light &base)
                : inner_(base)
            {}


            FfiColor GetColor() const {
                auto orig = inner_.GetColor();
                return FfiColor(std::move(orig));
            }

            uint32_t GetId() const {
                return inner_.GetId();
            }

            float GetIntensity() const {
                return inner_.GetIntensity();
            }

            const FfiLocation GetLocation() const {
                auto orig = inner_.GetLocation();
                return FfiLocation(std::move(orig));
            }

            carla::rpc::LightState::LightGroup GetLightGroup() const {
                return inner_.GetLightGroup();
            }

            std::unique_ptr<FfiClientLightState> GetLightState() const {
                LightState orig = inner_.GetLightState();
                return std::make_unique<FfiClientLightState>(std::move(orig));
            }

            bool IsOn() const {
                return inner_.IsOn();
            }

            bool IsOff() const {
                return inner_.IsOff();
            }

            void SetColor(FfiColor color) {
                inner_.SetColor(color.as_builtin());
            }

            void SetIntensity(float intensity) {
                inner_.SetIntensity(intensity);
            }

            void SetLightGroup(carla::rpc::LightState::LightGroup group) {
                inner_.SetLightGroup(group);
            }

            void SetLightState(const FfiClientLightState& state) {
                inner_.SetLightState(state.as_builtin());
            }

            void TurnOn() {
                inner_.TurnOn();
            }

            void TurnOff() {
                inner_.TurnOff();
            }

        private:
            Light& inner_;
        };
    }
} // namespace carla_rust
