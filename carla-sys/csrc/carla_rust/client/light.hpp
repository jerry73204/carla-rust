#pragma once

#include <cstdint>
#include <memory>
#include "carla/Memory.h"
#include "carla/client/Light.h"
#include "carla/client/LightState.h"
#include "carla_rust/client/light_state.hpp"
#include "carla_rust/client/result.hpp"
#include "carla_rust/sensor/data/color.hpp"

namespace carla_rust {
namespace rpc {
enum class FfiRpcLightGroup : uint8_t;
}

namespace client {
using carla::SharedPtr;
using carla::client::Light;
using LightId = carla::client::LightId;
using carla::client::LightState;
using carla_rust::client::FfiClientLightState;
using carla_rust::rpc::FfiRpcLightGroup;
using carla_rust::sensor::data::FfiColor;

class FfiLightRef {
public:
    FfiLightRef(Light& base) : inner_(base) {}

    FfiColor GetColor(FfiError& error) const {
        return ffi_call(error, FfiColor(), [&]() {
            auto orig = inner_.GetColor();
            return FfiColor(std::move(orig));
        });
    }

    uint32_t GetId() const { return inner_.GetId(); }

    float GetIntensity(FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_.GetIntensity(); });
    }

    const FfiLocation GetLocation(FfiError& error) const {
        return ffi_call(error, FfiLocation(), [&]() {
            auto orig = inner_.GetLocation();
            return FfiLocation(std::move(orig));
        });
    }

    FfiRpcLightGroup GetLightGroup(FfiError& error) const {
        return ffi_call(error, static_cast<FfiRpcLightGroup>(0), [&]() {
            auto group = inner_.GetLightGroup();
            return static_cast<FfiRpcLightGroup>(group);
        });
    }

    FfiClientLightState GetLightState(FfiError& error) const {
        return ffi_call(error, FfiClientLightState(), [&]() {
            LightState orig = inner_.GetLightState();
            return FfiClientLightState(std::move(orig));
        });
    }

    bool IsOn(FfiError& error) const {
        return ffi_call(error, false, [&]() { return inner_.IsOn(); });
    }

    bool IsOff(FfiError& error) const {
        return ffi_call(error, true, [&]() { return inner_.IsOff(); });
    }

    void SetColor(FfiColor color, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetColor(color.as_builtin()); });
    }

    void SetIntensity(float intensity, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetIntensity(intensity); });
    }

    void SetLightGroup(FfiRpcLightGroup group, FfiError& error) {
        ffi_call_void(error, [&]() {
            auto group_ = static_cast<carla::rpc::LightState::LightGroup>(group);
            inner_.SetLightGroup(group_);
        });
    }

    void SetLightState(const FfiClientLightState& state, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetLightState(state.as_builtin()); });
    }

    void TurnOn(FfiError& error) {
        ffi_call_void(error, [&]() { inner_.TurnOn(); });
    }

    void TurnOff(FfiError& error) {
        ffi_call_void(error, [&]() { inner_.TurnOff(); });
    }

private:
    Light& inner_;
};
}  // namespace client
}  // namespace carla_rust
