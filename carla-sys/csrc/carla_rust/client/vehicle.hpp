#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla_rust/compat.hpp"
#include "carla/client/Vehicle.h"
#include "carla/rpc/AckermannControllerSettings.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla/rpc/VehicleDoor.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/VehicleWheels.h"
#include "carla/rpc/VehicleAckermannControl.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla/rpc/VehicleFailureState.h"
#ifdef CARLA_VERSION_0916
#include "carla/rpc/VehicleTelemetryData.h"
#endif
#include "carla/client/TrafficLight.h"
#include "carla_rust/client/traffic_light.hpp"
#include "carla_rust/client/result.hpp"
#include "carla_rust/rpc/vehicle_physics_control.hpp"
#include "carla_rust/rpc/vehicle_telemetry_data.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Vehicle;
using carla::rpc::AckermannControllerSettings;
using carla::rpc::TrafficLightState;
using carla::rpc::VehicleAckermannControl;
using carla::rpc::VehicleControl;
using carla::rpc::VehicleDoor;
using carla::rpc::VehicleLightState;
using carla::rpc::VehiclePhysicsControl;
using carla::rpc::VehicleWheelLocation;
using carla_rust::rpc::FfiVehiclePhysicsControl;

// Vehicle
class FfiVehicle {
public:
    FfiVehicle(SharedPtr<Vehicle>&& base) : inner_(std::move(base)) {}

    void SetAutopilot(bool enabled, uint16_t tm_port, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetAutopilot(enabled, tm_port); });
    }

    void ShowDebugTelemetry(bool enabled, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->ShowDebugTelemetry(enabled); });
    }

    void ApplyControl(const VehicleControl& control, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->ApplyControl(control); });
    }

    void ApplyAckermannControl(const VehicleAckermannControl& control, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->ApplyAckermannControl(control); });
    }

    void ApplyPhysicsControl(const FfiVehiclePhysicsControl& physics_control,
                             FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->ApplyPhysicsControl(physics_control.inner()); });
    }

    AckermannControllerSettings GetAckermannControllerSettings(FfiError& error) const {
        return ffi_call(error, AckermannControllerSettings(),
                        [&]() { return inner_->GetAckermannControllerSettings(); });
    }

    void ApplyAckermannControllerSettings(const AckermannControllerSettings& settings,
                                          FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->ApplyAckermannControllerSettings(settings); });
    }

    void OpenDoor(const VehicleDoor door_idx, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->OpenDoor(door_idx); });
    }

    void CloseDoor(const VehicleDoor door_idx, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->CloseDoor(door_idx); });
    }

    void SetLightState(const VehicleLightState::LightState& light_state, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetLightState(light_state); });
    }

    void SetWheelSteerDirection(VehicleWheelLocation wheel_location, float angle_in_deg,
                                FfiError& error) const {
        ffi_call_void(error,
                      [&]() { inner_->SetWheelSteerDirection(wheel_location, angle_in_deg); });
    }

    float GetWheelSteerAngle(VehicleWheelLocation wheel_location, FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_->GetWheelSteerAngle(wheel_location); });
    }

    VehicleControl GetControl() const { return inner_->GetControl(); }

    std::unique_ptr<FfiVehiclePhysicsControl> GetPhysicsControl(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiVehiclePhysicsControl>(nullptr), [&]() {
            auto orig = inner_->GetPhysicsControl();
            return std::make_unique<FfiVehiclePhysicsControl>(std::move(orig));
        });
    }

    VehicleLightState::LightState GetLightState() const { return inner_->GetLightState(); }

    float GetSpeedLimit() const { return inner_->GetSpeedLimit(); }

    TrafficLightState GetTrafficLightState() const { return inner_->GetTrafficLightState(); }

    bool IsAtTrafficLight() const { return inner_->IsAtTrafficLight(); }

    carla::rpc::VehicleFailureState GetFailureState() const { return inner_->GetFailureState(); }

    FfiBoundingBox GetBoundingBox() const {
        auto orig = inner_->GetBoundingBox();
        return FfiBoundingBox(std::move(orig));
    }

    std::shared_ptr<FfiTrafficLight> GetTrafficLight(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiTrafficLight>(nullptr),
                        [&]() -> std::shared_ptr<FfiTrafficLight> {
                            auto tl = inner_->GetTrafficLight();
                            if (tl == nullptr) {
                                return nullptr;
                            }
                            return std::make_shared<FfiTrafficLight>(std::move(tl));
                        });
    }

#ifdef CARLA_VERSION_0916
    std::unique_ptr<carla_rust::rpc::FfiVehicleTelemetryData> GetTelemetryData(
        FfiError& error) const {
        return ffi_call(error, std::unique_ptr<carla_rust::rpc::FfiVehicleTelemetryData>(nullptr),
                        [&]() {
                            return std::make_unique<carla_rust::rpc::FfiVehicleTelemetryData>(
                                inner_->GetTelemetryData());
                        });
    }

    void SetWheelPitchAngle(VehicleWheelLocation wheel_location, float angle_in_deg,
                            FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetWheelPitchAngle(wheel_location, angle_in_deg); });
    }

    float GetWheelPitchAngle(VehicleWheelLocation wheel_location, FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_->GetWheelPitchAngle(wheel_location); });
    }

    std::vector<carla::geom::Transform> GetVehicleBoneWorldTransforms(FfiError& error) const {
        return ffi_call(error, std::vector<carla::geom::Transform>(),
                        [&]() { return inner_->GetVehicleBoneWorldTransforms(); });
    }

    void RestorePhysXPhysics(FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->RestorePhysXPhysics(); });
    }
#endif

    void EnableCarSim(std::string simfile_path, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->EnableCarSim(std::move(simfile_path)); });
    }

    void UseCarSimRoad(bool enabled, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->UseCarSimRoad(enabled); });
    }

    void EnableChronoPhysics(uint64_t MaxSubsteps, float MaxSubstepDeltaTime,
                             std::string VehicleJSON, std::string PowertrainJSON,
                             std::string TireJSON, std::string BaseJSONPath,
                             FfiError& error) const {
        ffi_call_void(error, [&]() {
            inner_->EnableChronoPhysics(MaxSubsteps, MaxSubstepDeltaTime, std::move(VehicleJSON),
                                        std::move(PowertrainJSON), std::move(TireJSON),
                                        std::move(BaseJSONPath));
        });
    }

    std::shared_ptr<FfiActor> to_actor() const {
        SharedPtr<Actor> ptr = carla_static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<Vehicle> inner_;
};
}  // namespace client
}  // namespace carla_rust
