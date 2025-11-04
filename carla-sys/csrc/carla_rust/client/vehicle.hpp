#pragma once

#include <memory>
#include "carla/Memory.h"
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

    void SetAutopilot(bool enabled = true, uint16_t tm_port = TM_DEFAULT_PORT) const {
        inner_->SetAutopilot(enabled, tm_port);
    }

    void ShowDebugTelemetry(bool enabled = true) const { inner_->ShowDebugTelemetry(enabled); }

    void ApplyControl(const VehicleControl& control) const { inner_->ApplyControl(control); }

    void ApplyAckermannControl(const VehicleAckermannControl& control) const {
        inner_->ApplyAckermannControl(control);
    }

    void ApplyPhysicsControl(const FfiVehiclePhysicsControl& physics_control) const {
        inner_->ApplyPhysicsControl(physics_control.inner());
    }

    AckermannControllerSettings GetAckermannControllerSettings() const {
        return inner_->GetAckermannControllerSettings();
    }

    void ApplyAckermannControllerSettings(const AckermannControllerSettings& settings) const {
        inner_->ApplyAckermannControllerSettings(settings);
    }

    void OpenDoor(const VehicleDoor door_idx) const { inner_->OpenDoor(door_idx); }

    void CloseDoor(const VehicleDoor door_idx) const { inner_->CloseDoor(door_idx); }

    void SetLightState(const VehicleLightState::LightState& light_state) const {
        inner_->SetLightState(light_state);
    }

    void SetWheelSteerDirection(VehicleWheelLocation wheel_location, float angle_in_deg) const {
        inner_->SetWheelSteerDirection(wheel_location, angle_in_deg);
    }

    float GetWheelSteerAngle(VehicleWheelLocation wheel_location) const {
        return inner_->GetWheelSteerAngle(wheel_location);
    }

    VehicleControl GetControl() const { return inner_->GetControl(); }

    FfiVehiclePhysicsControl GetPhysicsControl() const {
        auto orig = inner_->GetPhysicsControl();
        return FfiVehiclePhysicsControl(std::move(orig));
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

    // SharedPtr<TrafficLight> GetTrafficLight() const {}

#ifdef CARLA_VERSION_0916
    carla_rust::rpc::FfiVehicleTelemetryData GetTelemetryData() const {
        return carla_rust::rpc::FfiVehicleTelemetryData(inner_->GetTelemetryData());
    }

    void SetWheelPitchAngle(VehicleWheelLocation wheel_location, float angle_in_deg) const {
        inner_->SetWheelPitchAngle(wheel_location, angle_in_deg);
    }

    float GetWheelPitchAngle(VehicleWheelLocation wheel_location) const {
        return inner_->GetWheelPitchAngle(wheel_location);
    }

    std::vector<carla::geom::Transform> GetVehicleBoneWorldTransforms() const {
        return inner_->GetVehicleBoneWorldTransforms();
    }

    void RestorePhysXPhysics() const { inner_->RestorePhysXPhysics(); }
#endif

    void EnableCarSim(std::string simfile_path) const {
        inner_->EnableCarSim(std::move(simfile_path));
    }

    void UseCarSimRoad(bool enabled) const { inner_->UseCarSimRoad(enabled); }

    void EnableChronoPhysics(uint64_t MaxSubsteps, float MaxSubstepDeltaTime,
                             std::string VehicleJSON = "", std::string PowertrainJSON = "",
                             std::string TireJSON = "", std::string BaseJSONPath = "") const {
        inner_->EnableChronoPhysics(MaxSubsteps, MaxSubstepDeltaTime, std::move(VehicleJSON),
                                    std::move(PowertrainJSON), std::move(TireJSON),
                                    std::move(BaseJSONPath));
    }

    std::shared_ptr<FfiActor> to_actor() const {
        SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<Vehicle> inner_;
};
}  // namespace client
}  // namespace carla_rust
