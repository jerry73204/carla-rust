#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Vehicle.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla/rpc/VehicleDoor.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/VehicleWheels.h"
#include "carla/rpc/TrafficLightState.h"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Vehicle;
        using carla::rpc::VehicleControl;
        using carla::rpc::VehiclePhysicsControl;
        using carla::rpc::VehicleDoor;
        using carla::rpc::VehicleLightState;
        using carla::rpc::VehicleWheelLocation;
        using carla::rpc::TrafficLightState;

        // Vehicle
        class FfiVehicle {
        public:
            FfiVehicle(SharedPtr<Vehicle> &&base)
                : inner_(std::move(base))
            {}


            void SetAutopilot(bool enabled = true, uint16_t tm_port = TM_DEFAULT_PORT) const {
                inner_->SetAutopilot(enabled, tm_port);
            }

            void ShowDebugTelemetry(bool enabled = true) const {
                inner_->ShowDebugTelemetry(enabled);
            }

            /// Apply @a control to this vehicle.
            void ApplyControl(const VehicleControl &control) const {
                inner_->ApplyControl(control);
            }

            void ApplyPhysicsControl(const VehiclePhysicsControl &physics_control) const {
                inner_->ApplyPhysicsControl(physics_control);
            }

            void OpenDoor(const VehicleDoor door_idx) const {
                inner_->OpenDoor(door_idx);
            }

            void CloseDoor(const VehicleDoor door_idx) const {
                inner_->CloseDoor(door_idx);
            }

            void SetLightState(const VehicleLightState::LightState &light_state) const {
                inner_->SetLightState(light_state);
            }

            void SetWheelSteerDirection(VehicleWheelLocation wheel_location, float angle_in_deg) const {
                inner_->SetWheelSteerDirection(wheel_location, angle_in_deg);
            }

            float GetWheelSteerAngle(VehicleWheelLocation wheel_location) const {
                return inner_->GetWheelSteerAngle(wheel_location);
            }

            VehicleControl GetControl() const {
                return inner_->GetControl();
            }

            VehiclePhysicsControl GetPhysicsControl() const {
                return inner_->GetPhysicsControl();
            }

            VehicleLightState::LightState GetLightState() const {
                return inner_->GetLightState();
            }

            float GetSpeedLimit() const {
                return inner_->GetSpeedLimit();
            }

            TrafficLightState GetTrafficLightState() const {
                return inner_->GetTrafficLightState();
            }

            bool IsAtTrafficLight() const {
                return inner_->IsAtTrafficLight();
            }

            // SharedPtr<TrafficLight> GetTrafficLight() const {}

            void EnableCarSim(std::string simfile_path) const {
                inner_->EnableCarSim(simfile_path);
            }

            void UseCarSimRoad(bool enabled) const {
                inner_->UseCarSimRoad(enabled);
            }

            void EnableChronoPhysics(uint64_t MaxSubsteps,
                                     float MaxSubstepDeltaTime,
                                     std::string VehicleJSON = "",
                                     std::string PowertrainJSON = "",
                                     std::string TireJSON = "",
                                     std::string BaseJSONPath = "") const {
                inner_->EnableChronoPhysics(MaxSubsteps,
                                           MaxSubstepDeltaTime,
                                           VehicleJSON,
                                           PowertrainJSON,
                                           TireJSON,
                                           BaseJSONPath);
            }

            std::shared_ptr<FfiActor> to_actor() const {
                SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
                return std::make_shared<FfiActor>(std::move(ptr));
            }

        private:
            SharedPtr<Vehicle> inner_;
        };
    }
} // namespace carla_rust