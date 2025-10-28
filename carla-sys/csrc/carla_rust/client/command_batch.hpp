#pragma once

#include "carla/client/ActorBlueprint.h"
#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Vector3D.h"
#include "carla/rpc/ActorDescription.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/Command.h"
#include "carla/rpc/CommandResponse.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla/rpc/VehicleAckermannControl.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla/rpc/WalkerControl.h"
#include "../geom.hpp"
#include "../rpc/vehicle_physics_control.hpp"
#include <vector>

namespace carla_rust {
namespace client {

using carla::geom::Location;
using carla::geom::Transform;
using carla::geom::Vector3D;
using carla::rpc::ActorDescription;
using carla::rpc::ActorId;
using carla::rpc::Command;
using carla::rpc::CommandResponse;
using carla::rpc::TrafficLightState;
using carla::rpc::VehicleAckermannControl;
using carla::rpc::VehicleControl;
using carla::rpc::VehicleLightState;
using carla::rpc::VehiclePhysicsControl;
using carla::rpc::WalkerControl;

/// FFI wrapper for accumulating batch commands.
class FfiCommandBatch {
public:
    FfiCommandBatch() = default;

    // Spawn actor
    void AddSpawnActor(ActorDescription&& description, const geom::FfiTransform& transform,
                       uint32_t parent_id) {
        if (parent_id == 0) {
            commands_.emplace_back(
                Command::SpawnActor{std::move(description), transform.as_native()});
        } else {
            commands_.emplace_back(
                Command::SpawnActor{std::move(description), transform.as_native(), parent_id});
        }
    }

    // Destroy actor
    void AddDestroyActor(uint32_t actor_id) {
        commands_.emplace_back(Command::DestroyActor{actor_id});
    }

    // Apply vehicle control
    void AddApplyVehicleControl(uint32_t actor_id, const VehicleControl& control) {
        commands_.emplace_back(Command::ApplyVehicleControl{actor_id, control});
    }

    // Apply vehicle Ackermann control
    void AddApplyVehicleAckermannControl(uint32_t actor_id,
                                         const VehicleAckermannControl& control) {
        commands_.emplace_back(Command::ApplyVehicleAckermannControl{actor_id, control});
    }

    // Apply walker control
    void AddApplyWalkerControl(uint32_t actor_id, const WalkerControl& control) {
        commands_.emplace_back(Command::ApplyWalkerControl{actor_id, control});
    }

    // Apply vehicle physics control
    void AddApplyVehiclePhysicsControl(
        uint32_t actor_id, const carla_rust::rpc::FfiVehiclePhysicsControl& ffi_physics) {
        commands_.emplace_back(
            Command::ApplyVehiclePhysicsControl{actor_id, ffi_physics.as_native()});
    }

    // Apply transform
    void AddApplyTransform(uint32_t actor_id, const geom::FfiTransform& transform) {
        commands_.emplace_back(Command::ApplyTransform{actor_id, transform.as_native()});
    }

    // Apply location
    void AddApplyLocation(uint32_t actor_id, const geom::FfiLocation& location) {
        commands_.emplace_back(Command::ApplyLocation{actor_id, location.as_native()});
    }

    // Apply walker state
    void AddApplyWalkerState(uint32_t actor_id, const geom::FfiTransform& transform, float speed) {
        commands_.emplace_back(Command::ApplyWalkerState{actor_id, transform.as_native(), speed});
    }

    // Apply target velocity
    void AddApplyTargetVelocity(uint32_t actor_id, const Vector3D& velocity) {
        commands_.emplace_back(Command::ApplyTargetVelocity{actor_id, velocity});
    }

    // Apply target angular velocity
    void AddApplyTargetAngularVelocity(uint32_t actor_id, const Vector3D& angular_velocity) {
        commands_.emplace_back(Command::ApplyTargetAngularVelocity{actor_id, angular_velocity});
    }

    // Apply impulse
    void AddApplyImpulse(uint32_t actor_id, const Vector3D& impulse) {
        commands_.emplace_back(Command::ApplyImpulse{actor_id, impulse});
    }

    // Apply force
    void AddApplyForce(uint32_t actor_id, const Vector3D& force) {
        commands_.emplace_back(Command::ApplyForce{actor_id, force});
    }

    // Apply angular impulse
    void AddApplyAngularImpulse(uint32_t actor_id, const Vector3D& impulse) {
        commands_.emplace_back(Command::ApplyAngularImpulse{actor_id, impulse});
    }

    // Apply torque
    void AddApplyTorque(uint32_t actor_id, const Vector3D& torque) {
        commands_.emplace_back(Command::ApplyTorque{actor_id, torque});
    }

    // Set simulate physics
    void AddSetSimulatePhysics(uint32_t actor_id, bool enabled) {
        commands_.emplace_back(Command::SetSimulatePhysics{actor_id, enabled});
    }

    // Set enable gravity
    void AddSetEnableGravity(uint32_t actor_id, bool enabled) {
        commands_.emplace_back(Command::SetEnableGravity{actor_id, enabled});
    }

    // Set autopilot
    void AddSetAutopilot(uint32_t actor_id, bool enabled, uint16_t tm_port) {
        commands_.emplace_back(Command::SetAutopilot{actor_id, enabled, tm_port});
    }

    // Show debug telemetry
    void AddShowDebugTelemetry(uint32_t actor_id, bool enabled) {
        commands_.emplace_back(Command::ShowDebugTelemetry{actor_id, enabled});
    }

    // Set vehicle light state
    void AddSetVehicleLightState(uint32_t actor_id, uint32_t light_state) {
        commands_.emplace_back(Command::SetVehicleLightState{actor_id, light_state});
    }

    // Console command
    void AddConsoleCommand(const std::string& cmd) {
        commands_.emplace_back(Command::ConsoleCommand{cmd});
    }

    // Set traffic light state
    void AddSetTrafficLightState(uint32_t actor_id, uint8_t state) {
        commands_.emplace_back(
            Command::SetTrafficLightState{actor_id, static_cast<TrafficLightState>(state)});
    }

    // Get the accumulated commands (consumes the batch)
    std::vector<Command> TakeCommands() { return std::move(commands_); }

    // Get the number of commands in the batch
    size_t Size() const { return commands_.size(); }

    // Clear all commands
    void Clear() { commands_.clear(); }

private:
    std::vector<Command> commands_;
};

// FFI wrapper for command response
struct FfiCommandResponse {
    bool HasError() const { return has_error_; }

    std::string GetErrorMessage() const { return error_message_; }

    uint32_t GetActorId() const { return actor_id_; }

    static FfiCommandResponse FromNative(const CommandResponse& response) {
        FfiCommandResponse ffi_response;
        ffi_response.has_error_ = response.HasError();
        if (ffi_response.has_error_) {
            ffi_response.error_message_ = response.GetError().What();
            ffi_response.actor_id_ = 0;
        } else {
            ffi_response.error_message_ = "";
            ffi_response.actor_id_ = response.Get();
        }
        return ffi_response;
    }

private:
    bool has_error_;
    std::string error_message_;
    uint32_t actor_id_;
};

}  // namespace client
}  // namespace carla_rust
