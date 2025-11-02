//! Batch command system for efficient multi-actor operations.
//!
//! The command system allows executing multiple operations in a single simulation
//! step, which is significantly more efficient than individual calls.
//!
//! # Examples
//!
//! ```no_run
//! use carla::{client::Client, rpc::Command};
//!
//! let mut client = Client::default();
//! let world = client.world();
//! let bp_lib = world.blueprint_library();
//! let vehicle_bp = bp_lib.find("vehicle.tesla.model3").unwrap();
//! let spawn_points = world.map().recommended_spawn_points();
//!
//! // Create batch of spawn commands
//! let commands = vec![
//!     Command::spawn_actor(
//!         vehicle_bp.clone(),
//!         spawn_points.get(0).unwrap().clone(),
//!         None,
//!     ),
//!     Command::spawn_actor(
//!         vehicle_bp.clone(),
//!         spawn_points.get(1).unwrap().clone(),
//!         None,
//!     ),
//!     Command::spawn_actor(
//!         vehicle_bp.clone(),
//!         spawn_points.get(2).unwrap().clone(),
//!         None,
//!     ),
//! ];
//!
//! // Execute all spawns in one batch
//! let responses = client.apply_batch_sync(commands, false);
//! for response in responses {
//!     if let Some(actor_id) = response.actor_id() {
//!         println!("Spawned actor: {}", actor_id);
//!     }
//! }
//! ```

use crate::{
    client::ActorBlueprint,
    geom::{Location, Transform, Vector3D},
    rpc::{
        ActorId, TrafficLightState, VehicleAckermannControl, VehicleControl, VehicleLightState,
        VehiclePhysicsControl, WalkerControl,
    },
};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::FfiCommandBatch;
use cxx::UniquePtr;

/// A command to be executed in a batch operation.
///
/// Commands allow efficient batch processing of multiple operations in a single
/// simulation step. Each command type corresponds to a specific actor operation.
pub enum Command {
    /// Spawn a new actor in the simulation.
    ///
    /// # Fields
    /// - `blueprint` - Actor blueprint defining type and attributes
    /// - `transform` - Initial spawn location and rotation
    /// - `parent` - Optional parent actor for attachment
    SpawnActor {
        blueprint: ActorBlueprint,
        transform: Transform,
        parent: Option<ActorId>,
    },

    /// Remove an actor from the simulation.
    DestroyActor { actor_id: ActorId },

    /// Apply vehicle control (throttle, steering, brake).
    ApplyVehicleControl {
        actor_id: ActorId,
        control: VehicleControl,
    },

    /// Apply Ackermann steering model control.
    ApplyVehicleAckermannControl {
        actor_id: ActorId,
        control: VehicleAckermannControl,
    },

    /// Apply walker (pedestrian) control.
    ApplyWalkerControl {
        actor_id: ActorId,
        control: WalkerControl,
    },

    /// Set vehicle physics parameters.
    ApplyVehiclePhysicsControl {
        actor_id: ActorId,
        physics_control: VehiclePhysicsControl,
    },

    /// Teleport actor to a new transform.
    ApplyTransform {
        actor_id: ActorId,
        transform: Transform,
    },

    /// Teleport actor to a new location (rotation unchanged).
    ApplyLocation {
        actor_id: ActorId,
        location: Location,
    },

    /// Set walker state (transform and speed).
    ApplyWalkerState {
        actor_id: ActorId,
        transform: Transform,
        speed: f32,
    },

    /// Set actor's target velocity.
    ApplyTargetVelocity {
        actor_id: ActorId,
        velocity: Vector3D,
    },

    /// Set actor's target angular velocity.
    ApplyTargetAngularVelocity {
        actor_id: ActorId,
        angular_velocity: Vector3D,
    },

    /// Apply instantaneous impulse to actor.
    ApplyImpulse {
        actor_id: ActorId,
        impulse: Vector3D,
    },

    /// Apply continuous force to actor.
    ApplyForce { actor_id: ActorId, force: Vector3D },

    /// Apply instantaneous angular impulse (rotation).
    ApplyAngularImpulse {
        actor_id: ActorId,
        impulse: Vector3D,
    },

    /// Apply continuous torque (rotational force).
    ApplyTorque { actor_id: ActorId, torque: Vector3D },

    /// Enable or disable physics simulation for actor.
    SetSimulatePhysics { actor_id: ActorId, enabled: bool },

    /// Enable or disable gravity for actor.
    SetEnableGravity { actor_id: ActorId, enabled: bool },

    /// Enable or disable autopilot for vehicle.
    SetAutopilot {
        actor_id: ActorId,
        enabled: bool,
        tm_port: u16,
    },

    /// Show or hide debug telemetry display.
    ShowDebugTelemetry { actor_id: ActorId, enabled: bool },

    /// Set vehicle light state (headlights, brake lights, etc.).
    SetVehicleLightState {
        actor_id: ActorId,
        light_state: VehicleLightState,
    },

    /// Execute a console command.
    ConsoleCommand { command: String },

    /// Set traffic light state (red, yellow, green).
    SetTrafficLightState {
        actor_id: ActorId,
        state: TrafficLightState,
    },
}

impl Command {
    /// Creates a spawn actor command.
    pub fn spawn_actor(
        blueprint: ActorBlueprint,
        transform: Transform,
        parent: Option<ActorId>,
    ) -> Self {
        Self::SpawnActor {
            blueprint,
            transform,
            parent,
        }
    }

    /// Creates a destroy actor command.
    pub fn destroy_actor(actor_id: ActorId) -> Self {
        Self::DestroyActor { actor_id }
    }

    /// Creates an apply vehicle control command.
    pub fn apply_vehicle_control(actor_id: ActorId, control: VehicleControl) -> Self {
        Self::ApplyVehicleControl { actor_id, control }
    }

    /// Creates an apply walker control command.
    pub fn apply_walker_control(actor_id: ActorId, control: WalkerControl) -> Self {
        Self::ApplyWalkerControl { actor_id, control }
    }

    /// Creates an apply transform command.
    pub fn apply_transform(actor_id: ActorId, transform: Transform) -> Self {
        Self::ApplyTransform {
            actor_id,
            transform,
        }
    }

    /// Creates a set autopilot command.
    pub fn set_autopilot(actor_id: ActorId, enabled: bool, tm_port: u16) -> Self {
        Self::SetAutopilot {
            actor_id,
            enabled,
            tm_port,
        }
    }

    /// Creates a console command.
    pub fn console_command(command: String) -> Self {
        Self::ConsoleCommand { command }
    }

    /// Adds this command to an FFI command batch.
    ///
    /// This method translates the Rust Command enum into the corresponding
    /// C++ command by calling the appropriate Add* method on FfiCommandBatch.
    pub(crate) fn add(&self, batch: &mut UniquePtr<FfiCommandBatch>) {
        use carla_sys::carla_rust::client::FfiCommandBatch as Batch;

        match self {
            Self::SpawnActor {
                blueprint,
                transform,
                parent,
            } => {
                let desc = blueprint.inner.MakeActorDescription().within_unique_ptr();
                Batch::AddSpawnActor(
                    batch.pin_mut(),
                    desc,
                    transform.as_ffi(),
                    parent.unwrap_or(0),
                );
            }
            Self::DestroyActor { actor_id } => {
                Batch::AddDestroyActor(batch.pin_mut(), *actor_id);
            }
            Self::ApplyVehicleControl { actor_id, control } => {
                Batch::AddApplyVehicleControl(batch.pin_mut(), *actor_id, control);
            }
            Self::ApplyVehicleAckermannControl { actor_id, control } => {
                Batch::AddApplyVehicleAckermannControl(batch.pin_mut(), *actor_id, control);
            }
            Self::ApplyWalkerControl { actor_id, control } => {
                Batch::AddApplyWalkerControl(batch.pin_mut(), *actor_id, control);
            }
            Self::ApplyVehiclePhysicsControl {
                actor_id,
                physics_control,
            } => {
                let ffi_physics = physics_control.to_cxx();
                Batch::AddApplyVehiclePhysicsControl(batch.pin_mut(), *actor_id, &ffi_physics);
            }
            Self::ApplyTransform {
                actor_id,
                transform,
            } => {
                Batch::AddApplyTransform(batch.pin_mut(), *actor_id, transform.as_ffi());
            }
            Self::ApplyLocation { actor_id, location } => {
                Batch::AddApplyLocation(batch.pin_mut(), *actor_id, location);
            }
            Self::ApplyWalkerState {
                actor_id,
                transform,
                speed,
            } => {
                Batch::AddApplyWalkerState(batch.pin_mut(), *actor_id, transform.as_ffi(), *speed);
            }
            Self::ApplyTargetVelocity { actor_id, velocity } => {
                Batch::AddApplyTargetVelocity(batch.pin_mut(), *actor_id, velocity);
            }
            Self::ApplyTargetAngularVelocity {
                actor_id,
                angular_velocity,
            } => {
                Batch::AddApplyTargetAngularVelocity(batch.pin_mut(), *actor_id, angular_velocity);
            }
            Self::ApplyImpulse { actor_id, impulse } => {
                Batch::AddApplyImpulse(batch.pin_mut(), *actor_id, impulse);
            }
            Self::ApplyForce { actor_id, force } => {
                Batch::AddApplyForce(batch.pin_mut(), *actor_id, force);
            }
            Self::ApplyAngularImpulse { actor_id, impulse } => {
                Batch::AddApplyAngularImpulse(batch.pin_mut(), *actor_id, impulse);
            }
            Self::ApplyTorque { actor_id, torque } => {
                Batch::AddApplyTorque(batch.pin_mut(), *actor_id, torque);
            }
            Self::SetSimulatePhysics { actor_id, enabled } => {
                Batch::AddSetSimulatePhysics(batch.pin_mut(), *actor_id, *enabled);
            }
            Self::SetEnableGravity { actor_id, enabled } => {
                Batch::AddSetEnableGravity(batch.pin_mut(), *actor_id, *enabled);
            }
            Self::SetAutopilot {
                actor_id,
                enabled,
                tm_port,
            } => {
                Batch::AddSetAutopilot(batch.pin_mut(), *actor_id, *enabled, *tm_port);
            }
            Self::ShowDebugTelemetry { actor_id, enabled } => {
                Batch::AddShowDebugTelemetry(batch.pin_mut(), *actor_id, *enabled);
            }
            Self::SetVehicleLightState {
                actor_id,
                light_state,
            } => {
                // Extract the underlying uint32_t flag value from VehicleLightState
                // VehicleLightState is actually VehicleLightState_LightState which has a .flag field
                // But autocxx wraps it, so we need to access the underlying value
                // For now, use a workaround by passing the POD type directly
                unsafe {
                    // SAFETY: VehicleLightState_LightState is a POD type containing just a flag field
                    let flag_value = std::mem::transmute_copy::<_, u32>(light_state);
                    Batch::AddSetVehicleLightState(batch.pin_mut(), *actor_id, flag_value);
                }
            }
            Self::ConsoleCommand { command } => {
                // Convert String to CxxString
                use cxx::let_cxx_string;
                let_cxx_string!(cmd = command);
                Batch::AddConsoleCommand(batch.pin_mut(), &cmd);
            }
            Self::SetTrafficLightState { actor_id, state } => {
                // TrafficLightState is an enum class backed by uint8_t
                // Extract the underlying value
                unsafe {
                    // SAFETY: TrafficLightState is an enum class with uint8_t underlying type
                    let state_value = std::mem::transmute_copy::<_, u8>(state);
                    Batch::AddSetTrafficLightState(batch.pin_mut(), *actor_id, state_value);
                }
            }
        }
    }
}
