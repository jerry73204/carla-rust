//! Remote Procedure Call (RPC) types for client-server communication.
//!
//! This module contains configuration structures, control commands, and state
//! information exchanged between the client and CARLA server. These types
//! correspond to the `carla.rpc` namespace in the Python API.
//!
//! # Vehicle Control
//! - [`VehicleControl`] - Basic vehicle control (throttle, brake, steer)
//! - [`VehicleAckermannControl`] - Ackermann steering control
//! - [`VehiclePhysicsControl`] - Physics parameters (mass, drag, wheels)
//! - [`AckermannControllerSettings`] - Ackermann controller configuration
//!
//! # Walker Control
//! - [`WalkerControl`] - Walker/pedestrian movement control (direction, speed, jump)
//! - [`WalkerBoneControlIn`] / [`WalkerBoneControlOut`] - Skeletal animation control
//!
//! # Simulation Settings
//! - [`EpisodeSettings`] - Simulation settings (synchronous mode, time step, etc.)
//! - [`WeatherParameters`] - Weather conditions (clouds, rain, fog, sun)
//!
//! # Lights and Appearance
//! - [`VehicleLightState`] - Vehicle light states (headlights, brake lights, etc.)
//! - [`LightState`] / [`LightGroup`] - Street light control
//! - [`Color`] - RGBA color representation
//!
//! # Environment
//! - [`EnvironmentObjectRef`] - Static world objects
//! - [`MapLayer`] - Map layer flags (buildings, props, etc.)
//!
//! # Telemetry (CARLA 0.9.16+)
//! - [`VehicleTelemetryData`] - Vehicle telemetry including wheel data
//! - [`WheelTelemetryData`] - Individual wheel telemetry
//!
//! # Miscellaneous
//! - [`ActorId`] - Unique actor identifier
//! - [`AttachmentType`] - How actors attach to parents
//! - [`TrafficLightState`] - Traffic light color states
//! - [`VehicleDoor`] - Vehicle door enumeration
//! - [`VehicleFailureState`] - Vehicle failure states
//!
//! # Python API Reference
//!
//! See the [carla](https://carla.readthedocs.io/en/0.9.16/python_api/#carla-package)
//! documentation for the Python equivalent types.
//!
//! # Examples
//!
//! ```no_run
//! use carla::{client::Client, rpc::VehicleControl};
//!
//! let client = Client::default();
//! let world = client.world();
//!
//! // Spawn a vehicle
//! # let bp_lib = world.blueprint_library();
//! # let vehicle_bp = bp_lib.filter("vehicle.tesla.model3").get(0).unwrap();
//! # let spawn_points = world.map().recommended_spawn_points();
//! # let vehicle = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
//! # let vehicle: carla::client::Vehicle = vehicle.try_into().unwrap();
//!
//! // Create and apply vehicle control
//! let mut control = VehicleControl::default();
//! control.throttle = 0.5;
//! control.steer = -0.2;
//! vehicle.apply_control(&control);
//! ```
//!
//! Most types are re-exported from the C++ library via FFI.

mod color;
mod command;
mod command_response;
mod environment_object;
mod episode_settings;
mod light_id;
mod map_layer;
mod vehicle_failure_state;
mod vehicle_light_state;
mod vehicle_light_state_list;
mod vehicle_physics_control;
#[cfg(carla_0916)]
mod vehicle_telemetry_data;
mod walker_bone_control;

pub use carla_sys::{
    carla::rpc::{
        AckermannControllerSettings, AttachmentType, GearPhysicsControl,
        OpendriveGenerationParameters, TrafficLightState, VehicleAckermannControl, VehicleControl,
        VehicleDoor, VehicleWheelLocation, WalkerControl, WeatherParameters, WheelPhysicsControl,
    },
    carla_rust::rpc::{
        FfiActorId as ActorId, FfiLabelledPoint as LabelledPoint, FfiRpcLightGroup as LightGroup,
        FfiRpcLightState as LightState,
    },
};
pub use color::*;
pub use command::*;
pub use command_response::*;
pub use environment_object::*;
pub use episode_settings::*;
pub use light_id::*;
pub use map_layer::*;
pub use vehicle_failure_state::*;
pub use vehicle_light_state::*;
pub use vehicle_light_state_list::*;
pub use vehicle_physics_control::*;
#[cfg(carla_0916)]
pub use vehicle_telemetry_data::*;
pub use walker_bone_control::*;
