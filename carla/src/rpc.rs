//! Remote Procedure Call (RPC) types for client-server communication.
//!
//! This module contains configuration structures, control commands, and state
//! information exchanged between the client and CARLA server:
//!
//! # Vehicle Control
//! - [`VehicleControl`] - Basic vehicle control (throttle, brake, steer)
//! - [`VehicleAckermannControl`] - Ackermann steering control
//! - [`VehiclePhysicsControl`] - Physics parameters (mass, drag, wheels)
//! - [`AckermannControllerSettings`] - Ackermann controller configuration
//!
//! # Walker Control
//! - [`WalkerControl`] - Walker/pedestrian movement control (direction, speed, jump)
//!
//! # Simulation Settings
//! - [`EpisodeSettings`] - Simulation settings (synchronous mode, time step, etc.)
//! - [`WeatherParameters`] - Weather conditions (clouds, rain, fog, sun)
//!
//! # Lights and Appearance
//! - [`VehicleLightState`] - Vehicle light states (headlights, brake lights, etc.)
//! - [`LightState`] / [`LightGroup`] - Street light control
//!
//! # Environment
//! - `EnvironmentObject` - Static world objects
//! - [`MapLayer`] - Map layer flags (buildings, props, etc.)
//!
//! # Miscellaneous
//! - [`ActorId`] - Unique actor identifier
//! - [`AttachmentType`] - How actors attach to parents
//! - [`TrafficLightState`] - Traffic light color states
//!
//! Most types are re-exported from the C++ library via FFI.

mod environment_object;
mod episode_settings;
mod light_id;
mod map_layer;
mod vehicle_failure_state;
mod vehicle_light_state_list;
mod vehicle_physics_control;
#[cfg(carla_0916)]
mod vehicle_telemetry_data;
mod walker_bone_control;

pub use carla_sys::{
    carla::rpc::{
        AckermannControllerSettings, AttachmentType, GearPhysicsControl,
        OpendriveGenerationParameters, TrafficLightState, VehicleAckermannControl, VehicleControl,
        VehicleDoor, VehicleLightState_LightState as VehicleLightState, VehicleWheelLocation,
        WalkerControl, WeatherParameters, WheelPhysicsControl,
    },
    carla_rust::rpc::{
        FfiActorId as ActorId, FfiLabelledPoint as LabelledPoint, FfiRpcLightGroup as LightGroup,
        FfiRpcLightState as LightState,
    },
};
pub use environment_object::*;
pub use episode_settings::*;
pub use light_id::*;
pub use map_layer::*;
pub use vehicle_failure_state::*;
pub use vehicle_light_state_list::*;
pub use vehicle_physics_control::*;
#[cfg(carla_0916)]
pub use vehicle_telemetry_data::*;
pub use walker_bone_control::*;
