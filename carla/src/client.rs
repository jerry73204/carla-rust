//! Client library for the CARLA simulator.
//!
//! This module contains the core types for interacting with a CARLA server,
//! corresponding to the `carla` namespace in the Python API.
//!
//! # Core Types
//!
//! ## Connection
//! - [`Client`] - Connection to CARLA server
//!   ([Python: carla.Client](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Client))
//! - [`World`] - The simulation world
//!   ([Python: carla.World](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World))
//!
//! ## Actors
//! - [`Actor`] - Base type for all entities in the simulation
//!   ([Python: carla.Actor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor))
//! - [`Vehicle`] - Cars, trucks, motorcycles
//!   ([Python: carla.Vehicle](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Vehicle))
//! - [`Walker`] - Pedestrians
//!   ([Python: carla.Walker](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker))
//! - [`WalkerAIController`] - AI controller for pedestrians
//!   ([Python: carla.WalkerAIController](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.WalkerAIController))
//! - [`Sensor`] - Base type for all sensors
//!   ([Python: carla.Sensor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Sensor))
//! - [`TrafficLight`] - Traffic light actors
//!   ([Python: carla.TrafficLight](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight))
//! - [`TrafficSign`] - Traffic sign actors
//!   ([Python: carla.TrafficSign](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficSign))
//!
//! ## Blueprints
//! - [`ActorBlueprint`] - Template for spawning actors
//!   ([Python: carla.ActorBlueprint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.ActorBlueprint))
//! - [`BlueprintLibrary`] - Collection of available blueprints
//!   ([Python: carla.BlueprintLibrary](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.BlueprintLibrary))
//!
//! ## Navigation
//! - [`Map`] - Road network and waypoint access
//!   ([Python: carla.Map](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map))
//! - [`Waypoint`] - Point on the road for navigation
//!   ([Python: carla.Waypoint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint))
//! - [`Junction`] - Road intersection
//!   ([Python: carla.Junction](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Junction))
//! - [`Landmark`] - Road signs and traffic signals
//!   ([Python: carla.Landmark](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark))
//!
//! ## Lighting
//! - [`LightMut`] - Street light (mutable reference)
//!   ([Python: carla.Light](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light))
//! - [`LightManager`] - Manages all street lights
//!   ([Python: carla.LightManager](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager))
//!
//! ## State and Snapshots
//! - [`WorldSnapshot`] - Immutable snapshot of world state
//!   ([Python: carla.WorldSnapshot](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.WorldSnapshot))
//! - [`ActorSnapshot`] - Snapshot of a single actor's state
//!   ([Python: carla.ActorSnapshot](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.ActorSnapshot))
//! - [`Timestamp`] - Simulation time information
//!   ([Python: carla.Timestamp](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Timestamp))
//!
//! ## Utilities
//! - [`DebugHelper`] - Draw debug shapes in the world
//!   ([Python: carla.DebugHelper](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.DebugHelper))
//!
//! # Python API Reference
//!
//! See the [carla package](https://carla.readthedocs.io/en/0.9.16/python_api/#carla-package)
//! documentation for the Python equivalent types.

mod actor;
mod actor_attribute;
mod actor_attribute_value_list;
mod actor_base;
mod actor_blueprint;
mod actor_builder;
mod actor_kind;
mod actor_list;
mod actor_snapshot;
mod actor_vec;
mod blueprint_library;
mod bounding_box_list;
mod carla_client;
mod debug_helper;
mod environment_object_list;
mod junction;
mod labelled_point_list;
mod landmark;
mod landmark_list;
mod light;
mod light_list;
mod light_manager;
mod map;
mod sensor;
mod timestamp;
mod traffic_light;
mod traffic_light_list;
mod traffic_sign;
mod vehicle;
mod walker;
mod walker_ai_controller;
mod waypoint;
mod waypoint_list;
mod world;
mod world_snapshot;

pub use actor::*;
pub use actor_attribute::*;
pub use actor_attribute_value_list::*;
pub use actor_base::*;
pub use actor_blueprint::*;
pub use actor_builder::*;
pub use actor_kind::*;
pub use actor_list::*;
pub use actor_snapshot::*;
pub use actor_vec::*;
pub use blueprint_library::*;
pub use bounding_box_list::*;
pub use carla_client::*;
pub use debug_helper::*;
pub use environment_object_list::*;
pub use junction::*;
pub use labelled_point_list::*;
pub use landmark::*;
pub use landmark_list::*;
pub use light::*;
pub use light_list::*;
pub use light_manager::*;
pub use map::*;
pub use sensor::*;
pub use timestamp::*;
pub use traffic_light::*;
pub use traffic_light_list::*;
pub use traffic_sign::*;
pub use vehicle::*;
pub use walker::*;
pub use walker_ai_controller::*;
pub use waypoint::*;
pub use waypoint_list::*;
pub use world::*;
pub use world_snapshot::*;

pub use carla_sys::carla_rust::client::FfiClientLightState as LightState;
