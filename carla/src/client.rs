//! Client connection and world management.
//!
//! This module contains the main client interface for connecting to CARLA
//! and managing the simulation world, mirroring the `carla::client` namespace.

mod actor;
mod blueprint;
mod blueprint_library;
mod camera;
mod client;
mod lidar;
mod sensor;
mod traffic_light;
mod traffic_sign;
mod vehicle;
mod walker;
mod world;

pub use actor::{Actor, ActorSnapshot};
pub use blueprint::{ActorAttribute, ActorAttributeType, ActorBlueprint};
pub use blueprint_library::BlueprintLibrary;
pub use camera::{Camera, CameraType};
pub use client::Client;
pub use lidar::LiDAR;
pub use sensor::Sensor;
pub use traffic_light::TrafficLight;
pub use traffic_sign::TrafficSign;
pub use vehicle::Vehicle;
pub use walker::Walker;
pub use world::{WeatherParameters, World, WorldSettings, WorldSnapshot};

/// Unique identifier for actors in the simulation.
pub type ActorId = u32;
