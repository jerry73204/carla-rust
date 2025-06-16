//! Client connection and world management.
//!
//! This module contains the main client interface for connecting to CARLA
//! and managing the simulation world, mirroring the `carla::client` namespace.

mod blueprint;
mod blueprint_library;
mod client;
mod world;

pub use blueprint::{ActorAttribute, ActorAttributeType, ActorBlueprint};
pub use blueprint_library::BlueprintLibrary;
pub use client::Client;
pub use world::{WeatherParameters, World, WorldSettings, WorldSnapshot};
