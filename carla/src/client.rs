//! Client connection and world management.
//!
//! This module contains the main client interface for connecting to CARLA
//! and managing the simulation world, mirroring the `carla::client` namespace.

mod blueprint;
mod blueprint_library;
mod client_impl;
mod world;

pub use blueprint::{ActorAttribute, ActorAttributeType, ActorBlueprint, AttributeValue, RGBColor};
pub use blueprint_library::BlueprintLibrary;
pub use client_impl::Client;
pub use world::{WeatherParameters, World, WorldSettings, WorldSnapshot};
