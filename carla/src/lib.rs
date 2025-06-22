//! # CARLA Rust Client Library
//!
//! High-level Rust bindings for CARLA simulator that mirror the C++ API structure.
//! This crate provides a safe, idiomatic Rust interface to CARLA's autonomous driving
//! simulation platform.
//!
//! ## Architecture
//!
//! This library follows CARLA's C++ API structure exactly:
//! - [`client`] - Client connection, world management, and actor control
//! - [`sensor`] - Sensor data collection and processing  
//! - [`traffic_manager`] - Traffic simulation and behavior control
//! - [`road`] - Road network navigation and waypoint management
//! - [`geom`] - Geometry types for transforms, locations, and vectors
//! - [`rpc`] - Remote procedure call data structures
//!
//! ## Module Organization
//!
//! All submodules are publicly accessible, allowing direct imports like:
//! - `use carla::client::blueprint::ActorBlueprint;`
//! - `use carla::actor::vehicle::Vehicle;`
//! - `use carla::road::waypoint::Waypoint;`
//!
//! Common types are also re-exported at the crate root for convenience.
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use carla::{
//!     actor::vehicle::VehicleControl,
//!     client::Client,
//!     geom::{Location, Rotation, Transform},
//!     traits::VehicleT,
//! };
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! // Connect to CARLA server
//! let client = Client::new("localhost", 2000, None)?;
//! let world = client.world()?;
//!
//! // Spawn a vehicle
//! let blueprint_lib = world.blueprint_library()?;
//! let vehicle_bp = blueprint_lib
//!     .find("vehicle.tesla.model3")?
//!     .ok_or("Vehicle blueprint not found")?;
//!
//! let spawn_transform = Transform::new(Location::new(0.0, 0.0, 0.5), Rotation::default());
//!
//! let actor = world.spawn_actor(&vehicle_bp, &spawn_transform, None)?;
//! match actor.to_vehicle() {
//!     Ok(vehicle) => {
//!         // Control the vehicle
//!         let control = VehicleControl {
//!             throttle: 0.8,
//!             steer: 0.0,
//!             brake: 0.0,
//!             ..Default::default()
//!         };
//!         vehicle.apply_control(&control)?;
//!     }
//!     Err(_actor) => {
//!         // Not a vehicle
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ## Features
//!
//! - **Type Safety**: Rust's type system prevents common C++ API misuse
//! - **Memory Safety**: Automatic resource management through RAII
//! - **Error Handling**: Comprehensive `Result` types with detailed error information
//! - **Performance**: Zero-cost abstractions over the CXX FFI layer
//! - **API Compatibility**: 1:1 mapping with CARLA's C++ client library

#![warn(missing_docs)]
#![warn(clippy::all)]
#![allow(dead_code)] // During development

// Re-export essential types from carla-sys for internal use
// pub(crate) use carla_sys;

// Core modules mirroring CARLA C++ API structure
pub mod actor;
pub mod batch;
pub mod client;
pub mod geom;
pub mod road;
pub mod sensor_data;
pub mod streaming;
pub mod time;
pub mod traffic_manager;

// Error handling
pub mod error;

// Utility modules
pub mod traits;
pub mod utils;

// Re-export commonly used types for convenience
pub use actor::{ActorExt, SensorExt};
pub use client::Client;
pub use error::CarlaError;

/// Type alias for results returned by this library
pub type Result<T, E = CarlaError> = std::result::Result<T, E>;

/// Library version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// CARLA simulator version this library targets
pub const CARLA_VERSION: &str = "0.10.0";
