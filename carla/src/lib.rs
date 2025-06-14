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
//! ## Quick Start
//!
//! ```rust,no_run
//! use carla::{Client, Location, Rotation, Transform};
//!
//! // Connect to CARLA server
//! let client = Client::new("localhost", 2000, None)?;
//! let world = client.get_world();
//!
//! // Spawn a vehicle
//! let blueprint_lib = world.get_blueprint_library();
//! let vehicle_bp = blueprint_lib.find("vehicle.tesla.model3")?;
//!
//! let spawn_transform = Transform::new(Location::new(0.0, 0.0, 0.5), Rotation::default());
//!
//! let actor = world.spawn_actor(&vehicle_bp, &spawn_transform, None)?;
//! if let Some(vehicle) = actor.as_vehicle() {
//!     // Control the vehicle
//!     let control = VehicleControl {
//!         throttle: 0.8,
//!         steer: 0.0,
//!         brake: 0.0,
//!         ..Default::default()
//!     };
//!     vehicle.apply_control(&control)?;
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
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

// Re-export essential types from carla-cxx for internal use
// pub(crate) use carla_cxx;

// Core modules mirroring CARLA C++ API structure
pub mod client;
pub mod geom;
pub mod road;
pub mod rpc;
pub mod sensor;
pub mod streaming;
pub mod time;
pub mod traffic_manager;

// Error handling
pub mod error;

// Utility modules
mod traits;
mod utils;

// Re-export commonly used types for convenience
pub use client::{
    Actor, ActorBlueprint, ActorId, BlueprintLibrary, Client, Sensor, TrafficLight, TrafficSign,
    Vehicle, Walker, World, WorldSnapshot,
};

pub use geom::{BoundingBox, Location, Rotation, Transform, Vector2D, Vector3D};

pub use rpc::{
    ActorCommand, VehicleControl, VehicleDoorType, VehicleLightState, VehiclePhysicsControl,
    VehicleTelemetryData, WalkerControl,
};

pub use sensor::{
    CollisionData, DepthImageData, GNSSData, IMUData, ImageData, InstanceSegmentationImageData,
    LaneInvasionData, LiDARData, RGBImageData, RadarData, SemanticSegmentationImageData,
    SensorData,
};

pub use traffic_manager::{TrafficManager, TrafficManagerConfig};

pub use road::{
    GeoLocation, Junction, Lane, LaneChange, LaneMarkingColor, LaneMarkingType, LaneType, Map,
    Waypoint,
};

pub use time::Timestamp;

pub use streaming::{SensorStream, StreamConfig};

pub use error::{
    ActorError, CarlaError, ClientError, MapError, SensorError, SpawnError, TrafficManagerError,
    WorldError,
};

// Re-export traits for common actor behaviors
pub use traits::{ActorT, SensorT, VehicleT, WalkerT};

/// Type alias for results returned by this library
pub type Result<T, E = CarlaError> = std::result::Result<T, E>;

/// Library version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// CARLA simulator version this library targets
pub const CARLA_VERSION: &str = "0.10.0";
