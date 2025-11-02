//! Rust client library for CARLA autonomous driving simulator.
//!
//! This crate provides safe, idiomatic Rust bindings to the CARLA simulator,
//! allowing you to control vehicles, sensors, and the simulation environment
//! for autonomous driving research and development.

#![warn(rustdoc::broken_intra_doc_links)]
#![warn(rustdoc::private_intra_doc_links)]
#![deny(rustdoc::invalid_codeblock_attributes)]
//!
//! # Overview
//!
//! CARLA (Car Learning to Act) is an open-source simulator for autonomous driving
//! research. This crate wraps the C++ client library, providing a type-safe and
//! ergonomic Rust interface.
//!
//! **Supported CARLA Version:** 0.9.14
//!
//! # Quick Start
//!
//! ```no_run
//! use carla::{client::Client, rpc::VehicleControl};
//!
//! // Connect to the CARLA server
//! let client = Client::connect("localhost", 2000, None);
//! let mut world = client.world();
//!
//! // Get the blueprint library
//! let blueprint_library = world.blueprint_library();
//! # let vehicle_bp = blueprint_library.filter("vehicle.tesla.model3").get(0).unwrap();
//! # let spawn_points = world.map().recommended_spawn_points();
//! # let spawn_point = spawn_points.get(0).unwrap();
//! # let actor = world.spawn_actor(&vehicle_bp, &spawn_point).unwrap();
//! # let vehicle: carla::client::Vehicle = actor.try_into().unwrap();
//!
//! // Control a vehicle
//! let mut control = vehicle.control();
//! control.throttle = 0.5;
//! vehicle.apply_control(&control);
//! ```
//!
//! # Core Concepts
//!
//! ## Client and World
//!
//! The [`client::Client`] is the entry point for connecting to a CARLA server.
//! The [`client::World`] represents the simulation world and provides methods
//! for spawning actors, accessing the map, and controlling simulation settings.
//!
//! ## Actors
//!
//! Everything in the simulation is an [`client::Actor`]:
//! - **Vehicles** ([`client::Vehicle`]) - Cars, trucks, motorcycles
//! - **Walkers** - Pedestrians
//! - **Sensors** ([`client::Sensor`]) - Cameras, LiDAR, radar, etc.
//! - **Traffic Signs/Lights** ([`client::TrafficSign`], [`client::TrafficLight`])
//!
//! ## Sensors and Data
//!
//! Attach sensors to vehicles to collect data:
//! - Camera images ([`sensor::data::Image`])
//! - LiDAR point clouds ([`sensor::data::LidarMeasurement`])
//! - Collision events ([`sensor::data::CollisionEvent`])
//! - GNSS data ([`sensor::data::GnssMeasurement`])
//! - IMU data ([`sensor::data::ImuMeasurement`])
//!
//! ## Traffic Manager
//!
//! The [`traffic_manager::TrafficManager`] provides autopilot functionality
//! for creating realistic traffic scenarios with multiple vehicles.
//!
//! # Module Organization
//!
//! - [`client`] - Core client, world, and actor types
//! - [`sensor`] - Sensor types and sensor data
//! - [`traffic_manager`] - Traffic management and autopilot
//! - [`geom`] - Geometry types (vectors, transforms, locations)
//! - [`rpc`] - RPC types for configuration and control
//! - [`road`] - Road network and waypoint navigation
//! - [`prelude`] - Commonly used traits and extension methods
//!
//! # Thread Safety
//!
//! Most types in this crate implement [`Send`] and [`Sync`], allowing them to be
//! safely shared across threads. However, note that:
//!
//! - The CARLA server connection is not thread-safe by default
//! - Sensor callbacks run on separate threads
//! - Use appropriate synchronization when sharing mutable state
//!
//! See the individual type documentation for specific thread safety guarantees.
//!
//! # Performance Considerations
//!
//! - **Synchronous vs Asynchronous Mode**: CARLA can run in synchronous mode
//!   (stepped by client) or asynchronous mode (free-running). Use synchronous
//!   mode for deterministic simulations.
//!
//! - **Sensor Data**: Sensor callbacks may receive large amounts of data (e.g.,
//!   high-resolution images). Consider processing data on separate threads.
//!
//! - **Unwrap Optimizations**: This crate uses `unwrap_unchecked()` in many
//!   places where the C++ API guarantees non-null returns, eliminating runtime
//!   overhead. See `UNWRAP_REPLACEMENTS.md` for details.
//!
//! # Safety and FFI
//!
//! This crate uses FFI (Foreign Function Interface) to communicate with the
//! CARLA C++ library. The public API is safe, but internally uses `unsafe`
//! blocks that have been carefully audited. Key safety guarantees:
//!
//! - All public APIs are safe to use
//! - Null pointer checks are performed where necessary
//! - Memory is properly managed across the FFI boundary
//! - SharedPtr semantics are preserved
//!
//! See `AUDIT.md` for a comprehensive safety audit.
//!
//! # Python API Correspondence
//!
//! This crate closely follows the [CARLA Python API], making it easy to
//! translate Python examples to Rust:
//!
//! | Python | Rust |
//! |--------|------|
//! | `carla.Client()` | [`client::Client::connect()`] |
//! | `client.get_world()` | [`client::Client::world()`] |
//! | `world.spawn_actor()` | [`client::World::spawn_actor()`] |
//! | `vehicle.apply_control()` | [`client::Vehicle::apply_control()`] |
//!
//! [CARLA Python API]: https://carla.readthedocs.io/en/0.9.14/python_api/
//!
//! # Examples
//!
//! ## Spawning and Controlling a Vehicle
//!
//! ```no_run
//! use carla::client::Client;
//!
//! let client = Client::connect("localhost", 2000, None);
//! let mut world = client.world();
//! # let blueprint_library = world.blueprint_library();
//! # let vehicle_bp = blueprint_library.filter("vehicle.tesla.model3").get(0).unwrap();
//! # let spawn_points = world.map().recommended_spawn_points();
//! # let spawn_point = spawn_points.get(0).unwrap();
//! # let actor = world.spawn_actor(&vehicle_bp, &spawn_point).unwrap();
//! # let vehicle: carla::client::Vehicle = actor.try_into().unwrap();
//!
//! // Apply vehicle control
//! let mut control = vehicle.control();
//! control.throttle = 1.0;
//! control.steer = -0.5;
//! vehicle.apply_control(&control);
//! ```
//!
//! ## Attaching a Camera Sensor
//!
//! ```no_run
//! use carla::{client::Client, sensor::data::Image};
//! # use nalgebra::Isometry3;
//!
//! let client = Client::connect("localhost", 2000, None);
//! let mut world = client.world();
//! # let blueprint_library = world.blueprint_library();
//! # let camera_bp = blueprint_library.filter("sensor.camera.rgb").get(0).unwrap();
//! # let camera_transform = Isometry3::identity();
//! # let camera = world.spawn_actor(&camera_bp, &camera_transform).unwrap();
//! # let sensor: carla::client::Sensor = camera.try_into().unwrap();
//!
//! // Listen for camera images
//! sensor.listen(|data| {
//!     if let Ok(image) = Image::try_from(data) {
//!         println!("Received image: {}x{}", image.width(), image.height());
//!     }
//! });
//! ```
//!
//! ## Using the Traffic Manager
//!
//! ```no_run
//! use carla::client::Client;
//!
//! let client = Client::connect("localhost", 2000, None);
//! let mut world = client.world();
//! let mut traffic_manager = client.instance_tm(None);
//!
//! # let blueprint_library = world.blueprint_library();
//! # let vehicle_bp = blueprint_library.filter("vehicle.*").get(0).unwrap();
//! # let spawn_points = world.map().recommended_spawn_points();
//! # let spawn_point = spawn_points.get(0).unwrap();
//! # let actor = world.spawn_actor(&vehicle_bp, &spawn_point).unwrap();
//! # let vehicle: carla::client::Vehicle = actor.try_into().unwrap();
//!
//! // Enable autopilot and configure behavior
//! vehicle.set_autopilot(true);
//! traffic_manager.set_percentage_speed_difference(&vehicle, -20.0);
//! ```
//!
//! # Feature Flags
//!
//! This crate currently has no optional features. All functionality is enabled
//! by default.
//!
//! # Links
//!
//! - [CARLA Simulator](https://carla.org/)
//! - [CARLA Documentation](https://carla.readthedocs.io/)
//! - [Python API Reference](https://carla.readthedocs.io/en/0.9.14/python_api/)
//! - [GitHub Repository](https://github.com/jerry73204/carla-rust)
//!
//! # See Also
//!
//! - [`carla_sys`] - Low-level FFI bindings (re-exported by this crate)
//! - [`prelude`] - Import commonly used traits with `use carla::prelude::*`

pub mod client;
pub mod geom;
pub mod road;
pub mod rpc;
pub mod sensor;
pub mod traffic_manager;
mod utils;

/// Convenient re-exports of commonly used traits and extension methods.
///
/// This module provides a convenient way to import frequently used traits
/// without having to import them individually. Use `use carla::prelude::*;`
/// to bring all these traits into scope.
///
/// # Included Traits
///
/// - [`ActorBase`](crate::client::ActorBase) - Core actor functionality
/// - [`TimestampExt`](crate::client::TimestampExt) - Timestamp conversion utilities
/// - [`LocationExt`](crate::geom::LocationExt) - Location type conversions
/// - [`RotationExt`](crate::geom::RotationExt) - Rotation type conversions
/// - [`Vector2DExt`](crate::geom::Vector2DExt) - 2D vector conversions
/// - [`Vector3DExt`](crate::geom::Vector3DExt) - 3D vector conversions
/// - [`SensorDataBase`](crate::sensor::SensorDataBase) - Sensor data common methods
///
/// # Example
///
/// ```
/// use carla::{geom::Location, prelude::*};
/// use nalgebra::Translation3;
///
/// // LocationExt is now in scope
/// let translation = Translation3::new(1.0, 2.0, 3.0);
/// let loc = Location::from_na_translation(&translation);
/// ```
pub mod prelude {
    pub use crate::{
        client::{ActorBase as _, TimestampExt as _},
        geom::{LocationExt as _, RotationExt as _, Vector2DExt as _, Vector3DExt as _},
        sensor::SensorDataBase as _,
    };
}

pub use carla_sys;
