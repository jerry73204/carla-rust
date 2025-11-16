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
//! **Supported CARLA Versions:** 0.9.14, 0.9.15, 0.9.16 (default: 0.9.16)
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
//! - [`agents`] - Navigation agents for autonomous vehicle control
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

pub mod agents;

/// Client library for the CARLA simulator.
///
/// This module contains the core types for interacting with a CARLA server,
/// corresponding to the `carla` namespace in the Python API.
///
/// # Core Types
///
/// ## Connection
/// - [`Client`](client::Client) - Connection to CARLA server
///   ([Python: carla.Client](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Client))
/// - [`World`](client::World) - The simulation world
///   ([Python: carla.World](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World))
///
/// ## Actors
/// - [`Actor`](client::Actor) - Base type for all entities in the simulation
///   ([Python: carla.Actor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Actor))
/// - [`Vehicle`](client::Vehicle) - Cars, trucks, motorcycles
///   ([Python: carla.Vehicle](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Vehicle))
/// - [`Walker`](client::Walker) - Pedestrians
///   ([Python: carla.Walker](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker))
/// - [`WalkerAIController`](client::WalkerAIController) - AI controller for pedestrians
///   ([Python: carla.WalkerAIController](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.WalkerAIController))
/// - [`Sensor`](client::Sensor) - Base type for all sensors
///   ([Python: carla.Sensor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Sensor))
/// - [`TrafficLight`](client::TrafficLight) - Traffic light actors
///   ([Python: carla.TrafficLight](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficLight))
/// - [`TrafficSign`](client::TrafficSign) - Traffic sign actors
///   ([Python: carla.TrafficSign](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficSign))
///
/// ## Blueprints
/// - [`ActorBlueprint`](client::ActorBlueprint) - Template for spawning actors
///   ([Python: carla.ActorBlueprint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.ActorBlueprint))
/// - [`BlueprintLibrary`](client::BlueprintLibrary) - Collection of available blueprints
///   ([Python: carla.BlueprintLibrary](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.BlueprintLibrary))
///
/// ## Navigation
/// - [`Map`](client::Map) - Road network and waypoint access
///   ([Python: carla.Map](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map))
/// - [`Waypoint`](client::Waypoint) - Point on the road for navigation
///   ([Python: carla.Waypoint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint))
/// - [`Junction`](client::Junction) - Road intersection
///   ([Python: carla.Junction](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Junction))
/// - [`Landmark`](client::Landmark) - Road signs and traffic signals
///   ([Python: carla.Landmark](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark))
///
/// ## Lighting
/// - [`LightMut`](client::LightMut) - Street light (mutable reference)
///   ([Python: carla.Light](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light))
/// - [`LightManager`](client::LightManager) - Manages all street lights
///   ([Python: carla.LightManager](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager))
///
/// ## State and Snapshots
/// - [`WorldSnapshot`](client::WorldSnapshot) - Immutable snapshot of world state
///   ([Python: carla.WorldSnapshot](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.WorldSnapshot))
/// - [`ActorSnapshot`](client::ActorSnapshot) - Snapshot of a single actor's state
///   ([Python: carla.ActorSnapshot](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.ActorSnapshot))
/// - [`Timestamp`](client::Timestamp) - Simulation time information
///   ([Python: carla.Timestamp](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Timestamp))
///
/// ## Utilities
/// - [`DebugHelper`](client::DebugHelper) - Draw debug shapes in the world
///   ([Python: carla.DebugHelper](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.DebugHelper))
///
/// # Python API Reference
///
/// See the [carla package](https://carla.readthedocs.io/en/0.9.16/python_api/#carla-package)
/// documentation for the Python equivalent types.
pub mod client;
pub mod error;

/// Geometry types and utilities.
///
/// This module provides geometry primitives used throughout CARLA:
/// - [`Location`](geom::Location) - 3D position (x, y, z)
/// - [`Rotation`](geom::Rotation) - 3D rotation (pitch, yaw, roll in degrees)
/// - [`Transform`](geom::Transform) - Combined position and rotation
/// - [`Vector2D`](geom::Vector2D) - 2D vector
/// - [`Vector3D`](geom::Vector3D) - 3D vector
/// - [`BoundingBox`](geom::BoundingBox) - Axis-aligned or oriented bounding boxes
/// - [`GeoLocation`](geom::GeoLocation) - GPS coordinates (latitude, longitude, altitude)
///
/// # Coordinate System
///
/// CARLA uses Unreal Engine's **left-handed Z-up** coordinate system:
/// - **X-axis**: Forward (+X points forward)
/// - **Y-axis**: Right (+Y points right)
/// - **Z-axis**: Up (+Z points up)
///
/// ## Left-Handed vs Right-Handed
///
/// In a left-handed coordinate system, if you point your left thumb along +X (forward)
/// and your index finger along +Y (right), your middle finger points along +Z (up).
///
/// This differs from right-handed systems (common in mathematics and some graphics APIs)
/// where +Y often points left instead of right.
///
/// ## Rotation Convention
///
/// CARLA uses intrinsic Euler angles in degrees:
/// - **Roll**: Rotation around X-axis (forward) - positive rolls right wing down
/// - **Pitch**: Rotation around Y-axis (right) - positive pitches nose up
/// - **Yaw**: Rotation around Z-axis (up) - positive yaws counter-clockwise when looking down
///
/// # nalgebra Integration
///
/// Geometry types provide conversions to/from [nalgebra](https://nalgebra.org) types:
/// - [`Location`](geom::Location) - Provides `from_na_*()` and `to_na_*()` methods for `Translation3` / `Point3`
/// - [`Rotation`](geom::Rotation) - Provides `from_na()` and `to_na()` methods for `UnitQuaternion` conversion
/// - [`Transform`](geom::Transform) - Provides `from_na()` and `to_na()` methods for `Isometry3` conversion
/// - [`Vector2D`](geom::Vector2D) - Provides `from_na()` and `to_na()` methods for `Vector2` conversion
/// - [`Vector3D`](geom::Vector3D) - Provides `from_na()` and `to_na()` methods for `Vector3` conversion
///
/// **IMPORTANT**: Converted types (Isometry3, Translation3, UnitQuaternion) still represent
/// CARLA's left-handed coordinate system. nalgebra is used as a math library;
/// coordinate semantics are preserved.
///
/// ## Transform Composition
///
/// For composing transforms (e.g., sensor mounting), use the `*` operator:
///
/// ```ignore
/// use carla::geom::{Location, Rotation, Transform};
///
/// // Vehicle transform
/// let vehicle = Transform {
///     location: Location { x: 10.0, y: 0.0, z: 0.0 },
///     rotation: Rotation { pitch: 0.0, yaw: 0.0, roll: 0.0 }
/// };
///
/// // Sensor offset (2m forward, 1m right)
/// let sensor_offset = Transform {
///     location: Location { x: 2.0, y: 1.0, z: 0.0 },
///     rotation: Rotation { pitch: 0.0, yaw: 0.0, roll: 0.0 }
/// };
///
/// // Compose to get sensor world position
/// let sensor_world = vehicle * sensor_offset;
/// ```
///
/// You can also compose transforms via nalgebra (requires understanding coordinate systems):
///
/// ```ignore
/// use carla::geom::Transform;
/// let result = Transform::from_na(&(t1.to_na() * t2.to_na()));  // Advanced
/// ```
///
/// ## Handedness-Dependent Operations
///
/// Be cautious with operations that depend on coordinate system handedness:
///
/// ### Cross Products
/// Vector cross products depend on handedness:
/// - Left-handed: `forward × right = up`
/// - Right-handed: `forward × right = -up`
///
/// nalgebra's `cross()` method assumes right-handed convention by default.
/// Use CARLA types for geometric operations to ensure correctness.
///
/// ### Rotation Directions
/// Positive rotations differ between systems:
/// - In CARLA (left-handed): Positive yaw rotates counter-clockwise looking down
/// - In right-handed: Positive yaw rotates counter-clockwise looking up
///
/// ### Best Practices
///
/// - **Prefer CARLA types** for geometry operations: [`Transform`](geom::Transform), [`Location`](geom::Location), [`Rotation`](geom::Rotation)
/// - **Use nalgebra** for linear algebra: matrix ops, linear solvers, decompositions
/// - **Convert at boundaries**: Get data as nalgebra, compute, convert back to CARLA
/// - **Test with simulator**: Verify spatial relationships match expectations
///
/// # Examples
///
/// ```
/// use carla::geom::{Location, Transform};
/// use nalgebra::{Isometry3, Translation3, UnitQuaternion};
///
/// // Create a location from nalgebra
/// let translation = Translation3::new(1.0, 2.0, 3.0);
/// let location = Location::from_na_translation(&translation);
///
/// // Create a transform
/// let transform = Isometry3::from_parts(translation, UnitQuaternion::identity());
/// let carla_transform = Transform::from_na(&transform);
/// ```
pub mod geom;

/// Road network and navigation types based on OpenDRIVE.
///
/// This module provides types for working with CARLA's road network topology,
/// which follows the [OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
/// standard. It corresponds to the `carla.libcarla` road-related classes in the
/// Python API.
///
/// # Key Types
///
/// - [`Waypoint`](client::Waypoint) - Points on the road for navigation
/// - [`Landmark`](client::Landmark) - Road signs and traffic signals
/// - Road identifiers: [`RoadId`](road::RoadId), [`LaneId`](road::LaneId), [`JuncId`](road::JuncId), [`SectionId`](road::SectionId)
/// - [`LaneType`](road::LaneType) - Lane classification (driving, parking, sidewalk, shoulder, etc.)
/// - [`SignalOrientation`](road::SignalOrientation) - Traffic signal orientation
/// - [`element::LaneMarking`](road::element::LaneMarking) - Lane marking properties
///
/// # Navigation
///
/// Waypoints are the primary tool for road-following navigation. Use
/// [`Map`](client::Map) methods to query and navigate the road network:
/// - [`Map::generate_waypoints()`](client::Map::generate_waypoints) - Get all waypoints
/// - [`Map::waypoint_at()`](client::Map::waypoint_at) - Get waypoint at location
/// - [`Waypoint::next()`](client::Waypoint::next) - Get next waypoints
///
/// # OpenDRIVE Concepts
///
/// The road network is organized hierarchically:
/// - **Road** ([`RoadId`](road::RoadId)) - A stretch of road with multiple lanes
/// - **Section** ([`SectionId`](road::SectionId)) - A segment of a road with consistent lane configuration
/// - **Lane** ([`LaneId`](road::LaneId)) - Individual lanes within a section
/// - **Junction** ([`JuncId`](road::JuncId)) - Intersection connecting multiple roads
///
/// # Python API Reference
///
/// See the [Python API documentation](https://carla.readthedocs.io/en/0.9.16/python_api/#carla-waypoint)
/// for the Python equivalent types, particularly:
/// - [carla.Waypoint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint)
/// - [carla.Landmark](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark)
/// - [carla.LaneMarking](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LaneMarking)
pub mod road;

/// Remote Procedure Call (RPC) types for client-server communication.
///
/// This module contains configuration structures, control commands, and state
/// information exchanged between the client and CARLA server. These types
/// correspond to the `carla.rpc` namespace in the Python API.
///
/// # Vehicle Control
/// - [`VehicleControl`](rpc::VehicleControl) - Basic vehicle control (throttle, brake, steer)
/// - [`VehicleAckermannControl`](rpc::VehicleAckermannControl) - Ackermann steering control
/// - [`VehiclePhysicsControl`](rpc::VehiclePhysicsControl) - Physics parameters (mass, drag, wheels)
/// - [`AckermannControllerSettings`](rpc::AckermannControllerSettings) - Ackermann controller configuration
///
/// # Walker Control
/// - [`WalkerControl`](rpc::WalkerControl) - Walker/pedestrian movement control (direction, speed, jump)
/// - [`WalkerBoneControlIn`](rpc::WalkerBoneControlIn) / [`WalkerBoneControlOut`](rpc::WalkerBoneControlOut) - Skeletal animation control
///
/// # Simulation Settings
/// - [`EpisodeSettings`](rpc::EpisodeSettings) - Simulation settings (synchronous mode, time step, etc.)
/// - [`WeatherParameters`](rpc::WeatherParameters) - Weather conditions (clouds, rain, fog, sun)
///
/// # Lights and Appearance
/// - [`VehicleLightState`](rpc::VehicleLightState) - Vehicle light states (headlights, brake lights, etc.)
/// - [`LightState`](rpc::LightState) / [`LightGroup`](rpc::LightGroup) - Street light control
/// - [`Color`](rpc::Color) - RGBA color representation
///
/// # Environment
/// - [`EnvironmentObjectRef`](rpc::EnvironmentObjectRef) - Static world objects
/// - [`MapLayer`](rpc::MapLayer) - Map layer flags (buildings, props, etc.)
#[cfg_attr(carla_0916, doc = "")]
#[cfg_attr(carla_0916, doc = "# Telemetry (CARLA 0.9.16+)")]
#[cfg_attr(
    carla_0916,
    doc = "- [`VehicleTelemetryData`](rpc::VehicleTelemetryData) - Vehicle telemetry including wheel data"
)]
#[cfg_attr(
    carla_0916,
    doc = "- [`WheelTelemetryData`](rpc::WheelTelemetryData) - Individual wheel telemetry"
)]
#[cfg_attr(carla_0916, doc = "")]
/// # Miscellaneous
/// - [`ActorId`](rpc::ActorId) - Unique actor identifier
/// - [`AttachmentType`](rpc::AttachmentType) - How actors attach to parents
/// - [`TrafficLightState`](rpc::TrafficLightState) - Traffic light color states
/// - [`VehicleDoor`](rpc::VehicleDoor) - Vehicle door enumeration
/// - [`VehicleFailureState`](rpc::VehicleFailureState) - Vehicle failure states
///
/// # Python API Reference
///
/// See the [carla](https://carla.readthedocs.io/en/0.9.16/python_api/#carla-package)
/// documentation for the Python equivalent types.
///
/// # Examples
///
/// ```no_run
/// use carla::{client::Client, rpc::VehicleControl};
///
/// let client = Client::default();
/// let world = client.world();
///
/// // Spawn a vehicle
/// # let bp_lib = world.blueprint_library();
/// # let vehicle_bp = bp_lib.filter("vehicle.tesla.model3").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let vehicle = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
/// # let vehicle: carla::client::Vehicle = vehicle.try_into().unwrap();
///
/// // Create and apply vehicle control
/// let mut control = VehicleControl::default();
/// control.throttle = 0.5;
/// control.steer = -0.2;
/// vehicle.apply_control(&control);
/// ```
///
/// Most types are re-exported from the C++ library via FFI.
pub mod rpc;

/// Sensor data types for processing sensor measurements.
///
/// This module provides types for handling data from CARLA sensors. It corresponds
/// to the `carla.sensor` namespace in the Python API.
///
/// # Key Types
/// - [`SensorData`](sensor::SensorData) - Base type for all sensor data
/// - [`SensorDataBase`](sensor::SensorDataBase) - Trait providing common sensor data methods
/// - [`data`](sensor::data) - Specific sensor data types (images, LiDAR, collision, etc.)
/// - [`camera`](sensor::camera) - Camera projection and coordinate transformation utilities
///
/// # Sensor Data Types
///
/// The [`data`](sensor::data) submodule contains specialized types for each sensor:
///
/// ## Vision Sensors
/// - **RGB Camera**: [`data::Image`](sensor::data::Image) - Color images
/// - **Depth Camera**: [`data::Image`](sensor::data::Image) - Depth maps
/// - **Semantic Segmentation**: [`data::Image`](sensor::data::Image) - Semantic class labels per pixel
///
/// ## 3D Sensors
/// - **LiDAR**: [`data::LidarMeasurement`](sensor::data::LidarMeasurement) - 3D point cloud data
/// - **Semantic LiDAR**: [`data::SemanticLidarMeasurement`](sensor::data::SemanticLidarMeasurement) - Point cloud with semantic labels
/// - **Radar**: [`data::RadarMeasurement`](sensor::data::RadarMeasurement) - Radar detection points
///
/// ## Physics Sensors
/// - **Collision Detector**: [`data::CollisionEvent`](sensor::data::CollisionEvent) - Collision detection and impulse
/// - **Lane Invasion**: [`data::LaneInvasionEvent`](sensor::data::LaneInvasionEvent) - Lane crossing detection
/// - **Obstacle Detector**: [`data::ObstacleDetectionEvent`](sensor::data::ObstacleDetectionEvent) - Proximity detection
///
/// ## Navigation Sensors
/// - **GNSS**: [`data::GnssMeasurement`](sensor::data::GnssMeasurement) - GPS coordinates (latitude, longitude, altitude)
/// - **IMU**: [`data::ImuMeasurement`](sensor::data::ImuMeasurement) - Accelerometer, gyroscope, compass
///
/// ## Advanced Sensors
/// - **DVS Camera**: [`data::DVSEventArray`](sensor::data::DVSEventArray) - Event-based vision sensor
/// - **Optical Flow**: [`data::OpticalFlowImage`](sensor::data::OpticalFlowImage) - Motion vectors
///
/// # Camera Utilities
///
/// The [`camera`](sensor::camera) module provides utilities for camera projection and sensor fusion:
/// - [`camera::build_projection_matrix`](sensor::camera::build_projection_matrix) - Build camera intrinsic matrix
/// - [`camera::world_to_camera`](sensor::camera::world_to_camera) - Transform world coordinates to camera space
/// - [`camera::project_to_2d`](sensor::camera::project_to_2d) - Project 3D points to 2D image coordinates
///
/// # Python API Reference
///
/// See the [carla.sensor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla-sensor)
/// documentation for the Python equivalent types.
///
/// # Examples
///
/// ## RGB Camera
///
/// ```no_run
/// use carla::{
///     client::Client,
///     sensor::{data::Image, SensorDataBase},
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// // Spawn a camera
/// let bp_lib = world.blueprint_library();
/// let camera_bp = bp_lib.filter("sensor.camera.rgb").get(0).unwrap();
/// let spawn_points = world.map().recommended_spawn_points();
/// let camera = world
///     .spawn_actor(&camera_bp, &spawn_points.get(0).unwrap())
///     .unwrap();
/// let sensor: carla::client::Sensor = camera.try_into().unwrap();
///
/// // Listen for images
/// sensor.listen(|data| {
///     if let Ok(image) = Image::try_from(data) {
///         println!("Received {}x{} image", image.width(), image.height());
///     }
/// });
/// ```
pub mod sensor;

/// Traffic manager for controlling autopilot vehicles with realistic behavior.
///
/// The [`TrafficManager`](traffic_manager::TrafficManager) provides fine-grained control over groups of vehicles
/// in autopilot mode, enabling realistic urban traffic simulation with customizable
/// behaviors. This module corresponds to the
/// [carla.TrafficManager](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager)
/// class in the Python API.
///
/// # Key Features
///
/// - **Speed control**: Set global or per-vehicle speed limits
/// - **Lane behavior**: Configure lane changes, lane offset, keep-right rules
/// - **Safety**: Collision detection, safe distances
/// - **Traffic rules**: Ignore pedestrians/vehicles, run red lights (probabilistic)
/// - **Routing**: Custom paths and imported routes
/// - **Performance**: Hybrid physics mode for large-scale simulations
///
/// # Python API Reference
///
/// See the [carla.TrafficManager](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.TrafficManager)
/// documentation for the Python equivalent.
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let mut world = client.world();
/// let mut tm = client.instance_tm(8000);
///
/// // Spawn some vehicles
/// let bp_lib = world.blueprint_library();
/// let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// let spawn_points = world.map().recommended_spawn_points();
/// let vehicle1 = world
///     .spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap())
///     .unwrap();
/// let vehicle2 = world
///     .spawn_actor(&vehicle_bp, &spawn_points.get(1).unwrap())
///     .unwrap();
///
/// // Register vehicles with traffic manager
/// let vehicles = [vehicle1.clone(), vehicle2.clone()];
/// tm.register_vehicles(&vehicles);
///
/// // Configure behavior
/// tm.set_global_percentage_speed_difference(-20.0); // 20% faster
/// tm.set_auto_lane_change(&vehicle1, true);
/// tm.set_distance_to_leading_vehicle(&vehicle1, 5.0);
///
/// // Enable autopilot (done on vehicle, not TM)
/// let vehicle1_typed: carla::client::Vehicle = vehicle1.try_into().unwrap();
/// vehicle1_typed.set_autopilot(true);
/// ```
pub mod traffic_manager;
mod utils;

// Re-export error types at crate root for convenience
pub use error::{
    CarlaError, ConnectionError, InternalError, MapError, OperationError, ResourceError,
    ResourceType, Result, SensorError, ValidationError,
};

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
/// - [`SensorDataBase`](crate::sensor::SensorDataBase) - Sensor data common methods
///
/// # Example
///
/// ```
/// use carla::geom::Location;
/// use nalgebra::Translation3;
///
/// // Location methods are now directly on the type
/// let translation = Translation3::new(1.0, 2.0, 3.0);
/// let loc = Location::from_na_translation(&translation);
/// ```
pub mod prelude {
    pub use crate::{
        client::{ActorBase as _, TimestampExt as _},
        sensor::SensorDataBase as _,
    };
}

pub use carla_sys;
