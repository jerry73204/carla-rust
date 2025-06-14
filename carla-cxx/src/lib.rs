//! High-level Rust bindings for CARLA simulator using direct CXX FFI.
//!
//! This crate provides safe, idiomatic Rust bindings to CARLA's C++ client library
//! using the CXX bridge for direct C++/Rust interop without a C wrapper layer.
//!
//! # Architecture
//!
//! - **ffi** - Raw FFI bridge definitions using CXX
//! - **geometry** - Mathematical operations for CARLA geometry types
//! - **client** - High-level wrappers for Client, World, Actor management
//! - **blueprint** - Extension traits for ActorBlueprint functionality
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use carla_cxx::{ClientWrapper, SimpleLocation, SimpleRotation, SimpleTransform};
//!
//! // Connect to CARLA server
//! let mut client = ClientWrapper::new("localhost", 2000)?;
//! let world = client.get_world();
//!
//! // Get blueprint library and spawn a vehicle
//! let blueprint_library = world.get_blueprint_library();
//! if let Some(vehicle_bp) = blueprint_library.find("vehicle.tesla.model3") {
//!     let spawn_point =
//!         SimpleTransform::new(SimpleLocation::new(0.0, 0.0, 0.5), SimpleRotation::ZERO);
//!
//!     if let Ok(vehicle) = world.spawn_actor(&vehicle_bp, &spawn_point, None) {
//!         println!("Spawned vehicle with ID: {}", vehicle.get_id());
//!     }
//! }
//! # Ok::<(), anyhow::Error>(())
//! ```

#![allow(dead_code)]

// Module declarations
pub mod advanced_world;
pub mod batch_operations;
pub mod blueprint;
pub mod client;
pub mod debug_helper;
pub mod ffi;
pub mod geometry;
pub mod landmark;
pub mod light_manager;
pub mod map;
pub mod recording;
pub mod sensor;
pub mod time;
pub mod traffic_light;
pub mod traffic_manager;
pub mod traffic_sign;
pub mod vehicle;
pub mod walker;
pub mod walker_ai;
pub mod weather;
pub mod world_interaction;

#[cfg(test)]
mod tests;

// Re-export the most commonly used types for convenience
pub use ffi::{
    Actor, ActorBlueprint, BlueprintLibrary, Client, Junction, Map, Sensor, SimpleBoundingBox,
    SimpleColor, SimpleDebugArrow, SimpleDebugBox, SimpleDebugLine, SimpleDebugPoint,
    SimpleDebugString, SimpleGeoLocation, SimpleLaneMarking, SimpleLocation, SimpleRotation,
    SimpleTransform, SimpleVector2D, SimpleVector3D, SimpleVehicleControl, SimpleWalkerControl,
    SimpleWaypointInfo, SimpleWeatherParameters, TrafficLight, TrafficManager, TrafficSign,
    Vehicle, Walker, Waypoint, World,
};

pub use batch_operations::{
    BatchCommandBuilder, BatchCommandType, BatchResponseExt,
    TrafficLightState as BatchTrafficLightState, VehicleLightState as BatchVehicleLightState,
};
pub use blueprint::ActorBlueprintExt;
pub use client::{ActorWrapper, BlueprintLibraryWrapper, ClientWrapper, WorldWrapper};
pub use landmark::{landmark_utils, LandmarkInfo, LandmarkType};
pub use light_manager::{LightGroup, LightInfo, LightManagerWrapper, LightState};
pub use map::{
    JunctionWrapper, LaneChange, LaneMarking, LaneMarkingColor, LaneMarkingType, LaneType,
    MapWrapper, SignalOrientation, WaypointWrapper,
};
pub use sensor::{
    CollisionData,
    CrossedLaneMarking,
    // Advanced sensor types
    DVSEvent,
    DVSEventArray,
    GNSSData,
    IMUData,
    ImageData,
    LaneInvasionData,
    LiDARData,
    LiDARPoint,
    ObstacleDetectionEvent,
    RadarData,
    RadarDetection,
    RssResponse,
    SemanticLidarData,
    SemanticLidarDetection,
    SensorData,
    SensorWrapper,
};
pub use time::Timestamp;
pub use traffic_light::{TrafficLightState, TrafficLightWrapper};
pub use traffic_manager::{
    RoadOption, TrafficManagerAction, TrafficManagerConfig, TrafficManagerStats,
    TrafficManagerVehicleConfig, TrafficManagerWrapper,
};
pub use traffic_sign::{TrafficSignType, TrafficSignWrapper};
pub use vehicle::{
    AckermannControl, EnginePhysics, GearPhysicsControl, SteeringPhysics, TransmissionPhysics,
    VehicleControl, VehicleDoorState, VehicleDoorType, VehicleLightState, VehiclePhysicsControl,
    VehicleWrapper, WheelPhysicsControl,
};
pub use walker::{Vector3D, WalkerControl, WalkerWrapper};
pub use walker_ai::{Location as WalkerLocation, WalkerAIBehavior, WalkerAIControllerWrapper};
pub use world_interaction::{
    ActorListExt, CityObjectLabel, LabelledPoint, OptionalLabelledPointExt, OptionalLocationExt,
};

// Re-export world interaction utilities
pub use world_interaction::{defaults as world_defaults, ray_casting};

// Re-export batch operation utilities
pub use batch_operations::batch_utils;

// Re-export debug visualization utilities
pub use debug_helper::{colors as debug_colors, DebugDrawBuilder, DebugDrawExt};

// Re-export advanced world features
pub use advanced_world::{
    texture_utils, AdvancedWorldExt, MapLayers, MaterialParameter, TextureColorBuilder,
    TextureFloatColorBuilder,
};

// Re-export geometry implementations (they're implemented as trait impls on the ffi types)
// The geometry module contains all the impl blocks for the Simple* types from ffi
