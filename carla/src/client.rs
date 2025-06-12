//! Client library for the CARLA simulator.
//!
//! This module provides the complete client API matching the C++ structure,
//! including actors, world management, and simulation control.

mod actor_minimal;
mod client;
mod map_minimal;
mod vehicle_minimal;
mod walker; // New walker module for pedestrian functionality
mod world;

// TODO: Uncomment these modules as they are migrated to new C API
// mod ackermann_control;
mod actor; // Enhanced actor with complete functionality
           // mod actor_attribute;
           // mod actor_attribute_value_list;
mod actor_base;
// mod actor_blueprint;
// mod actor_builder;
// mod actor_kind;
// mod actor_list;
// mod actor_vec;
// mod blueprint_library;
// mod bounding_box_list;
// mod environment_object_list;
// mod junction;
// mod labelled_point_list;
// mod landmark;
// mod landmark_list;
// mod light;
// mod light_list;
// mod light_manager;
// mod map;
// mod sensor;
// mod timestamp;
// mod traffic_light;
// mod traffic_light_list;
// mod traffic_sign;
// mod vehicle;
// mod waypoint;
// mod waypoint_list;
// mod world_snapshot;

pub use actor::Actor; // Use enhanced actor instead of minimal
pub use actor_base::ActorBase;
pub use actor_minimal::{ActorBlueprint, BlueprintLibrary};
pub use client::*;
pub use map_minimal::{Map, Waypoint};
pub use vehicle_minimal::{
    AckermannControllerSettings, TrafficLightState, Vehicle, VehicleAckermannControl,
    VehicleControl, VehicleDoor, VehicleFailureState, VehicleLightState, VehicleWheelLocation,
};
pub use walker::{Walker, WalkerControl};
pub use world::*;

// Re-export sensor types from unified sensor module
pub use crate::sensor::{
    CollisionEvent, DvsAnalysis, DvsEvent, DvsEventArray, GnssData, ImageData,
    ImageRegionOfInterest, ImuData, LaneInvasionEvent, LidarData, LidarPoint,
    ObstacleDetectionEvent, OpticalFlowAnalysis, OpticalFlowImage, OpticalFlowPixel, RadarData,
    RadarDetection, SemanticLidarData, SemanticLidarPoint, Sensor, SensorData, SensorDataType,
};
