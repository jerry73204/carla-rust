//! Client library for the CARLA simulator.
//!
//! This module provides the complete client API matching the C++ structure,
//! including actors, world management, and simulation control.

mod actor; // Enhanced actor with complete functionality and physics
mod actor_base;
mod actor_list;
mod blueprint; // Blueprint management (ActorBlueprint and BlueprintLibrary)
mod client;
mod map; // Map and waypoint functionality (merged from map_minimal)
mod sensor; // Sensor actor management (newtype wrapper around Actor)
mod traffic_sign;
mod vehicle; // Enhanced vehicle with newtype wrapper and advanced features
mod vehicle_physics; // Vehicle physics control module
mod walker; // New walker module for pedestrian functionality
mod world;

// TODO: Uncomment these modules as they are migrated to new C API
// mod ackermann_control;
// mod actor_attribute;
// mod actor_attribute_value_list;
// mod actor_blueprint;
// mod actor_builder;
// mod actor_kind;
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
// mod traffic_light_list;
// mod waypoint;
// mod waypoint_list;
// mod world_snapshot;

pub use actor::{Actor, ActorAttribute, TrafficLight, TrafficSign}; // Use enhanced actor with physics
pub use actor_base::ActorBase;
pub use actor_list::{ActorList, ActorListIterator, ActorListOwnedIterator};
pub use blueprint::{ActorBlueprint, BlueprintLibrary}; // Export from new blueprint module
pub use client::*;
pub use map::{Map, Waypoint};
pub use sensor::{
    EnhancedSensorUserData, Sensor, SensorCallback, SensorErrorHandler, SensorUserData,
}; // Export sensor actor from client module
   // Export enhanced Vehicle with newtype wrapper, control types, and physics types
pub use vehicle::{
    AckermannControllerSettings, DoorState, EnginePhysics, GearPhysicsControl, SuspensionPhysics,
    TirePhysics, TrafficLightState, Vehicle, VehicleAckermannControl, VehicleControl,
    VehicleDamage, VehicleDoor, VehicleFailureState, VehicleLightState, VehiclePhysicsControl,
    VehiclePhysicsControlAdvanced, VehicleSubsystem, VehicleTelemetryData, VehicleWheelLocation,
    WheelPhysics, WheelPhysicsControl, WheelPosition,
};
pub use walker::{
    AnimationMode, AnimationState, BoneTransform, Walker, WalkerAIController, WalkerBoneControl,
    WalkerControl, WalkerState,
};
pub use world::*;

// Re-export sensor data types from unified sensor module
// TODO: Some types temporarily disabled for Phase 5.1 - will be re-enabled in Phase 6
pub use crate::sensor::{
    // CollisionEvent, DvsEvent, DvsEventArray, - temporarily disabled
    DvsAnalysis,
    GnssData,
    ImageData,
    ImageRegionOfInterest,
    ImuData,
    // LaneInvasionEvent, ObstacleDetectionEvent, - temporarily disabled
    LidarData,
    LidarPoint,
    OpticalFlowAnalysis,
    // OpticalFlowImage, OpticalFlowPixel, - temporarily disabled
    RadarData,
    RadarDetection,
    SemanticLidarData,
    SemanticLidarPoint,
    SensorAttribute,
    SensorAttributeType,
    SensorCalibrationData,
    SensorConfiguration,
    SensorData,
    SensorDataType,
};
