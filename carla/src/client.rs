//! Client library for the CARLA simulator.

mod actor_minimal;
mod client;
mod map_minimal;
mod vehicle_minimal;
mod world;

// TODO: Uncomment these modules as they are migrated
// mod ackermann_control;
// mod actor;
// mod actor_attribute;
// mod actor_attribute_value_list;
// mod actor_base;
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

pub use actor_minimal::{Actor, ActorBlueprint, BlueprintLibrary};
pub use client::*;
pub use map_minimal::{Map, Waypoint};
pub use vehicle_minimal::{
    AckermannControllerSettings, TrafficLightState, Vehicle, VehicleAckermannControl,
    VehicleControl, VehicleDoor, VehicleFailureState, VehicleLightState, VehicleWheelLocation,
};
pub use world::*;
