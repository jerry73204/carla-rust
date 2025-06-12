mod collision_event;
pub use collision_event::*;

mod obstacle_detection_event;
pub use obstacle_detection_event::*;

mod lane_invasion_event;
pub use lane_invasion_event::*;

mod image;
pub use image::*;

mod gnss_measurement;
pub use gnss_measurement::*;

mod imu_measurement;
pub use imu_measurement::*;

mod radar_measurement;
pub use radar_measurement::*;

mod lidar_measurement;
pub use lidar_measurement::*;

mod semantic_lidar_measurement;
pub use semantic_lidar_measurement::*;

mod dvs_event;
pub use dvs_event::*;

mod optical_flow;
pub use optical_flow::*;

// TODO: Update sensor data imports to use new C API types
// pub use carla_sys::{
//     carla::sensor::data::RadarDetection,
//     carla_rust::sensor::data::{
//         FfiColor as Color, FfiLidarDetection as LidarDetection,
//         FfiSemanticLidarDetection as SemanticLidarDetection,
//     },
// };

// Temporary sensor data types - will be updated when C API is integrated
use nalgebra::{Point3, Vector3};

#[derive(Debug, Clone)]
pub struct RadarDetection {
    pub velocity: f32,
    pub azimuth: f32,
    pub altitude: f32,
    pub depth: f32,
}

#[derive(Debug, Clone)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

#[derive(Debug, Clone)]
pub struct LidarDetection {
    pub point: Point3<f32>,
    pub intensity: f32,
}

#[derive(Debug, Clone)]
pub struct SemanticLidarDetection {
    pub point: Point3<f32>,
    pub cos_inc_angle: f32,
    pub object_idx: u32,
    pub object_tag: u32,
}
