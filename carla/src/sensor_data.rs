//! Sensor data collection and processing module.
//!
//! This module provides high-level Rust APIs for all CARLA sensor types including:
//! - Camera sensors (RGB, Depth, Semantic, Instance Segmentation)
//! - LiDAR sensors (Standard and Semantic)
//! - Radar sensors
//! - IMU (Inertial Measurement Unit)
//! - GNSS (Global Navigation Satellite System)
//! - Collision detection sensors
//! - Lane invasion detection sensors
//! - DVS (Dynamic Vision Sensor)
//! - Optical Flow cameras
//! - Obstacle Detection sensors
//! - RSS (Road Safety) sensors

// Sensor modules
// TODO: Re-enable when callback FFI functions are CXX-compatible
// pub mod callback;
pub mod camera;
pub mod collision;
pub mod data;
pub mod dvs;
pub mod gnss;
pub mod imu;
pub mod lane_invasion;
pub mod lidar;
pub mod obstacle_detection;
pub mod optical_flow;
pub mod radar;
pub mod rss;

// Re-export sensor data and processing functionality
pub use data::*;

// Re-export sensor implementations
// TODO: Re-enable when callback FFI functions are CXX-compatible
// pub use callback::*;
pub use camera::*;
pub use collision::*;
pub use dvs::*;
pub use gnss::*;
pub use imu::*;
pub use lane_invasion::*;
pub use lidar::*;
pub use obstacle_detection::*;
pub use optical_flow::*;
pub use radar::*;
pub use rss::*;
