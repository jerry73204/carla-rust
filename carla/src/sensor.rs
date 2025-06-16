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
mod camera;
mod collision;
mod data;
mod dvs;
mod gnss;
mod imu;
mod lane_invasion;
mod lidar;
mod obstacle_detection;
mod optical_flow;
mod radar;
mod rss;

// Re-export sensor data and processing functionality
pub use data::*;

// Re-export sensor implementations
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
