//! Comprehensive sensor data types and processing utilities.
//!
//! This module provides the complete sensor system for CARLA, including:
//! - Raw sensor data types (Image, LiDAR, Radar, IMU, etc.)
//! - Advanced sensor data processing and analysis
//! - Multi-sensor fusion capabilities
//! - Integration with ROS2 sensor message types
//!
//! For sensor actor management, see `crate::client::Sensor`.

// Organize sensor data and analysis into submodules
// TODO: Enable when Phase 6 data types are ready - currently disabled due to missing dependencies
// pub mod data;
// pub mod analysis;

// Phase 6 Migration Roadmap:
// 1. Enable `pub mod data;` when C API types are available
// 2. Enable `pub mod analysis;` when advanced analysis features are needed
// 3. Migrate from `legacy_data.rs` to organized `data/` submodules:
//    - `data/image.rs` - Image sensor data types
//    - `data/lidar.rs` - LiDAR point cloud data
//    - `data/radar.rs` - Radar detection data
//    - `data/imu.rs` - IMU sensor data
//    - `data/gnss.rs` - GNSS positioning data
//    - `data/collision.rs` - Collision event data
//    - `data/lane_invasion.rs` - Lane invasion event data
//    - `data/dvs.rs` - Dynamic Vision Sensor data
//    - `data/optical_flow.rs` - Optical flow data
// 4. Add `analysis/` submodules:
//    - `analysis/image_processing.rs` - Image analysis utilities
//    - `analysis/point_cloud.rs` - LiDAR analysis utilities
//    - `analysis/fusion.rs` - Multi-sensor fusion
//    - `analysis/object_detection.rs` - Object detection pipelines

// Legacy data support (temporary - to be removed after migration)
mod legacy_data;
mod sensor_data;

// Phase 6 preparation: Create stub modules for future organization
// These will be enabled once the C API provides the necessary types

/// Sensor data type definitions (Phase 6).
/// Currently disabled - will be enabled when C API provides proper data structures.
pub mod data {
    //! Future home for organized sensor data types.
    //! This module will contain dedicated submodules for each sensor type
    //! with improved type safety and memory efficiency.
}

/// Sensor analysis utilities (Phase 6).
/// Currently disabled - will be enabled when advanced analysis features are implemented.
pub mod analysis {
    //! Future home for sensor data analysis and processing utilities.
    //! This module will provide advanced processing capabilities including
    //! multi-sensor fusion, object detection, and performance optimization.
}

// Re-export sensor data types from organized modules
// TODO: Enable when Phase 6 data types are ready
// pub use data::*;

// Re-export analysis utilities
// TODO: Enable when Phase 6 analysis is ready
// pub use analysis::*;

// Re-export current sensor data functionality
pub use sensor_data::*;

// Re-export additional types from legacy data module that aren't in the new data module
pub use legacy_data::{
    DvsAnalysis, GnssData, ImageData, ImageRegionOfInterest, ImuData, LidarData, LidarPoint,
    OpticalFlowAnalysis, RadarData, RadarDetection, SemanticLidarData, SemanticLidarPoint,
};

// TODO: Define SensorDataType when sensor type system is finalized
pub type SensorDataType = u32;

// Re-export the Sensor actor from the client module
pub use crate::client::{
    EnhancedSensorUserData, Sensor, SensorCallback, SensorErrorHandler, SensorUserData,
};

use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ffi::{c_void, CStr, CString};

/// Sensor calibration data for advanced sensor operations.
#[derive(Debug, Clone)]
pub struct SensorCalibrationData {
    /// Camera intrinsic matrix (3x3) for camera sensors.
    pub intrinsic_matrix: Option<[f32; 9]>,
    /// Camera distortion coefficients.
    pub distortion_coefficients: Option<Vec<f32>>,
    /// Field of view in degrees.
    pub fov: Option<f32>,
    /// Sensor resolution (width, height).
    pub resolution: Option<(u32, u32)>,
}

impl SensorCalibrationData {
    /// Create from C calibration data.
    /// TODO: Implement when C API provides proper calibration data structure
    pub(crate) fn from_c_data(_c_data: *mut std::ffi::c_void) -> Self {
        // TODO: Implement conversion when C API is available
        Self {
            intrinsic_matrix: None,
            distortion_coefficients: None,
            fov: None,
            resolution: None,
        }
    }
}

/// Sensor attribute key-value pair.
#[derive(Debug, Clone)]
pub struct SensorAttribute {
    /// Attribute key.
    pub key: String,
    /// Attribute value.
    pub value: String,
    /// Attribute type hint.
    pub attribute_type: SensorAttributeType,
}

/// Sensor attribute type enumeration.
#[derive(Debug, Clone, PartialEq)]
pub enum SensorAttributeType {
    /// Boolean attribute.
    Bool,
    /// Integer attribute.
    Int,
    /// Float attribute.
    Float,
    /// String attribute.
    String,
    /// RGB color attribute.
    RGBColor,
}

/// Sensor configuration for batch attribute setting.
#[derive(Debug, Clone)]
pub struct SensorConfiguration {
    /// Map of attribute key-value pairs.
    pub attributes: std::collections::HashMap<String, String>,
}

impl SensorConfiguration {
    /// Create a new sensor configuration.
    pub fn new() -> Self {
        Self {
            attributes: std::collections::HashMap::new(),
        }
    }

    /// Add an attribute to the configuration.
    pub fn attribute(mut self, key: &str, value: &str) -> Self {
        self.attributes.insert(key.to_string(), value.to_string());
        self
    }

    /// Set image size for camera sensors.
    pub fn image_size(self, width: u32, height: u32) -> Self {
        self.attribute("image_size_x", &width.to_string())
            .attribute("image_size_y", &height.to_string())
    }

    /// Set field of view for camera sensors.
    pub fn fov(self, fov_degrees: f32) -> Self {
        self.attribute("fov", &fov_degrees.to_string())
    }

    /// Set sensor tick for data collection frequency.
    pub fn sensor_tick(self, tick_seconds: f32) -> Self {
        self.attribute("sensor_tick", &tick_seconds.to_string())
    }

    /// Set range for distance-based sensors.
    pub fn range(self, range_meters: f32) -> Self {
        self.attribute("range", &range_meters.to_string())
    }
}

impl Default for SensorConfiguration {
    fn default() -> Self {
        Self::new()
    }
}

/// Sensor configuration helper for sensor-specific settings.
#[derive(Debug, Clone)]
pub struct SensorConfigHelper {
    /// Sensor type identifier.
    pub sensor_type: String,
}

impl SensorConfigHelper {
    /// Create a new sensor configuration helper.
    pub fn new(sensor_type: String) -> Result<Self> {
        Ok(Self { sensor_type })
    }

    /// Get camera-specific configuration template.
    pub fn camera_config(&self) -> SensorConfiguration {
        match self.sensor_type.as_str() {
            t if t.contains("camera") => SensorConfiguration::new()
                .image_size(800, 600)
                .fov(90.0)
                .sensor_tick(0.1),
            _ => SensorConfiguration::new(),
        }
    }

    /// Get LiDAR-specific configuration template.
    pub fn lidar_config(&self) -> SensorConfiguration {
        match self.sensor_type.as_str() {
            t if t.contains("lidar") => SensorConfiguration::new()
                .range(100.0)
                .attribute("rotation_frequency", "10")
                .attribute("channels", "32")
                .attribute("points_per_second", "56000"),
            _ => SensorConfiguration::new(),
        }
    }

    /// Get radar-specific configuration template.
    pub fn radar_config(&self) -> SensorConfiguration {
        match self.sensor_type.as_str() {
            t if t.contains("radar") => SensorConfiguration::new()
                .range(100.0)
                .attribute("horizontal_fov", "30")
                .attribute("vertical_fov", "30")
                .attribute("points_per_second", "1500"),
            _ => SensorConfiguration::new(),
        }
    }
}

/// Sensor performance metrics for monitoring.
#[derive(Debug, Clone, Default)]
pub struct SensorPerformanceMetrics {
    /// Frames per second.
    pub fps: f32,
    /// Average processing time in milliseconds.
    pub avg_processing_time_ms: f32,
    /// Total frames processed.
    pub total_frames: u64,
    /// Total dropped frames.
    pub dropped_frames: u64,
    /// Memory usage in bytes.
    pub memory_usage_bytes: u64,
}

// TODO: Temporarily disabled for Phase 5.1 - will be re-enabled in Phase 6
// pub mod data;
// pub mod fusion;
// TODO: Re-enable when C API types are available
// pub mod image_analysis;
// pub mod lidar_analysis;
// pub mod motion;
