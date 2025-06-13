//! Comprehensive sensor data types and processing utilities.
//!
//! This module provides the complete sensor system for CARLA, including:
//! - Raw sensor data types (Image, LiDAR, Radar, IMU, etc.)
//! - Advanced sensor data processing and analysis
//! - Multi-sensor fusion capabilities
//! - Integration with ROS2 sensor message types
//!
//! For sensor actor management, see `crate::client::Sensor`.

mod sensor_data;
pub use sensor_data::*;

// Merge functionality from sensor_legacy
mod legacy_data;

// TODO: Temporarily disabled for Phase 5.1 - will be re-enabled in Phase 6
// Re-export all sensor data types (prefer new data types, supplement with legacy)
// pub use data::{
//     CollisionEvent, DvsEvent, DvsEventArray, GnssMeasurement as GnssData, Image,
//     ImuMeasurement as ImuData, LaneInvasionEvent, LidarMeasurement as LidarData,
//     ObstacleDetectionEvent, OpticalFlow as OpticalFlowImage, OpticalFlowPixel,
//     RadarMeasurement as RadarData,
// };

// Re-export additional types from legacy data module that aren't in the new data module
pub use legacy_data::{
    DvsAnalysis, GnssData, ImageData, ImageRegionOfInterest, ImuData, LidarData, LidarPoint,
    OpticalFlowAnalysis, RadarData, RadarDetection, SemanticLidarData, SemanticLidarPoint,
};

// TODO: Define SensorDataType when sensor type system is finalized
pub type SensorDataType = u32;

// Re-export the Sensor actor from the client module
pub use crate::client::{Sensor, SensorCallback, SensorUserData};

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

// TODO: Temporarily disabled for Phase 5.1 - will be re-enabled in Phase 6
// pub mod data;
// pub mod fusion;
// TODO: Re-enable when C API types are available
// pub mod image_analysis;
// pub mod lidar_analysis;
// pub mod motion;
