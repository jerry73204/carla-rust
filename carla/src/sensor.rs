//! Comprehensive sensor data types and processing utilities.
//!
//! This module provides the complete sensor system for CARLA, including:
//! - Sensor actor management and callbacks
//! - Raw sensor data types (Image, LiDAR, Radar, IMU, etc.)
//! - Advanced sensor data processing and analysis
//! - Multi-sensor fusion capabilities
//! - Integration with ROS2 sensor message types

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

// Sensor management
use crate::{
    client::{Actor, ActorBase},
    geom::{Transform, TransformExt},
    stubs::{
        carla_actor_is_sensor, carla_sensor_calibration_data_t, carla_sensor_callback_t,
        carla_sensor_data_type_t_CARLA_SENSOR_DATA_OPTICAL_FLOW, carla_sensor_get_attribute,
        carla_sensor_get_calibration_data, carla_sensor_is_listening, carla_sensor_listen,
        carla_sensor_set_attribute, carla_sensor_stop, carla_sensor_t,
    },
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Isometry3;
use std::{
    ffi::{c_void, CStr, CString},
    sync::{Arc, Mutex},
};

/// A sensor actor in the CARLA simulation.
/// This is a newtype wrapper around Actor that provides sensor-specific functionality.
#[derive(Clone, Debug)]
pub struct Sensor(pub Actor);

/// Sensor callback closure type for handling incoming sensor data.
pub type SensorCallback = Box<dyn FnMut(SensorData) + Send + 'static>;

/// User data structure for sensor callbacks.
pub struct SensorUserData {
    /// Callback function for sensor data processing.
    pub callback: Option<SensorCallback>,
    /// Custom user data pointer.
    pub user_data: *mut c_void,
}

impl Sensor {
    /// Create a Sensor from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null, and must point to a sensor actor.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_sensor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null sensor pointer"));
        }

        // Verify it's actually a sensor
        if !unsafe { carla_actor_is_sensor(ptr as *const carla_actor_t) } {
            return Err(anyhow!("Actor is not a sensor"));
        }

        // Create the base Actor first
        let actor = Actor::from_raw_ptr(ptr as *mut carla_actor_t)?;
        Ok(Self(actor))
    }

    /// Convert this Sensor back into a generic Actor.
    pub fn into_actor(self) -> Actor {
        self.0
    }

    /// Get access to the underlying Actor.
    pub fn actor(&self) -> &Actor {
        &self.0
    }

    /// Get mutable access to the underlying Actor.
    pub fn actor_mut(&mut self) -> &mut Actor {
        &mut self.0
    }

    pub(crate) fn raw_sensor_ptr(&self) -> *mut carla_sensor_t {
        self.0.raw_ptr() as *mut carla_sensor_t
    }

    /// Start listening for sensor data with a callback.
    pub fn listen<F>(&self, callback: F) -> Result<()>
    where
        F: FnMut(SensorData) + Send + 'static,
    {
        let user_data = Box::into_raw(Box::new(SensorUserData {
            callback: Some(Box::new(callback)),
            user_data: std::ptr::null_mut(),
        })) as *mut c_void;

        let error = unsafe {
            carla_sensor_listen(self.raw_sensor_ptr(), sensor_callback_wrapper, user_data)
        };
        check_carla_error(error)
    }

    /// Start listening for sensor data with a callback and custom user data.
    pub fn listen_with_user_data<F>(&self, callback: F, user_data: *mut c_void) -> Result<()>
    where
        F: FnMut(SensorData) + Send + 'static,
    {
        let callback_data = Box::into_raw(Box::new(SensorUserData {
            callback: Some(Box::new(callback)),
            user_data,
        })) as *mut c_void;

        let error = unsafe {
            carla_sensor_listen(
                self.raw_sensor_ptr(),
                sensor_callback_wrapper,
                callback_data,
            )
        };
        check_carla_error(error)
    }

    /// Stop listening for sensor data.
    pub fn stop(&self) -> Result<()> {
        let error = unsafe { carla_sensor_stop(self.raw_sensor_ptr()) };
        check_carla_error(error)
    }

    /// Check if the sensor is currently listening.
    pub fn is_listening(&self) -> bool {
        unsafe { carla_sensor_is_listening(self.raw_sensor_ptr()) }
    }

    /// Get sensor calibration data for advanced operations.
    pub fn get_calibration_data(&self) -> Result<SensorCalibrationData> {
        let c_calibration = unsafe { carla_sensor_get_calibration_data(self.raw_sensor_ptr()) };
        if c_calibration.is_null() {
            return Err(anyhow!("No calibration data available for this sensor"));
        }
        Ok(SensorCalibrationData::from_c_data(c_calibration))
    }

    /// Set a sensor attribute by key-value pair.
    pub fn set_attribute(&self, key: &str, value: &str) -> Result<()> {
        let c_key = CString::new(key)?;
        let c_value = CString::new(value)?;

        let error = unsafe {
            carla_sensor_set_attribute(self.raw_sensor_ptr(), c_key.as_ptr(), c_value.as_ptr())
        };
        check_carla_error(error)
    }

    /// Get a sensor attribute by key.
    pub fn get_attribute(&self, key: &str) -> Result<String> {
        let c_key = CString::new(key)?;

        let c_value = unsafe { carla_sensor_get_attribute(self.raw_sensor_ptr(), c_key.as_ptr()) };

        if c_value.is_null() {
            return Err(anyhow!("Attribute '{}' not found", key));
        }

        let value = unsafe { CStr::from_ptr(c_value) }
            .to_string_lossy()
            .into_owned();
        Ok(value)
    }

    /// Get all sensor attributes.
    pub fn get_attributes(&self) -> Result<Vec<SensorAttribute>> {
        // TODO: Implement when C API provides carla_sensor_get_all_attributes
        Err(anyhow!(
            "Sensor attribute enumeration not yet implemented in C API"
        ))
    }

    /// Configure sensor parameters.
    pub fn configure(&self, config: &SensorConfiguration) -> Result<()> {
        for (key, value) in &config.attributes {
            self.set_attribute(key, value)?;
        }
        Ok(())
    }

    /// Destroy this sensor.
    pub fn destroy(self) -> Result<()> {
        self.0.destroy()
    }
}

// Implement ActorBase trait for Sensor
impl ActorBase for Sensor {
    fn raw_ptr(&self) -> *mut carla_actor_t {
        self.0.raw_ptr()
    }

    fn id(&self) -> u32 {
        self.0.id()
    }

    fn type_id(&self) -> String {
        self.0.type_id()
    }

    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }

    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
}

/// Sensor callback wrapper function for C FFI.
unsafe extern "C" fn sensor_callback_wrapper(
    sensor_data: *mut carla_sensor_data_t,
    user_data: *mut c_void,
) {
    if user_data.is_null() || sensor_data.is_null() {
        return;
    }

    let user_data_ptr = user_data as *mut SensorUserData;
    let user_data_ref = &mut *user_data_ptr;

    if let Some(ref mut callback) = user_data_ref.callback {
        // Convert C sensor data to Rust SensorData
        if let Ok(sensor_data_rust) = SensorData::from_raw_ptr(sensor_data) {
            callback(sensor_data_rust);
        }
    }
}

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
    pub(crate) fn from_c_data(_c_data: carla_sensor_calibration_data_t) -> Self {
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

// SAFETY: Sensor wraps a thread-safe C API
unsafe impl Send for Sensor {}
unsafe impl Sync for Sensor {}

// TODO: Temporarily disabled for Phase 5.1 - will be re-enabled in Phase 6
// pub mod data;
// pub mod fusion;
// TODO: Re-enable when C API types are available
// pub mod image_analysis;
// pub mod lidar_analysis;
// pub mod motion;
