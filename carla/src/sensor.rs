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

// Re-export all sensor data types (prefer new data types, supplement with legacy)
pub use data::{
    CollisionEvent, DvsEvent, DvsEventArray, GnssMeasurement as GnssData, Image,
    ImuMeasurement as ImuData, LaneInvasionEvent, LidarMeasurement as LidarData,
    ObstacleDetectionEvent, OpticalFlow as OpticalFlowImage, OpticalFlowPixel,
    RadarMeasurement as RadarData,
};

// Re-export additional types from legacy data module that aren't in the new data module
pub use legacy_data::{
    DvsAnalysis,
    GnssData as LegacyGnssData,
    ImageData,
    ImageRegionOfInterest,
    ImuData as LegacyImuData,
    // Re-export legacy versions with different names to avoid conflicts
    LidarData as LegacyLidarData,
    LidarPoint,
    OpticalFlowAnalysis,
    RadarData as LegacyRadarData,
    RadarDetection,
    SemanticLidarData,
    SemanticLidarPoint,
};

// TODO: Define SensorDataType when sensor type system is finalized
pub type SensorDataType = u32;

// Sensor management
use crate::{
    geom::TransformExt, stubs::carla_sensor_data_type_t_CARLA_SENSOR_DATA_OPTICAL_FLOW,
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Isometry3;
use std::{
    ffi::c_void,
    sync::{Arc, Mutex},
};

/// A sensor actor in the CARLA simulation.
pub struct Sensor {
    pub(crate) inner: *mut carla_sensor_t,
    // Keep track of callbacks to prevent them from being dropped
    _callback_data: Arc<Mutex<Option<Box<dyn FnMut(SensorData) + Send + 'static>>>>,
}

impl std::fmt::Debug for Sensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Sensor")
            .field("inner", &self.inner)
            .field("_callback_data", &"<callback>")
            .finish()
    }
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

        Ok(Self {
            inner: ptr,
            _callback_data: Arc::new(Mutex::new(None)),
        })
    }

    /// Get the sensor's unique ID.
    pub fn id(&self) -> u32 {
        unsafe { carla_actor_get_id(self.inner as *const carla_actor_t) }
    }

    /// Get the sensor's type ID.
    pub fn type_id(&self) -> String {
        let type_id_ptr = unsafe { carla_actor_get_type_id(self.inner as *const carla_actor_t) };
        unsafe { crate::utils::c_string_to_rust(type_id_ptr) }
    }

    /// Get the sensor's current transform.
    pub fn get_transform(&self) -> crate::geom::Transform {
        let c_transform = unsafe { carla_actor_get_transform(self.inner as *const carla_actor_t) };
        crate::geom::Transform::from_c_transform(c_transform)
    }

    /// Set the sensor's transform.
    pub fn set_transform(&self, transform: &crate::geom::Transform) -> Result<()> {
        let c_transform = transform.to_c_transform();
        let error =
            unsafe { carla_actor_set_transform(self.inner as *mut carla_actor_t, &c_transform) };
        check_carla_error(error)
    }

    /// Check if the sensor is still alive in the simulation.
    pub fn is_alive(&self) -> bool {
        unsafe { carla_actor_is_alive(self.inner as *const carla_actor_t) }
    }

    /// Start listening for sensor data with a callback.
    pub fn listen<F>(&mut self, callback: F) -> Result<()>
    where
        F: FnMut(SensorData) + Send + 'static,
    {
        // TODO: Implement sensor callback registration when C API is available
        *self._callback_data.lock().unwrap() = Some(Box::new(callback));
        todo!("Sensor listening not yet implemented in C API")
    }

    /// Stop listening for sensor data.
    pub fn stop(&mut self) -> Result<()> {
        // TODO: Implement sensor callback deregistration when C API is available
        *self._callback_data.lock().unwrap() = None;
        todo!("Sensor stop not yet implemented in C API")
    }

    /// Destroy this sensor.
    pub fn destroy(self) -> Result<()> {
        let error = unsafe { carla_actor_destroy(self.inner as *mut carla_actor_t) };
        check_carla_error(error)
    }

    // TODO: Add more sensor management methods:
    // - get_attributes()
    // - set_attribute()
    // - get_world()
    // - Sensor-specific configuration methods
}

pub mod data;
pub mod fusion;
// TODO: Re-enable when C API types are available
// pub mod image_analysis;
pub mod lidar_analysis;
// pub mod motion;
