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
    stubs::carla_sensor_data_type_t_CARLA_SENSOR_DATA_OPTICAL_FLOW,
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
/// This is a newtype wrapper around Actor that provides sensor-specific functionality.
pub struct Sensor(
    pub Actor,
    Arc<Mutex<Option<Box<dyn FnMut(SensorData) + Send + 'static>>>>,
);

impl std::fmt::Debug for Sensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Sensor")
            .field("actor", &self.0)
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

        // Create the base Actor first
        let actor = Actor::from_raw_ptr(ptr as *mut carla_actor_t)?;
        Ok(Self(actor, Arc::new(Mutex::new(None))))
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
        self.0.inner as *mut carla_sensor_t
    }

    /// Start listening for sensor data with a callback.
    pub fn listen<F>(&mut self, callback: F) -> Result<()>
    where
        F: FnMut(SensorData) + Send + 'static,
    {
        // TODO: Implement sensor callback registration when C API is available
        *self.1.lock().unwrap() = Some(Box::new(callback));
        todo!("Sensor listening not yet implemented in C API")
    }

    /// Stop listening for sensor data.
    pub fn stop(&mut self) -> Result<()> {
        // TODO: Implement sensor callback deregistration when C API is available
        *self.1.lock().unwrap() = None;
        todo!("Sensor stop not yet implemented in C API")
    }

    /// Destroy this sensor.
    pub fn destroy(self) -> Result<()> {
        self.0.destroy()
    }

    // TODO: Add more sensor management methods:
    // - get_attributes()
    // - set_attribute()
    // - get_world()
    // - Sensor-specific configuration methods
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

// TODO: Temporarily disabled for Phase 5.1 - will be re-enabled in Phase 6
// pub mod data;
// pub mod fusion;
// TODO: Re-enable when C API types are available
// pub mod image_analysis;
// pub mod lidar_analysis;
// pub mod motion;
