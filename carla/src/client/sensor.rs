use super::{Actor, ActorBase};
use crate::{
    geom::Transform,
    sensor::{SensorAttribute, SensorCalibrationData, SensorConfiguration, SensorData},
    stubs::{
        carla_sensor_get_attribute, carla_sensor_get_calibration_data, carla_sensor_is_listening,
        carla_sensor_listen, carla_sensor_set_attribute, carla_sensor_stop, carla_sensor_t,
    },
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ffi::{c_void, CStr, CString};

/// A sensor actor in the CARLA simulation.
/// This is a newtype wrapper around Actor that provides sensor-specific functionality.
#[derive(Clone, Debug)]
pub struct Sensor(pub Actor);

/// Sensor callback closure type for handling incoming sensor data.
pub type SensorCallback = Box<dyn FnMut(SensorData) + Send + 'static>;

/// Sensor error handler closure type.
pub type SensorErrorHandler = Box<dyn FnMut(String) + Send + 'static>;

/// User data structure for sensor callbacks.
pub struct SensorUserData {
    /// Callback function for sensor data processing.
    pub callback: Option<SensorCallback>,
    /// Custom user data pointer.
    pub user_data: *mut c_void,
}

/// Enhanced user data structure for advanced sensor callbacks.
pub struct EnhancedSensorUserData {
    /// Callback function for sensor data processing.
    pub callback: Option<SensorCallback>,
    /// Error handler for sensor errors.
    pub error_handler: Option<SensorErrorHandler>,
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
        if !unsafe { carla_sys::carla_actor_is_sensor(ptr as *const carla_actor_t) } {
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

    // Advanced sensor management functionality for Phase 3

    /// Start listening with enhanced error handling and user data.
    pub fn listen_with_error_handling<F, E>(&self, callback: F, error_handler: E) -> Result<()>
    where
        F: FnMut(SensorData) + Send + 'static,
        E: FnMut(String) + Send + 'static,
    {
        let user_data = Box::into_raw(Box::new(EnhancedSensorUserData {
            callback: Some(Box::new(callback)),
            error_handler: Some(Box::new(error_handler)),
            user_data: std::ptr::null_mut(),
        })) as *mut c_void;

        let error = unsafe {
            carla_sensor_listen(
                self.raw_sensor_ptr(),
                enhanced_sensor_callback_wrapper,
                user_data,
            )
        };
        check_carla_error(error)
    }

    /// Get sensor-specific configuration helpers.
    pub fn get_sensor_config_helper(&self) -> Result<crate::sensor::SensorConfigHelper> {
        crate::sensor::SensorConfigHelper::new(self.type_id())
    }

    /// Enable sensor data buffering for synchronization.
    pub fn enable_buffering(&self, buffer_size: usize) -> Result<()> {
        // TODO: Implement when C API provides sensor buffering
        self.set_attribute("enable_buffering", "true")?;
        self.set_attribute("buffer_size", &buffer_size.to_string())?;
        Ok(())
    }

    /// Synchronize with other sensors for multi-sensor coordination.
    pub fn synchronize_with(&self, other_sensors: &[&Sensor]) -> Result<()> {
        // TODO: Implement when C API provides sensor synchronization
        let sensor_ids: Vec<String> = other_sensors.iter().map(|s| s.id().to_string()).collect();
        self.set_attribute("sync_sensors", &sensor_ids.join(","))?;
        Ok(())
    }

    /// Get sensor performance metrics.
    pub fn get_performance_metrics(&self) -> Result<crate::sensor::SensorPerformanceMetrics> {
        // TODO: Implement when C API provides performance metrics
        Ok(crate::sensor::SensorPerformanceMetrics::default())
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

/// Enhanced sensor callback wrapper with error handling.
unsafe extern "C" fn enhanced_sensor_callback_wrapper(
    sensor_data: *mut carla_sensor_data_t,
    user_data: *mut c_void,
) {
    if user_data.is_null() {
        return;
    }

    let user_data_ptr = user_data as *mut EnhancedSensorUserData;
    let user_data_ref = &mut *user_data_ptr;

    if sensor_data.is_null() {
        if let Some(ref mut error_handler) = user_data_ref.error_handler {
            error_handler("Received null sensor data".to_string());
        }
        return;
    }

    // Convert C sensor data to Rust SensorData
    match SensorData::from_raw_ptr(sensor_data) {
        Ok(sensor_data_rust) => {
            if let Some(ref mut callback) = user_data_ref.callback {
                callback(sensor_data_rust);
            }
        }
        Err(e) => {
            if let Some(ref mut error_handler) = user_data_ref.error_handler {
                error_handler(format!("Failed to convert sensor data: {}", e));
            }
        }
    }
}

// Note: Sensor doesn't implement Drop because it's a newtype wrapper around Actor,
// and the underlying carla_actor_t will be freed when the Actor is dropped

// SAFETY: Sensor wraps a thread-safe C API
unsafe impl Send for Sensor {}
unsafe impl Sync for Sensor {}
