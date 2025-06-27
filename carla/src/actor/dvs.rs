//! DVS (Dynamic Vision Sensor) camera actor implementation.

use crate::{
    actor::{Sensor, SensorExt, SensorFfi},
    error::{CarlaResult, SensorError},
    sensor_data::DVSData,
};

/// DVS (Dynamic Vision Sensor) camera actor.
#[derive(Debug)]
pub struct DVSCamera(pub(crate) Sensor);

impl DVSCamera {
    /// Create a DVSCamera from a carla-sys SensorWrapper.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        let sensor = Sensor::from_cxx(sensor_wrapper)?;
        Ok(Self(sensor))
    }

    /// Start listening for DVS event data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new DVS event data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(DVSData) + Send + 'static,
    {
        // TODO: Implement proper callback handling with data transformation
        // This requires FFI support for typed sensor callbacks
        let _callback = callback;
        self.0.start_listening();
        Ok(())
    }

    //     /// Start listening with an async channel for DVS event data.
    //     ///
    //     /// Returns a receiver that yields DVS event arrays. Requires the `async` feature.
    //     #[cfg(feature = "async")]
    //     pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<DVSData>, SensorError> {
    //         // TODO: Implement using carla-sys FFI interface
    //         todo!("DVSCamera::listen_async not yet implemented with carla-sys FFI")
    //     }

    /// Get the latest DVS event array.
    pub fn data(&self) -> Option<DVSData> {
        if self.0.has_new_data() {
            let cxx_data = self.as_sensor_ffi().get_last_dvs_data();
            Some(DVSData::from_cxx(cxx_data))
        } else {
            None
        }
    }

    // DVS-specific configuration methods

    /// Get the positive threshold.
    pub fn positive_threshold(&self) -> Option<f32> {
        self.attribute("positive_threshold")
            .and_then(|s| s.parse().ok())
    }

    /// Get the negative threshold.
    pub fn negative_threshold(&self) -> Option<f32> {
        self.attribute("negative_threshold")
            .and_then(|s| s.parse().ok())
    }

    /// Get the refractory period in seconds.
    pub fn refractory_period(&self) -> Option<f32> {
        self.attribute("refractory_period_ns")
            .and_then(|s| s.parse::<f32>().ok())
            .map(|ns| ns / 1_000_000_000.0) // Convert nanoseconds to seconds
    }
}

// Note: DVS sensor attributes are set during creation through the blueprint.
// Runtime modification of these parameters is not supported by CARLA.
// To change sensor parameters, you must destroy the sensor and create a new one
// with the desired configuration.

impl SensorFfi for DVSCamera {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement all conversion traits using the macro
crate::impl_sensor_conversions!(DVSCamera, is_dvs);

// Implement ActorExt trait using the macro
crate::impl_sensor_actor_ext!(DVSCamera);

#[cfg(test)]
mod tests {

    #[test]
    fn test_dvs_camera_deref_actor_methods() {
        // This test would require a mock DVS camera
        // Verify ActorT methods work through Deref
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_dvs_camera_conversions() {
        // Test all conversion methods
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_dvs_event_stream() {
        // Test DVS event parsing and callbacks
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_dvs_configuration() {
        // Test DVS-specific settings
    }
}
