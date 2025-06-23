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

    /// Set the positive threshold.
    pub fn set_positive_threshold(&self, threshold: f32) -> CarlaResult<()> {
        // TODO: Implement DVS configuration through FFI
        let _threshold = threshold;
        todo!("DVSCamera::set_positive_threshold not yet implemented - missing FFI function")
    }

    /// Set the negative threshold.
    pub fn set_negative_threshold(&self, threshold: f32) -> CarlaResult<()> {
        // TODO: Implement DVS configuration through FFI
        let _threshold = threshold;
        todo!("DVSCamera::set_negative_threshold not yet implemented - missing FFI function")
    }

    /// Set the refractory period in seconds.
    pub fn set_refractory_period(&self, period: f32) -> CarlaResult<()> {
        // TODO: Implement DVS configuration through FFI
        let _period = period;
        todo!("DVSCamera::set_refractory_period not yet implemented - missing FFI function")
    }
}

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
