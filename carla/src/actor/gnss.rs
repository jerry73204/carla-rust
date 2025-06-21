//! GNSS sensor actor implementation.

use crate::{
    actor::{ActorFfi, Sensor, SensorExt, SensorFfi},
    error::{CarlaResult, SensorError},
    sensor_data::GNSSData,
};

/// GNSS (Global Navigation Satellite System) sensor actor.
#[derive(Debug)]
pub struct GNSS(pub(crate) Sensor);

impl GNSS {
    /// Create a GNSS sensor from a carla-sys SensorWrapper.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        let sensor = Sensor::from_cxx(sensor_wrapper)?;
        Ok(Self(sensor))
    }

    /// Start listening for GNSS data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new GNSS data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(GNSSData) + Send + 'static,
    {
        // TODO: Implement proper callback handling with data transformation
        // This requires FFI support for typed sensor callbacks
        let _callback = callback;
        self.0.start_listening();
        Ok(())
    }

    /// Start listening with an async channel for GNSS data.
    ///
    /// Returns a receiver that yields GNSS data. Requires the `async` feature.
    #[cfg(feature = "async")]
    pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<GNSSData>, SensorError> {
        // TODO: Implement using carla-sys FFI interface
        todo!("GNSS::listen_async not yet implemented with carla-sys FFI")
    }

    /// Get the latest GNSS measurement.
    pub fn data(&self) -> Option<GNSSData> {
        if self.0.has_new_data() {
            let cxx_data = self.as_sensor_ffi().get_last_gnss_data();
            Some(GNSSData::from_cxx(cxx_data))
        } else {
            None
        }
    }
}

impl SensorFfi for GNSS {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement all conversion traits using the macro
crate::impl_sensor_conversions!(GNSS, is_gnss);

#[cfg(test)]
mod tests {

    #[test]
    fn test_gnss_deref_actor_methods() {
        // This test would require a mock GNSS sensor
        // Verify ActorT methods work through Deref
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_gnss_conversions() {
        // Test Actor -> GNSS (should fail for non-GNSS)
        // Test Sensor -> GNSS (should fail for non-GNSS)
        // Test GNSS -> Actor
        // Test GNSS -> Sensor
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_conversion_error_recovery() {
        // Verify failed conversions return original value
    }
}
