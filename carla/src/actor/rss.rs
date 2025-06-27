//! RSS (Road Safety System) sensor actor implementation.

use crate::{
    actor::{Sensor, SensorExt, SensorFfi},
    error::{CarlaResult, SensorError},
    sensor_data::RSSData,
};

/// RSS (Road Safety System) sensor actor.
#[derive(Debug)]
pub struct RSSSensor(pub(crate) Sensor);

impl RSSSensor {
    /// Create an RSSSensor from a carla-sys SensorWrapper.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        let sensor = Sensor::from_cxx(sensor_wrapper)?;
        Ok(Self(sensor))
    }

    /// Start listening for RSS data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new RSS data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(RSSData) + Send + 'static,
    {
        // TODO: Implement proper callback handling with data transformation
        // This requires FFI support for typed sensor callbacks
        let _callback = callback;
        self.0.start_listening();
        Ok(())
    }

    //     /// Start listening with an async channel for RSS data.
    //     ///
    //     /// Returns a receiver that yields RSS data. Requires the `async` feature.
    //     #[cfg(feature = "async")]
    //     pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<RSSData>, SensorError> {
    //         // TODO: Implement using carla-sys FFI interface
    //         todo!("RSSSensor::listen_async not yet implemented with carla-sys FFI")
    //     }

    /// Get the latest RSS response data.
    pub fn data(&self) -> Option<RSSData> {
        if self.0.has_new_data() {
            let cxx_data = self.as_sensor_ffi().get_last_rss_data();
            Some(RSSData::from_cxx(cxx_data))
        } else {
            None
        }
    }

    // RSS-specific configuration methods

    /// Get the ego dynamics mode attribute.
    pub fn ego_dynamics_mode(&self) -> Option<String> {
        self.attribute("ego_dynamics_mode")
    }

    /// Get the other dynamics mode attribute.
    pub fn other_dynamics_mode(&self) -> Option<String> {
        self.attribute("other_dynamics_mode")
    }

    /// Check if RSS debug visualization is enabled.
    pub fn is_debug_enabled(&self) -> Option<bool> {
        self.attribute("visualize_results")
            .and_then(|s| s.parse().ok())
    }
}

// Note: RSS sensor attributes are set during creation through the blueprint.
// Runtime modification of RSS parameters is not supported by CARLA.
// To change sensor parameters, you must destroy the sensor and create a new one
// with the desired configuration.
//
// **WARNING**: The RSS functionality has been removed in CARLA 0.10.0.
// This API is kept for compatibility but is non-functional.

impl SensorFfi for RSSSensor {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement all conversion traits using the macro
crate::impl_sensor_conversions!(RSSSensor, is_rss);

// Implement ActorExt trait using the macro
crate::impl_sensor_actor_ext!(RSSSensor);

// Note: RSS (Road Safety Service) has been removed in CARLA 0.10.0
// These types are kept for API compatibility but are non-functional

#[cfg(test)]
mod tests {

    #[test]
    fn test_rss_sensor_deref_actor_methods() {
        // This test would require a mock RSS sensor
        // Verify ActorT methods work through Deref
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_rss_sensor_conversions() {
        // Test all conversion methods
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_rss_response_data() {
        // Test RSS response data handling
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_rss_configuration() {
        // Test RSS-specific settings
    }
}
