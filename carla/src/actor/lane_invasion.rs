//! Lane invasion sensor actor implementation.

use crate::{
    actor::{Sensor, SensorExt, SensorFfi},
    error::{CarlaResult, SensorError},
    sensor_data::LaneInvasionData,
};

/// Lane invasion sensor actor for detecting when a vehicle crosses lane markings.
#[derive(Debug)]
pub struct LaneInvasionSensor(pub(crate) Sensor);

impl LaneInvasionSensor {
    /// Create a LaneInvasionSensor from a carla-sys SensorWrapper.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        let sensor = Sensor::from_cxx(sensor_wrapper)?;
        Ok(Self(sensor))
    }

    /// Start listening for lane invasion data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new lane invasion data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(LaneInvasionData) + Send + 'static,
    {
        // TODO: Implement proper callback handling with data transformation
        // This requires FFI support for typed sensor callbacks
        let _callback = callback;
        self.0.start_listening();
        Ok(())
    }

    //     /// Start listening with an async channel for lane invasion data.
    //     ///
    //     /// Returns a receiver that yields lane invasion data. Requires the `async` feature.
    //     #[cfg(feature = "async")]
    //     pub fn listen_async(
    //         &self,
    //     ) -> Result<tokio::sync::mpsc::Receiver<LaneInvasionData>, SensorError> {
    //         // TODO: Implement using carla-sys FFI interface
    //         todo!("LaneInvasionSensor::listen_async not yet implemented with carla-sys FFI")
    //     }

    /// Get the latest lane invasion event.
    pub fn data(&self) -> Option<LaneInvasionData> {
        if self.0.has_new_data() {
            let cxx_data = self.as_sensor_ffi().get_last_lane_invasion_data();
            Some(LaneInvasionData::from_cxx(cxx_data))
        } else {
            None
        }
    }
}

impl SensorFfi for LaneInvasionSensor {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement all conversion traits using the macro
crate::impl_sensor_conversions!(LaneInvasionSensor, is_lane_invasion);

// Implement ActorExt trait using the macro
crate::impl_sensor_actor_ext!(LaneInvasionSensor);

#[cfg(test)]
mod tests {

    #[test]
    fn test_lane_invasion_sensor_deref_actor_methods() {
        // This test would require a mock lane invasion sensor
        // Verify ActorT methods work through Deref
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_lane_invasion_sensor_conversions() {
        // Test all 6 conversion methods
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_lane_invasion_sensor_listen() {
        // Test callback registration
        // Test data transformation
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_lane_invasion_multiple_listeners() {
        // Verify error handling for multiple listen() calls
    }
}
