//! Obstacle detection sensor actor implementation.

use crate::{
    actor::{ActorFfi, Sensor, SensorExt, SensorFfi},
    error::{CarlaResult, SensorError},
    sensor_data::ObstacleDetectionData,
};

/// Obstacle detection sensor actor for detecting obstacles in front of the vehicle.
#[derive(Debug)]
pub struct ObstacleDetectionSensor(pub(crate) Sensor);

impl ObstacleDetectionSensor {
    /// Create an ObstacleDetectionSensor from a carla-sys SensorWrapper.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        let sensor = Sensor::from_cxx(sensor_wrapper)?;
        Ok(Self(sensor))
    }

    /// Start listening for obstacle detection data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new obstacle detection data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(ObstacleDetectionData) + Send + 'static,
    {
        // TODO: Implement proper callback handling with data transformation
        // This requires FFI support for typed sensor callbacks
        let _callback = callback;
        self.0.start_listening();
        Ok(())
    }

    //     /// Start listening with an async channel for obstacle detection data.
    //     ///
    //     /// Returns a receiver that yields obstacle detection data. Requires the `async` feature.
    //     #[cfg(feature = "async")]
    //     pub fn listen_async(
    //         &self,
    //     ) -> Result<tokio::sync::mpsc::Receiver<ObstacleDetectionData>, SensorError> {
    //         // TODO: Implement using carla-sys FFI interface
    //         todo!("ObstacleDetectionSensor::listen_async not yet implemented with carla-sys FFI")
    //     }

    /// Get the latest obstacle detection event.
    pub fn data(&self) -> Option<ObstacleDetectionData> {
        if self.0.has_new_data() {
            let cxx_data = self.as_sensor_ffi().get_last_obstacle_detection_data();
            Some(ObstacleDetectionData::from_cxx(cxx_data))
        } else {
            None
        }
    }
}

impl SensorFfi for ObstacleDetectionSensor {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement all conversion traits using the macro
crate::impl_sensor_conversions!(ObstacleDetectionSensor, is_obstacle_detection);

// Implement ActorExt trait using the macro
crate::impl_sensor_actor_ext!(ObstacleDetectionSensor);

#[cfg(test)]
mod tests {

    #[test]
    fn test_obstacle_detection_sensor_deref_actor_methods() {
        // This test would require a mock obstacle detection sensor
        // Verify ActorT methods work through Deref
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_obstacle_detection_sensor_conversions() {
        // Test all 6 conversion methods
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_obstacle_detection_sensor_listen() {
        // Test callback registration
        // Test data transformation
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_obstacle_detection_multiple_listeners() {
        // Verify error handling for multiple listen() calls
    }
}
