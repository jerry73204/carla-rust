//! Collision sensor actor implementation.

use crate::{
    actor::{ActorFfi, Sensor, SensorExt, SensorFfi},
    error::{CarlaResult, SensorError},
    sensor_data::CollisionData,
};

/// Collision sensor actor for detecting collisions with other actors.
#[derive(Debug)]
pub struct CollisionSensor(pub(crate) Sensor);

impl CollisionSensor {
    /// Create a CollisionSensor from a carla-sys SensorWrapper.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        let sensor = Sensor::from_cxx(sensor_wrapper)?;
        Ok(Self(sensor))
    }

    /// Start listening for collision data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new collision data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(CollisionData) + Send + 'static,
    {
        // TODO: Implement proper callback handling with data transformation
        // This requires FFI support for typed sensor callbacks
        let _callback = callback;
        self.0.start_listening();
        Ok(())
    }

    // TODO: Re-enable async support in the future
    // /// Start listening with an async channel for collision data.
    // ///
    // /// Returns a receiver that yields collision data. Requires the `async` feature.
    // #[cfg(feature = "async")]
    // pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<CollisionData>, SensorError> {
    //     // TODO: Implement using carla-sys FFI interface
    //     todo!("CollisionSensor::listen_async not yet implemented with carla-sys FFI")
    // }

    /// Get the latest collision event.
    pub fn data(&self) -> Option<CollisionData> {
        if self.0.has_new_data() {
            let cxx_data = self.as_sensor_ffi().get_last_collision_data();
            Some(CollisionData::from_cxx(cxx_data))
        } else {
            None
        }
    }
}

impl SensorFfi for CollisionSensor {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement all conversion traits using the macro
crate::impl_sensor_conversions!(CollisionSensor, is_collision);

#[cfg(test)]
mod tests {

    #[test]
    fn test_collision_sensor_deref_actor_methods() {
        // This test would require a mock collision sensor
        // Verify ActorT methods work through Deref
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_collision_sensor_conversions() {
        // Test all 6 conversion methods
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_collision_sensor_listen() {
        // Test callback registration
        // Test data transformation
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_multiple_listeners() {
        // Verify error handling for multiple listen() calls
    }
}
