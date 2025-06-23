//! IMU sensor actor implementation.

use crate::{
    actor::{Sensor, SensorExt, SensorFfi},
    error::{CarlaResult, SensorError},
    sensor_data::IMUData,
};

/// IMU (Inertial Measurement Unit) sensor actor.
#[derive(Debug)]
pub struct IMU(pub(crate) Sensor);

impl IMU {
    /// Create an IMU sensor from a carla-sys SensorWrapper.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        let sensor = Sensor::from_cxx(sensor_wrapper)?;
        Ok(Self(sensor))
    }

    /// Start listening for IMU data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new IMU data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(IMUData) + Send + 'static,
    {
        // TODO: Implement proper callback handling with data transformation
        // This requires FFI support for typed sensor callbacks
        let _callback = callback;
        self.0.start_listening();
        Ok(())
    }

    //     /// Start listening with an async channel for IMU data.
    //     ///
    //     /// Returns a receiver that yields IMU data. Requires the `async` feature.
    //     #[cfg(feature = "async")]
    //     pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<IMUData>, SensorError> {
    //         // TODO: Implement using carla-sys FFI interface
    //         todo!("IMU::listen_async not yet implemented with carla-sys FFI")
    //     }

    /// Get the latest IMU measurement.
    pub fn data(&self) -> Option<IMUData> {
        if self.0.has_new_data() {
            let cxx_data = self.as_sensor_ffi().get_last_imu_data();
            Some(IMUData::from_cxx(cxx_data))
        } else {
            None
        }
    }
}

impl SensorFfi for IMU {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement all conversion traits using the macro
crate::impl_sensor_conversions!(IMU, is_imu);

// Implement ActorExt trait using the macro
crate::impl_sensor_actor_ext!(IMU);

#[cfg(test)]
mod tests {

    #[test]
    fn test_imu_deref_actor_methods() {
        // This test would require a mock IMU sensor
        // Verify ActorT methods work through Deref
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_imu_conversions() {
        // Test Actor -> IMU (should fail for non-IMU)
        // Test Sensor -> IMU (should fail for non-IMU)
        // Test IMU -> Actor
        // Test IMU -> Sensor
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_conversion_error_recovery() {
        // Verify failed conversions return original value
    }
}
