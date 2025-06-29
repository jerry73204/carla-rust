//! Radar sensor actor implementation.

use crate::{
    actor::{Sensor, SensorExt, SensorFfi},
    error::{CarlaResult, SensorError},
    sensor_data::RadarData,
};

/// Radar sensor actor.
#[derive(Debug)]
pub struct Radar(pub(crate) Sensor);

impl Radar {
    /// Create a Radar sensor from a carla-sys SensorWrapper.
    pub(crate) fn from_cxx(sensor_wrapper: carla_sys::SensorWrapper) -> CarlaResult<Self> {
        let sensor = Sensor::from_cxx(sensor_wrapper)?;
        Ok(Self(sensor))
    }

    /// Start listening for radar data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new radar data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(RadarData) + Send + 'static,
    {
        // TODO: Implement proper callback handling with data transformation
        // This requires FFI support for typed sensor callbacks
        let _callback = callback;
        self.0.start_listening();
        Ok(())
    }

    //     /// Start listening with an async channel for radar data.
    //     ///
    //     /// Returns a receiver that yields radar data. Requires the `async` feature.
    //     #[cfg(feature = "async")]
    //     pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<RadarData>, SensorError> {
    //         // TODO: Implement using carla-sys FFI interface
    //         todo!("Radar::listen_async not yet implemented with carla-sys FFI")
    //     }

    /// Get the latest radar measurement.
    pub fn data(&self) -> Option<RadarData> {
        if self.0.has_new_data() {
            let cxx_data = self.as_sensor_ffi().get_last_radar_data();
            Some(RadarData::from_cxx(cxx_data))
        } else {
            None
        }
    }

    // Radar-specific configuration methods

    /// Get horizontal field of view in degrees.
    pub fn horizontal_fov(&self) -> Option<f32> {
        self.attribute("horizontal_fov")
            .and_then(|s| s.parse().ok())
    }

    /// Get vertical field of view in degrees.
    pub fn vertical_fov(&self) -> Option<f32> {
        self.attribute("vertical_fov").and_then(|s| s.parse().ok())
    }

    /// Get maximum range in meters.
    pub fn range(&self) -> Option<f32> {
        self.attribute("range").and_then(|s| s.parse().ok())
    }

    /// Get points per second.
    pub fn points_per_second(&self) -> Option<i32> {
        self.attribute("points_per_second")
            .and_then(|s| s.parse().ok())
    }
}

// Note: Radar sensor attributes are set during creation through the blueprint.
// Runtime modification of these parameters is not supported by CARLA.
// To change sensor parameters, you must destroy the sensor and create a new one
// with the desired configuration.

impl SensorFfi for Radar {
    fn as_sensor_ffi(&self) -> &carla_sys::SensorWrapper {
        self.0.as_sensor_ffi()
    }
}

// Implement all conversion traits using the macro
crate::impl_sensor_conversions!(Radar, is_radar);

// Implement ActorExt trait using the macro
crate::impl_sensor_actor_ext!(Radar);

#[cfg(test)]
mod tests {

    #[test]
    fn test_radar_deref_actor_methods() {
        // This test would require a mock radar sensor
        // Verify ActorT methods work through Deref
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_radar_conversions() {
        // Test all conversion methods
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_radar_data_parsing() {
        // Test RadarData parsing with sample data
    }

    #[test]
    #[ignore = "Requires CARLA server connection"]
    fn test_radar_configuration() {
        // Test radar-specific settings
    }
}
