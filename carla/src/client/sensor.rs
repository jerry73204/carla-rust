//! Sensor actor implementation.

use crate::{
    client::{Actor, ActorId},
    error::{CarlaResult, SensorError},
    geom::{Transform, Vector3D},
    traits::ActorT,
};

/// Sensor actor.
#[derive(Debug)]
pub struct Sensor {
    /// Base actor
    pub actor: Actor,
    // Additional sensor-specific fields
}

impl Sensor {
    /// Create a sensor from a carla-cxx SensorWrapper.
    /// TODO: Need Sensor to Actor conversion in carla-cxx FFI
    pub fn new(_sensor_wrapper: carla_cxx::SensorWrapper) -> Self {
        todo!("Sensor::new - need Sensor to Actor conversion FFI function")
    }

    /// Create a sensor from an actor.
    pub fn from_actor(actor: Actor) -> Self {
        Self { actor }
    }

    /// Get the underlying actor.
    pub fn as_actor(&self) -> &Actor {
        &self.actor
    }

    /// Start listening for sensor data with a callback.
    ///
    /// # Arguments
    /// * `callback` - Function called when new data is available
    ///
    /// # Errors
    /// Returns [`SensorError::AlreadyListening`] if already listening,
    /// or [`SensorError::CallbackFailed`] if callback registration fails.
    pub fn listen<F>(&self, callback: F) -> Result<(), SensorError>
    where
        F: Fn(Vec<u8>) + Send + 'static, // Generic sensor data as bytes
    {
        // TODO: Implement using carla-cxx FFI interface
        let _callback = callback;
        todo!("Sensor::listen not yet implemented with carla-cxx FFI")
    }

    /// Start listening with an async channel for sensor data.
    ///
    /// Returns a receiver that yields sensor data. Requires the `async` feature.
    #[cfg(feature = "async")]
    pub fn listen_async(&self) -> Result<tokio::sync::mpsc::Receiver<Vec<u8>>, SensorError> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Sensor::listen_async not yet implemented with carla-cxx FFI")
    }

    /// Stop listening for sensor data.
    ///
    /// # Errors
    /// Returns [`SensorError::NotListening`] if not currently listening.
    pub fn stop(&self) -> Result<(), SensorError> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Sensor::stop not yet implemented with carla-cxx FFI")
    }

    /// Check if the sensor is currently listening.
    pub fn is_listening(&self) -> bool {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Sensor::is_listening not yet implemented with carla-cxx FFI")
    }

    /// Get the sensor's data frame number (increments with each measurement).
    pub fn get_frame(&self) -> u64 {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Sensor::get_frame not yet implemented with carla-cxx FFI")
    }

    /// Get the timestamp of the last sensor measurement.
    pub fn get_timestamp(&self) -> f64 {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Sensor::get_timestamp not yet implemented with carla-cxx FFI")
    }

    /// Get sensor-specific attribute.
    pub fn get_attribute(&self, name: &str) -> Option<String> {
        // TODO: Implement using carla-cxx FFI interface
        let _name = name;
        todo!("Sensor::get_attribute not yet implemented with carla-cxx FFI")
    }

    /// Enable sensor recording.
    pub fn enable_recording(&self, filename: &str) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _filename = filename;
        todo!("Sensor::enable_recording not yet implemented with carla-cxx FFI")
    }

    /// Disable sensor recording.
    pub fn disable_recording(&self) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Sensor::disable_recording not yet implemented with carla-cxx FFI")
    }
}

impl ActorT for Sensor {
    fn get_id(&self) -> ActorId {
        self.actor.get_id()
    }
    fn get_type_id(&self) -> String {
        self.actor.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.actor.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.actor.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.actor.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.actor.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.actor.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.actor.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.actor.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.actor.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.actor.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.actor.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.actor.add_torque(torque)
    }
}
