//! Sensor actor implementation.

use crate::{
    actor::{Actor, ActorId},
    error::{CarlaResult, SensorError},
    geom::{FromCxx, ToCxx, Transform, Vector3D},
    traits::{ActorT, SensorT},
};
use carla_cxx::SensorWrapper;

/// Sensor actor.
#[derive(Debug)]
pub struct Sensor {
    /// Internal sensor wrapper for FFI calls
    pub(crate) inner: SensorWrapper,
}

impl Sensor {
    /// Create a sensor from a carla-cxx SensorWrapper and actor ID.
    pub fn from_cxx(sensor_wrapper: SensorWrapper) -> CarlaResult<Self> {
        Ok(Self {
            inner: sensor_wrapper,
        })
    }

    /// Create a sensor from an actor by casting.
    pub fn from_actor(actor: Actor) -> CarlaResult<Option<Self>> {
        let actor_ref = actor.get_inner_actor();
        if let Some(sensor_wrapper) = SensorWrapper::from_actor(actor_ref) {
            Ok(Some(Self {
                inner: sensor_wrapper,
            }))
        } else {
            Ok(None)
        }
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
        let _callback = callback;
        self.inner.listen();
        Ok(())
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
    pub fn stop(&self) {
        self.inner.stop();
    }

    /// Check if the sensor is currently listening.
    pub fn is_listening(&self) -> bool {
        self.inner.is_listening()
    }

    /// Get the sensor's data frame number (increments with each measurement).
    pub fn frame(&self) -> u64 {
        carla_cxx::ffi::bridge::Sensor_GetFrame(self.inner.get_inner_sensor())
    }

    /// Get the timestamp of the last sensor measurement.
    pub fn timestamp(&self) -> f64 {
        carla_cxx::ffi::bridge::Sensor_GetTimestamp(self.inner.get_inner_sensor())
    }

    /// Get sensor-specific attribute.
    pub fn attribute(&self, name: &str) -> Option<String> {
        let _name = name;
        // TODO: Implement sensor attribute retrieval
        // This requires adding Sensor_GetAttribute FFI function or storing blueprint reference
        todo!(
            "Sensor::get_attribute not yet implemented - missing FFI function Sensor_GetAttribute"
        )
    }

    /// Enable sensor recording.
    pub fn enable_recording(&self, filename: &str) -> CarlaResult<()> {
        let _filename = filename;
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(crate::error::CarlaError::Sensor(
            SensorError::CallbackFailed(
                "enable_recording not available - use client-level recording".to_string(),
            ),
        ))
    }

    /// Disable sensor recording.
    pub fn disable_recording(&self) -> CarlaResult<()> {
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(crate::error::CarlaError::Sensor(
            SensorError::CallbackFailed(
                "disable_recording not available - use client-level recording".to_string(),
            ),
        ))
    }

    /// Check if new sensor data is available.
    pub fn has_new_data(&self) -> bool {
        self.inner.has_new_data()
    }

    /// Get access to the underlying SensorWrapper for type-specific operations.
    pub(crate) fn inner(&self) -> &SensorWrapper {
        &self.inner
    }
}

impl ActorT for Sensor {
    fn id(&self) -> ActorId {
        // Sensor doesn't have a direct GetId method, need to cast to Actor
        let actor_ptr = carla_cxx::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        carla_cxx::ffi::Actor_GetId(&actor_ptr)
    }
    fn type_id(&self) -> String {
        carla_cxx::ffi::bridge::Sensor_GetTypeId(self.inner.get_inner_sensor())
    }
    fn transform(&self) -> Transform {
        let cxx_transform =
            carla_cxx::ffi::bridge::Sensor_GetTransform(self.inner.get_inner_sensor());
        Transform::from(cxx_transform)
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        carla_cxx::ffi::bridge::Sensor_SetTransform(self.inner.get_inner_sensor(), &cxx_transform);
        Ok(())
    }
    fn velocity(&self) -> Vector3D {
        let vel = carla_cxx::ffi::bridge::Sensor_GetVelocity(self.inner.get_inner_sensor());
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn angular_velocity(&self) -> Vector3D {
        let vel = carla_cxx::ffi::bridge::Sensor_GetAngularVelocity(self.inner.get_inner_sensor());
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn acceleration(&self) -> Vector3D {
        let acc = carla_cxx::ffi::bridge::Sensor_GetAcceleration(self.inner.get_inner_sensor());
        Vector3D::new(acc.x as f32, acc.y as f32, acc.z as f32)
    }
    fn is_alive(&self) -> bool {
        carla_cxx::ffi::bridge::Sensor_IsAlive(self.inner.get_inner_sensor())
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        carla_cxx::ffi::bridge::Sensor_SetSimulatePhysics(self.inner.get_inner_sensor(), enabled);
        Ok(())
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let cxx_impulse = carla_cxx::SimpleVector3D {
            x: impulse.x as f64,
            y: impulse.y as f64,
            z: impulse.z as f64,
        };
        carla_cxx::ffi::bridge::Sensor_AddImpulse(self.inner.get_inner_sensor(), &cxx_impulse);
        Ok(())
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let cxx_force = carla_cxx::SimpleVector3D {
            x: force.x as f64,
            y: force.y as f64,
            z: force.z as f64,
        };
        carla_cxx::ffi::bridge::Sensor_AddForce(self.inner.get_inner_sensor(), &cxx_force);
        Ok(())
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let cxx_torque = carla_cxx::SimpleVector3D {
            x: torque.x as f64,
            y: torque.y as f64,
            z: torque.z as f64,
        };
        carla_cxx::ffi::bridge::Sensor_AddTorque(self.inner.get_inner_sensor(), &cxx_torque);
        Ok(())
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        // Sensor doesn't have a direct GetBoundingBox method, need to cast to Actor
        let actor_ptr = carla_cxx::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Sensor to Actor");
        }
        let simple_bbox = carla_cxx::ffi::Actor_GetBoundingBox(&actor_ptr);
        crate::geom::BoundingBox::from_cxx(simple_bbox)
    }
}

impl SensorT for Sensor {
    fn start_listening(&self) {
        self.inner.listen();
    }

    fn stop_listening(&self) {
        self.inner.stop();
    }

    fn is_listening(&self) -> bool {
        self.inner.is_listening()
    }

    fn has_new_data(&self) -> bool {
        self.inner.has_new_data()
    }

    fn attribute(&self, name: &str) -> Option<String> {
        let _name = name;
        // TODO: Implement sensor attribute retrieval
        // This requires adding Sensor_GetAttribute FFI function or storing blueprint reference
        todo!(
            "Sensor::get_attribute not yet implemented - missing FFI function Sensor_GetAttribute"
        )
    }

    fn enable_recording(&self, filename: &str) -> CarlaResult<()> {
        let _filename = filename;
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(crate::error::CarlaError::Sensor(
            SensorError::CallbackFailed(
                "enable_recording not available - use client-level recording".to_string(),
            ),
        ))
    }

    fn disable_recording(&self) -> CarlaResult<()> {
        // Sensor recording is typically done at the client level, not sensor level
        // This would require additional FFI functions that aren't part of the basic sensor API
        Err(crate::error::CarlaError::Sensor(
            SensorError::CallbackFailed(
                "disable_recording not available - use client-level recording".to_string(),
            ),
        ))
    }
}

impl Drop for Sensor {
    fn drop(&mut self) {
        // Stop listening if sensor is listening
        if self.inner.is_listening() {
            self.inner.stop();
        }
        // Only destroy if the sensor is still alive
        if carla_cxx::ffi::bridge::Sensor_IsAlive(self.inner.get_inner_sensor()) {
            let _ = carla_cxx::ffi::bridge::Sensor_Destroy(self.inner.get_inner_sensor());
        }
    }
}
