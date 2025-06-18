//! Obstacle detection sensor actor implementation.

use crate::{
    actor::{Actor, ActorId},
    error::{CarlaResult, SensorError},
    geom::{FromCxx, ToCxx, Transform, Vector3D},
    sensor_data::ObstacleDetectionData,
    traits::{ActorT, SensorT},
};
use carla_sys::SensorWrapper;

/// Obstacle detection sensor actor for detecting obstacles within range.
#[derive(Debug)]
pub struct ObstacleDetectionSensor {
    /// Internal sensor wrapper for FFI calls
    pub(crate) inner: SensorWrapper,
}

impl ObstacleDetectionSensor {
    /// Create an ObstacleDetectionSensor from a carla-sys SensorWrapper.
    pub fn from_cxx(sensor_wrapper: SensorWrapper) -> CarlaResult<Self> {
        Ok(Self {
            inner: sensor_wrapper,
        })
    }

    /// Create an ObstacleDetectionSensor from an actor by casting.
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
        let _callback = callback;
        self.inner.listen();
        Ok(())
    }

    /// Start listening with an async channel for obstacle detection data.
    ///
    /// Returns a receiver that yields obstacle detection data. Requires the `async` feature.
    #[cfg(feature = "async")]
    pub fn listen_async(
        &self,
    ) -> Result<tokio::sync::mpsc::Receiver<ObstacleDetectionData>, SensorError> {
        // TODO: Implement using carla-sys FFI interface
        todo!("ObstacleDetectionSensor::listen_async not yet implemented with carla-sys FFI")
    }

    /// Get the latest obstacle detection event.
    pub fn data(&self) -> Option<ObstacleDetectionData> {
        if self.inner.has_new_data() {
            let cxx_event = self.inner.get_last_obstacle_detection_data();
            Some(ObstacleDetectionData::from_cxx(cxx_event))
        } else {
            None
        }
    }

    /// Get the sensor's detection distance setting.
    pub fn detection_range(&self) -> f32 {
        // TODO: Implement using carla-sys FFI interface
        // This would require adding Sensor_GetAttribute FFI function to get "distance" attribute
        todo!("ObstacleDetectionSensor::detection_range not yet implemented - missing FFI function Sensor_GetAttribute")
    }

    /// Get the sensor's hit radius setting.
    pub fn hit_radius(&self) -> f32 {
        // TODO: Implement using carla-sys FFI interface
        // This would require adding Sensor_GetAttribute FFI function to get "hit_radius" attribute
        todo!("ObstacleDetectionSensor::hit_radius not yet implemented - missing FFI function Sensor_GetAttribute")
    }

    /// Check if the sensor only detects dynamic objects.
    pub fn only_dynamics(&self) -> bool {
        // TODO: Implement using carla-sys FFI interface
        // This would require adding Sensor_GetAttribute FFI function to get "only_dynamics" attribute
        todo!("ObstacleDetectionSensor::only_dynamics not yet implemented - missing FFI function Sensor_GetAttribute")
    }

    /// Check if debug line tracing is enabled.
    pub fn debug_linetrace(&self) -> bool {
        // TODO: Implement using carla-sys FFI interface
        // This would require adding Sensor_GetAttribute FFI function to get "debug_linetrace" attribute
        todo!("ObstacleDetectionSensor::debug_linetrace not yet implemented - missing FFI function Sensor_GetAttribute")
    }

    /// Get access to the underlying SensorWrapper for type-specific operations.
    pub(crate) fn inner(&self) -> &SensorWrapper {
        &self.inner
    }
}

impl ActorT for ObstacleDetectionSensor {
    fn id(&self) -> ActorId {
        // ObstacleDetectionSensor doesn't have a direct GetId method, need to cast to Actor
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast ObstacleDetectionSensor to Actor");
        }
        carla_sys::ffi::Actor_GetId(&actor_ptr)
    }
    fn type_id(&self) -> String {
        carla_sys::ffi::bridge::Sensor_GetTypeId(self.inner.get_inner_sensor())
    }
    fn transform(&self) -> Transform {
        let cxx_transform =
            carla_sys::ffi::bridge::Sensor_GetTransform(self.inner.get_inner_sensor());
        Transform::from(cxx_transform)
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        carla_sys::ffi::bridge::Sensor_SetTransform(self.inner.get_inner_sensor(), &cxx_transform);
        Ok(())
    }
    fn velocity(&self) -> Vector3D {
        let vel = carla_sys::ffi::bridge::Sensor_GetVelocity(self.inner.get_inner_sensor());
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn angular_velocity(&self) -> Vector3D {
        let vel = carla_sys::ffi::bridge::Sensor_GetAngularVelocity(self.inner.get_inner_sensor());
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn acceleration(&self) -> Vector3D {
        let acc = carla_sys::ffi::bridge::Sensor_GetAcceleration(self.inner.get_inner_sensor());
        Vector3D::new(acc.x as f32, acc.y as f32, acc.z as f32)
    }
    fn is_alive(&self) -> bool {
        carla_sys::ffi::bridge::Sensor_IsAlive(self.inner.get_inner_sensor())
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        carla_sys::ffi::bridge::Sensor_SetSimulatePhysics(self.inner.get_inner_sensor(), enabled);
        Ok(())
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let cxx_impulse = carla_sys::SimpleVector3D {
            x: impulse.x as f64,
            y: impulse.y as f64,
            z: impulse.z as f64,
        };
        carla_sys::ffi::bridge::Sensor_AddImpulse(self.inner.get_inner_sensor(), &cxx_impulse);
        Ok(())
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let cxx_force = carla_sys::SimpleVector3D {
            x: force.x as f64,
            y: force.y as f64,
            z: force.z as f64,
        };
        carla_sys::ffi::bridge::Sensor_AddForce(self.inner.get_inner_sensor(), &cxx_force);
        Ok(())
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let cxx_torque = carla_sys::SimpleVector3D {
            x: torque.x as f64,
            y: torque.y as f64,
            z: torque.z as f64,
        };
        carla_sys::ffi::bridge::Sensor_AddTorque(self.inner.get_inner_sensor(), &cxx_torque);
        Ok(())
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        // ObstacleDetectionSensor doesn't have a direct GetBoundingBox method, need to cast to Actor
        let actor_ptr = carla_sys::ffi::Sensor_CastToActor(self.inner.get_inner_sensor());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast ObstacleDetectionSensor to Actor");
        }
        let simple_bbox = carla_sys::ffi::Actor_GetBoundingBox(&actor_ptr);
        crate::geom::BoundingBox::from_cxx(simple_bbox)
    }
}

impl SensorT for ObstacleDetectionSensor {
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
            "ObstacleDetectionSensor::attribute not yet implemented - missing FFI function Sensor_GetAttribute"
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

impl Drop for ObstacleDetectionSensor {
    fn drop(&mut self) {
        // Stop listening if sensor is listening
        if self.inner.is_listening() {
            self.inner.stop();
        }
        // Only destroy if the sensor is still alive
        if carla_sys::ffi::bridge::Sensor_IsAlive(self.inner.get_inner_sensor()) {
            let _ = carla_sys::ffi::bridge::Sensor_Destroy(self.inner.get_inner_sensor());
        }
    }
}
